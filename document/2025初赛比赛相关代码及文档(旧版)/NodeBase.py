import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2,LaserScan,Imu
from cv_bridge import CvBridge
from threading import Lock
import cv2
from enum import Enum, auto
# from scipy.signal import savgol_filter
import math
import time
import numpy as np
import open3d as o3d
class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    UNKNOWN = 3

class Direction(Enum):
    UNKNOWN = 0
    Left = 1
    Right = 2
#width 320
#height 180

#二维码
class QRCodeDetector:
    def __init__(self,InnerMatrix,distCoeffs):
        length = 200#单位毫米
        self.objectPoints = np.array([[-length/2,length/2,0],
                                      [length/2,length/2,0],
                                      [length/2,-length/2,0],
                                      [-length/2,-length/2,0]],dtype=np.float32)
        self.InnerMatrix = InnerMatrix
        self.distCoeffs = distCoeffs
        self.qrcoder = cv2.QRCodeDetector()
        self.result = None
    #得到信息
    def detect_qrcode(self,origin_frame): 
        if origin_frame is None:
            print("fail to grab image")
            return

        codeinfo = None
        result_frame = np.copy(origin_frame)
        gray_frame = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2GRAY)
        codeinfo, imagePoints, straight_qrcode = self.qrcoder.detectAndDecode(gray_frame)
        if codeinfo:
            print(f"information: {codeinfo}")
            self.result = codeinfo
            #solvepnp
            rvec = np.zeros((3, 1), dtype=np.float32)
            tvec = np.zeros((3, 1), dtype=np.float32)
    
            success, rvec, tvec = cv2.solvePnP(self.objectPoints, imagePoints, self.InnerMatrix, self.distCoeffs)
            # # 绘制
            # if success:
            #     cv2.drawFrameAxes(result_frame, InnerMatrix, distCoeffs, rvec, tvec, 100)
        else:
            print("fail to detect qrcode!")
        return self.result

#绿色箭头
class ArrowDetector:
    def __init__(self):
        self.convex_hull = None
        self.center = None
        self.target_point = None
        self.direction = Direction.UNKNOWN
        #hsv绿色阈值
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])
        #轮廓拟合越大越粗略
        self.approx_param = 0.03
        # #引导滤波参数
        # guided_filtered_radius = 15 #引导滤波的半径
        # guided_filtered_eps = 10 #引导滤波的平滑度参数
        #面积阈值
        self.min_area = 50
        #距离相等阈值
        self.equal_param = 0.2

    def getGreenFrame(self,origin_frame):
        hsv_frame = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2HSV)
    
        mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)
        mask = mask.astype(np.uint8)
        #图像插值放大，原图分辨率过低
        mask = cv2.resize(mask, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        #mask = cv2.GaussianBlur(mask, (5, 5), 0)
        # #过滤一些纹样
        # guided_filtered_frame = np.zeros_like(g)
        # guided_filtered_frame = cv2.ximgproc.guidedFilter(mask, g, guided_filtered_radius, guided_filtered_eps)
        # cv2.imshow("mask",mask)
        # cv2.waitKey(0)
        return mask

    def SortConvexHull(self):
        self.center = np.mean(self.convex_hull, axis=0)
        center_y = self.center[0][1]
        distances = np.abs(self.convex_hull[:, 0, 1] - center_y)
        sorted_indices = np.argsort(distances)
        return sorted_indices

    def judgeTarget(self,id):
        point = self.convex_hull[id]
        #
        for i in range(1,3):
            id_1 = (id + i - 5) if (id + i) > 4 else (id + i)
            id_2 = (id - i + 5) if (id - i) < 0 else (id - i)
            distance_1 = np.linalg.norm(point - self.convex_hull[id_1])
            distance_2 = np.linalg.norm(point - self.convex_hull[id_2])
            distance_threshold = (distance_1+distance_2)*self.equal_param
            diff_distance = abs(distance_1-distance_2)
            # print(f"distance_1:{distance_1}")
            # print(f"distance_2:{distance_2}")
            # print(f"distance_threshold:{distance_threshold}")
            # print(f"diff_distance:{diff_distance}")
            # print("\n")
            if diff_distance > distance_threshold:
                return False

        return True


    def detect_arrow(self,origin_frame):
        if origin_frame is None:
            print("fail to grab image")
            return

        result_frame = np.copy(origin_frame)
        self.convex_hull = None
        self.center = None
        self.target_point = None
        self.direction = Direction.UNKNOWN
        mask = self.getGreenFrame(origin_frame)
        canny_frame = cv2.Canny(mask, threshold1=20, threshold2=80)
        contours,hierarchy = cv2.findContours(canny_frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("mask",canny_frame)
        # cv2.waitKey(0)
        for contour in contours:
            if self.direction != Direction.UNKNOWN:
                break
            area = cv2.contourArea(contour)
            #perimeter = cv2.arcLength(contour, closed=True)
            if area < self.min_area:
                continue

            epsilon = self.approx_param * cv2.arcLength(contour, True)
            approx_contour = cv2.approxPolyDP(contour, epsilon, True)
            self.convex_hull = cv2.convexHull(approx_contour)
            #print(len(self.convex_hull))
            # 检查凸包顶点数量是否为五个
            if len(self.convex_hull) == 5:
                
                sorted_indices = self.SortConvexHull()#获取索引
            
                for id in sorted_indices:
                    point = self.convex_hull[id]
                    #print(point)
                    if self.judgeTarget(id):
                        self.target_point = self.convex_hull[id]
                        key = cv2.waitKey(1)
                        #print(self.center)
                        center_x = self.center[0][0]
                        self.direction = Direction.Left if self.target_point[0][0] < center_x else Direction.Right
                        break    
                #print("\n")
        # if self.direction != None:
        #     cv2.circle(result_frame,self.target_point[0],5,(0,0,255),-1)
        #     cv2.putText(result_frame, self.direction, self.target_point[0], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # cv2.imshow('image', result_frame)
        # key = cv2.waitKey(1)
        # if key == 'q':
        #     break
        return self.direction

class YellowLightDetector():
    def __init__(self,InnerMatrix,distCoeffs):
        # 3D模型坐标（单位：米）[长0.328m 宽0.22m]
        self.object_points = np.array([[-0.1708, -0.11289, 0.0],   # 左后下
                                       [0.1708, -0.11289, 0.0],    # 右后下
                                       [0.1708, 0.11289, 0.0],     # 右前下
                                       [-0.1708, 0.11289, 0.0]     # 左前下
                                      ], dtype=np.float32)
        
        # HSV阈值(change)
        self.lower_yellow = np.array([28, 110, 80], dtype=np.uint8)
        self.upper_yellow = np.array([32, 180, 220], dtype=np.uint8)
        #轮廓拟合越大越粗略
        self.approx_param = 0.02
        #
        self.min_area = 25
        self.InnerMatrix = InnerMatrix
        self.distCoeffs = distCoeffs
        #self.distance_pub = self.create_publisher(Float32, '/target_distance', 10)
        self._last_distance = 0.0  # 私有变量

    def sort_points(self, points):
        """排序四边形角点"""
        pts = points.reshape(4, 2)
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # 左上
        rect[2] = pts[np.argmax(s)]  # 右下
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # 右上
        rect[3] = pts[np.argmax(diff)]  # 左下
        return rect

    def detect_distance(self, origin_frame):
        if origin_frame is None:
            print("fail to grab image")
            return
        # 颜色
        hsv = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,15))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        distance = None
        for cnt in contours:
            if cv2.contourArea(cnt) < self.min_area:
                continue
                
            epsilon = self.approx_param * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # 必须是四边形
            if len(approx) != 4:
                continue
                
            # 排序角点
            try:
                image_points = self.sort_points(approx)
            except:
                continue
                
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points.astype(np.float32),
                self.InnerMatrix,
                self.distCoeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if success:
                distance = np.linalg.norm(tvec[2])
                # # 绘制结果
                # for pt in image_points:
                #     cv2.circle(frame, tuple(pt.astype(int)), 5, (0,255,0), -1)
                # cv2.putText(frame, f"Dist: {distance:.2f}m", 
                #            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                # 只处理第一个有效目标
                break  
                
        # # 显示结果
        # cv2.imshow("Detection", frame)
        # cv2.waitKey(1)
        return distance

class LimitHeightDetector(Node):
    def __init__(self,InnerMatrix,distCoeffs):
        self.object_points = np.array([
            [-0.55, -0.05, 0.0],  # 左下角
            [0.55, -0.05, 0.0],   # 右下角
            [0.55, 0.05, 0.0],    # 右上角
            [-0.55, 0.05, 0.0]    # 左上角
            ], dtype=np.float32)

        # 相机参数（以Gazebo 320x180为例）
        self.InnerMatrix = InnerMatrix
        self.distCoeffs = distCoeffs  

        self.min_area = 50
        #轮廓拟合越大越粗略
        self.approx_param = 0.02
        #
        self.lower_gray = np.array([80, 14, 32], dtype=np.uint8) 
        self.upper_gray = np.array([100, 45, 65], dtype=np.uint8)
        self._last_distance = 0.0  # 最后一次计算的距离，私有变量

    def sort_horizontal(self,points):
        pts = points.reshape(-1, 2)
        x_sort = pts[pts[:, 0].argsort()]
        if len(x_sort) >= 2:
            left = x_sort[:2][x_sort[:2, 1].argsort()]
            right = x_sort[-2:][x_sort[-2:, 1].argsort()]
            return np.array([left[0], left[1], right[1], right[0]], dtype=np.float32)
        else:
            return None

    def detect_distance(self, origin_frame):
        hsv = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_gray, self.upper_gray)

        # 形态学处理（横向强化）
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 1))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, horizontal_kernel)
        #cleaned = cv2.dilate(cleaned, horizontal_kernel, iterations=2)
        #cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, horizontal_kernel, iterations=2)

        # 轮廓检测
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_length = 0
        best_target = None
        distance = None

        # 遍历所有轮廓
        for cnt in contours:
            if cv2.contourArea(cnt) < self.min_area:
                continue

            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w < h:
                w, h = h, w

            if not (5.0 < (w / h) < 13.0):
                continue

            if w > max_length:
                max_length = w
                best_target = cnt

        # 处理最优目标
        if best_target is not None:
            epsilon = self.approx_param * cv2.arcLength(best_target, True)
            approx = cv2.approxPolyDP(best_target, epsilon, True)

            image_points = self.sort_horizontal(approx)

            if image_points is not None:
                try:
                    success, rvec, tvec = cv2.solvePnP(
                        self.object_points,
                        image_points.astype(np.float32),
                        self.InnerMatrix,
                        self.distCoeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if success:
                        distance = np.linalg.norm(tvec[2])
                        # for pt in image_points:
                        #     cv2.circle(frame, tuple(pt.astype(int)), 5, (0, 255, 0), -1)
                        # cv2.putText(frame, f"Dist: {distance:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                except Exception as e:
                    print(f"Error in solvePnP: {e}")
            
        return distance      
        # # 显示结果
        # cv2.imshow("Detection", frame)
        # cv2.waitKey(1)
        # return distance

    
class RGBCameraNode(Node):
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.is_node_running = False
        self.bridge = CvBridge()
        self.data_lock = Lock()
        self.frame = None
        self.update = False
        #相机内参
        self.InnerMatrix = np.array([[175.25,0,160],[0,175.25,90],[0,0,1]],dtype=np.float32)
        self.distCoeffs = np.array([0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        #相机外参
        self.cameraToRobot = [275.76,0,125.794]#单位毫米
        #订阅
        self.subscriber = self.create_subscription(Image,"rgb_camera/image_raw",self.ImageCallBack,qos_profile)
        #箭头
        self.ArrowDetector = ArrowDetector()
        #二维码
        self.QRCodeDetector = QRCodeDetector(self.InnerMatrix,self.distCoeffs)
        #黄灯
        self.YellowLightDetector = YellowLightDetector(self.InnerMatrix,self.distCoeffs)
        #限高杆
        self.LimitHeightDetector = LimitHeightDetector(self.InnerMatrix,self.distCoeffs)
        #self.subscriber

    #返回箭头方向
    def get_arrow_direction(self):
        real_answer = [0,0,0]
        count = 5
        for i in range(count):
            frame = self.get_new_frame()
            direction = self.ArrowDetector.detect_arrow(frame)
            real_answer[direction.value] += 1
        
        max_index = real_answer.index(max(real_answer))
        most_frequent_direction = Direction(max_index)
        if most_frequent_direction == Direction.UNKNOWN:
            print("Direction UNKNOWN!")

        return most_frequent_direction

    #返回二维码信息
    def get_qrcode(self):
        frame = self.get_new_frame()
        answer = self.QRCodeDetector.detect_qrcode(frame)
        if answer == None:
            print("qrcode UNKNOWN!")
        return answer
    #返回黄灯距离
    def get_distance_YellowLight(self):
        frame = self.get_new_frame()
        distance = self.YellowLightDetector.detect_distance(frame)
        if distance == None:
            print("YellowLight distance UNKNOWN!")
        return distance
    #返回限高杆距离
    def get_distance_LimitHeight(self):
        frame = self.get_new_frame()
        distance = self.LimitHeightDetector.detect_distance(frame)
        if distance == None:
            print("LimitHeight distance UNKNOWN!")    
        return distance

    def get_new_frame(self):
        while(True):
            if self.update == False:
                time.sleep(0.1)
            else:
                with self.data_lock:
                    self.update = False
                    return self.frame
                

    def ImageCallBack(self,receive_msg):
        if(self.is_node_running):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(receive_msg, desired_encoding='bgr8')
                with self.data_lock:
                    self.frame = cv_image
                    self.update = True

            except Exception as e:
                self.get_logger().error(f"Exception in callback: {e}")
                self.is_node_running = False
        else:
            pass
            
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10)
        self.frame = None
        self.update = False
        #相机内参
        self.InnerMatrix = np.array([[175.25,0,160],[0,175.25,90],[0,0,1]],dtype=np.float32)
        self.distCoeffs = np.array([0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        #相机外参
        self.cameraToRobot = [275.76,0,125.794]#单位毫米
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile)
        self.ArrowDetector = ArrowDetector()
        self.QRCodeDetector = QRCodeDetector(self.InnerMatrix,self.distCoeffs)
        self.YellowLightDetector = YellowLightDetector(self.InnerMatrix,self.distCoeffs)
        self.LimitHeightDetector = LimitHeightDetector(self.InnerMatrix,self.distCoeffs)
        self.lower_yellow = np.array([20, 100, 100])  # HSV阈值下限
        self.upper_yellow = np.array([30, 255, 255])  # HSV阈值上限
        self._yaw_adjust = 0.0 
        self._max_side_length = 0
        self._line_angle = 0.0  # 检测到的线条角度（弧度）
        self._alignment_start_time = 0.0  # 对齐开始时间戳
        self._aligned_duration = 0.0      # 已对齐持续时间
        self._distance = None
        self._lock = Lock()
        self._last_detection_time = 0
        self.thresholds = {
            '0': 0.2,
            '6': 0.2,
            '12': 0.2,
            '14': 0.2,
            '20': 0.2,
            '22': 0.2,
            '24': 0.2,
            '26': 0.2,
            '28': 0.2,
            '30': 0.2,
            '32': 0.2,
            '34': 0.2,
            '37': 0.2,
            '39': 0.2,
            '43': 0.2,
            '46': 0.2,
            '49': 0.2,
            '51': 0.2,
            '52': 0.2,
        }
        self.line_thresholds = {
            '5': 0.2,
            '8': 0.2,
            '13': 0.2,
            '21': 0.2,
            '23': 0.2,
            '27': 0.2,
            '29': 0.2,
            '33': 0.2,
            '38': 0.2,
            '42': 0.2,
            '45': 0.2,
        }
        self.detect_times = {
            '0': 0.5,
            '5': 0.5,
            '6': 0.5,
            '8': 0.5,
            '12': 0.5,
            '13': 0.5,
            '14': 0.5,
            '20': 0.5,
            '21': 0.5,
            '22': 0.5,
            '23': 0.5,
            '24': 0.5,
            '26': 0.5,
            '27': 0.5,
            '28': 0.5,
            '29': 0.5,
            '30': 0.5,
            '32': 0.5,
            '33': 0.5,
            '34': 0.5,
            '37': 0.5,
            '38': 0.5,
            '39': 0.5,
            '42': 0.5,
            '43': 0.5,
            '45': 0.5,
            '46': 0.5,
            '49': 0.5,
            '51': 0.5,
            '52': 0.5,
        }
        self._detected_flags = {
            '0': False,
            '2': False,
            '3': False,
            '4': False,
            '5': False,
            '6': False,
            '8': False,
            '9': False,
            '10': False,
            '12': False,
            '13': False,
            '14': False,
            '15': False,
            '16': False,
            '18': False,
            '19': False,
            '20': False,
            '21': False,
            '22': False,
            '23': False,
            '24': False,
            '26': False,
            '27': False,
            '28': False,
            '29': False,
            '30': False,
            '31': False,
            '32': False,
            '33': False,
            '34': False,
            '35': False,
            '36': False,
            '37': False,
            '38': False,
            '39': False,
            '40': False,
            '41': False,
            '42': False,
            '43': False,
            '45': False,
            '46': False,
            '47': False,
            '49': False,
            '50': False,
            '51': False,
            '52': False,
        }
        # 调试窗口（实际部署时可关闭）
        self.debug_mode = True

    def image_callback(self, msg):
        global state_id, QR1, QR2
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = cv_image.copy()  # 保存当前帧
            current_state = state_id  # 获取当前状态
            if current_state == 0:
                self._detect_0(cv_image)
            elif current_state == 1 or current_state == 7 or current_state == 11 or current_state == 17 or current_state == 2 or current_state == 31 or current_state == 44 or current_state == 48: 
                self._show(cv_image)
            elif current_state == 26:
                self._detect_2(cv_image)
            elif current_state == 3:
                self._detect_3(cv_image)
            elif current_state == 4:
                self._detect_4(cv_image)
            elif current_state == 5:
                self._detect_5(cv_image)
            elif current_state == 6:
                self._detect_6(cv_image)
            elif current_state == 8:
                self._detect_8(cv_image)
            elif current_state == 9:
                self._detect_9(cv_image)
            elif current_state == 10:
                self._detect_10(cv_image)
            elif current_state == 12:
                self._detect_12(cv_image)
            elif current_state == 13:
                self._detect_13(cv_image)
            elif current_state == 14:
                self._detect_14(cv_image)
            elif current_state == 15:
                self._detect_15(cv_image)
            elif current_state == 16:
                self._detect_16(cv_image)
            elif current_state == 18:
                self._detect_18(cv_image)
            elif current_state == 19:
                self._detect_19(cv_image)
            elif current_state == 20:
                self._detect_20(cv_image)
            elif current_state == 21:
                self._detect_21(cv_image)
            elif current_state == 22:
                self._detect_22(cv_image)
            elif current_state == 23:
                self._detect_23(cv_image)
            elif current_state == 24:
                self._detect_24(cv_image)
            elif current_state == 26:
                self._detect_26(cv_image)
            elif current_state == 27:
                self._detect_27(cv_image)
            elif current_state == 28:
                self._detect_28(cv_image)
            elif current_state == 29:
                self._detect_29(cv_image)
            elif current_state == 30:
                self._detect_30(cv_image)
            elif current_state == 32:
                self._detect_32(cv_image)
            elif current_state == 33:
                self._detect_33(cv_image)
            elif current_state == 34:
                self._detect_34(cv_image)
            elif current_state == 35:
                self._detect_35(cv_image)
            elif current_state == 36:
                self._detect_36(cv_image)
            elif current_state == 37:
                self._detect_37(cv_image)
            elif current_state == 38:
                self._detect_38(cv_image)
            elif current_state == 39:
                self._detect_39(cv_image)
            elif current_state == 40:
                self._detect_40(cv_image)
            elif current_state == 41:
                self._detect_41(cv_image)
            elif current_state == 42:
                self._detect_42(cv_image)
            elif current_state == 43:
                self._detect_43(cv_image)
            elif current_state == 45:
                self._detect_45(cv_image)
            elif current_state == 46:
                self._detect_46(cv_image)
            elif current_state == 47:
                self._detect_47(cv_image)
            elif current_state == 49:
                self._detect_49(cv_image)
            elif current_state == 50:
                self._detect_50(cv_image)
            elif current_state == 51:
                self._detect_51(cv_image)
            elif current_state == 52:
                self._detect_52(cv_image)
            else:
                pass
            
        except Exception as e:
            self.get_logger().error(f"图像处理失败: {str(e)}")
    
    def _show(self, cv_image):
        if self.debug_mode:
            cv2.imshow("Detection Preview", cv_image)
            cv2.waitKey(1)

    def show_line(self, cleaned, lines, angle_diff, x1, y1, x2, y2):
        debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
        if lines is not None:
            cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
            angle_deg = np.degrees(angle_diff)
            cv2.putText(debug_img, 
                        f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.imshow("Detection Preview", debug_img)
        cv2.waitKey(1)

    def show_area(self, cv_image, x_start, y_start, x_end, y_end):
        debug_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.rectangle(debug_img, 
                        (x_start, y_start),
                        (x_end, y_end),
                        (0,255,0), 2)
        cv2.imshow("Detection Preview", debug_img)
        cv2.waitKey(1)

    def find_yellow_area(self, cv_image, state, x_start, y_start, x_end, y_end):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        roi = hsv[y_start:y_end, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.thresholds[state]:
                if current_time - self._last_detection_time < self.detect_times[state]:
                    self._detected_flags[state] = True
                else:
                    self._detected_flags[state] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags[state] = False
        
        if self.debug_mode:
            self.show_area(cv_image, x_start, y_start, x_end, y_end)

    def find_QR_block(self, cv_image, state):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        # 设置上半部分ROI（取上半1/3区域）
        roi_height = height // 2
        roi = hsv[0:roi_height, 0:width]
        
        # 深色块的HSV阈值（示例为低明度范围，可根据实际调整）
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 60])
        
        mask = cv2.inRange(roi, lower_dark, upper_dark)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center_x = None
        if len(contours) > 0:
            # 找最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 100:  # 过滤噪声
                M = cv2.moments(max_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    # 计算相对于图像中心的偏移量
                    img_center_x = width // 2
                    offset = center_x - img_center_x
                    yaw_gain = 0.01  # 调整系数
                    yaw = -offset * yaw_gain  # 偏移越大，转向越急
                    with self._lock:
                        self._detected_flags[state] = True
                        self._yaw_adjust = yaw  # 存储yaw调整值

                    if self.debug_mode:
                        debug_img = cv2.cvtColor(roi, cv2.COLOR_HSV2BGR)
                        cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
                        cv2.circle(debug_img, (center_x, center_y), 5, (0,0,255), -1)
                        cv2.line(debug_img, (img_center_x,0), (img_center_x,roi_height), (255,0,0), 2)
                        cv2.imshow("Detection Preview", debug_img)
                        cv2.waitKey(1)
                        
        # 未检测到有效目标时重置
        with self._lock:
            self._detected_flags[state] = False
            self._yaw_adjust = 0.0

    def approach_QR(self, cv_image, state):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 60])
        mask = cv2.inRange(hsv, lower_dark, upper_dark)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_side = 0
        
        if len(contours) > 0:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
            
            for contour in contours:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if 4 <= len(approx) <= 6:
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int_(box)
                    
                    side1 = np.linalg.norm(box[0]-box[1])
                    side2 = np.linalg.norm(box[1]-box[2])
                    current_max_side = max(side1, side2)
                    
                    aspect_ratio = max(side1, side2) / (min(side1, side2) + 1e-5)
                    if 0.8 < aspect_ratio < 1.2: 
                        if current_max_side > max_side:
                            max_side = current_max_side
                            # print(f"检测到边长: {max_side:.1f}")
                            max_contour = contour
        
        with self._lock:
            if max_side > 46:  # 边长阈值
                self._max_side_length = max_side
                self._detected_flags[state] = True
            else:
                self._max_side_length = 0
                self._detected_flags[state] = False

        # 调试显示
        if self.debug_mode and 'max_contour' in locals():
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
            cv2.putText(debug_img, f"Quad Side: {max_side:.1f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def direction_adjust(self, cv_image, state):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        # 形态学处理
        kernel = np.ones((5,5), np.uint8)
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 边缘检测
        edges = cv2.Canny(cleaned, 50, 150)
        # 霍夫直线检测
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 
                               threshold=50, 
                               minLineLength=100, 
                               maxLineGap=10)
        current_time = time.time()
        with self._lock:
            self._detected_flags[state] = False
            self._line_angle = 0.0
            if lines is not None:
                # 取最长线段
                longest_line = max(lines, key=lambda x: np.linalg.norm(x[0][2:]-x[0][:2]))
                x1, y1, x2, y2 = longest_line[0]
                # 计算线段角度（相对于水平轴）
                delta_x = x2 - x1
                delta_y = y2 - y1
                if delta_x == 0:  # 垂直情况
                    line_angle = np.pi/2
                else:
                    line_angle = np.arctan(delta_y / delta_x)
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags[state] = True
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self.line_thresholds[state]:
                    if self._alignment_start_time == 0:
                        self._alignment_start_time = current_time
                    self._aligned_duration = current_time - self._alignment_start_time
                else:
                    self._alignment_start_time = 0.0
                    self._aligned_duration = 0.0
            else:
                self._alignment_start_time = 0.0
                self._aligned_duration = 0.0
            # 调试显示
            if self.debug_mode:
                self.show_line(cleaned, lines, angle_diff, x1, y1, x2, y2)

    def find_contours(self, cv_image, state):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        # 形态学处理去噪
        kernel = np.ones((5,5), np.uint8)
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 查找连通域
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]  # 过滤小噪声
        # 更新检测状态
        with self._lock:
            self._detected_flags[state] = (len(valid_contours) == 1)  # 连通域数量为1时触发
        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.drawContours(debug_img, valid_contours, -1, (0,255,0), 2)
            cv2.putText(debug_img, f"Yellow Regions: {len(valid_contours)}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def stay_mid(self, cv_image, state):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()
        KP = 1.8
        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[:, roi_start_x:roi_end_x]
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            edges = cv2.Canny(mask, 30, 80)
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180,
                                threshold=30, minLineLength=40, maxLineGap=30)
            nearest_line = None
            min_distance = float('inf')
            center_line_x = width // 2  # 全局中心线

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    gx1 = roi_start_x + x1
                    gx2 = roi_start_x + x2
                    gy1, gy2 = y1, y2
                    mid_x = (gx1 + gx2) / 2
                    mid_y = (gy1 + gy2) / 2
                    
                    distance = abs(mid_x - center_line_x)
                    
                    if distance < min_distance:
                        min_distance = distance
                        nearest_line = ((gx1, gy1, gx2, gy2), mid_x)

            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None
        
        # 左右检测区域参数
        detect_width = width // 3
        left_angle = find_nearest_line(center_x - width//6, detect_width, (255,0,255))  # 左检测区中心
        right_angle = find_nearest_line(center_x + width//6, detect_width, (255,0,255)) # 右检测区中心
        # 计算偏航误差
        yaw_error = 0.0
        valid_angles = []
        if left_angle is not None: valid_angles.append(left_angle)
        if right_angle is not None: valid_angles.append(right_angle)
        
        if len(valid_angles) >= 1:
            avg_angle = np.mean(valid_angles)
            # 目标角度应为0（与图像水平轴平行）
            yaw_error = avg_angle

        with self._lock:
            self._detected_flags[state] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * KP, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
        
    def _detect_0(self, cv_image):
        height, width = cv_image.shape[:2]
        x_start = width//2 + width//8
        x_end = x_start + width//4
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '0', x_start, y_start, x_end, y_end)
    
    def _detect_2(self, cv_image):
        self.find_QR_block(cv_image, '2')
    
    def _detect_3(self, cv_image):
        self.approach_QR(cv_image, '3')

    def _detect_4(self, cv_image):
        qr_data = self.QRCodeDetector.detect_qrcode(cv_image)
        with self._lock:
            self._detected_flags['4'] = (qr_data is not None)
    
    def _detect_5(self, cv_image):
        self.direction_adjust(cv_image, '5')
    
    def _detect_6(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '6', x_start, y_start, x_end, y_end)

    def _detect_8(self, cv_image):
        self.direction_adjust(cv_image, '8')
    
    def _detect_9(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '9', x_start, y_start, x_end, y_end)
    
    def _detect_10(self, cv_image): # 灰色，要改
        height, width = cv_image.shape[:2]
        roi_height = 48
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        y_start = height - roi_height
        y_end = height
        self.find_yellow_area(cv_image, '10', x_start, y_start, x_end, y_end)
    
    def _detect_12(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = width//8 - 10
        x_end = x_start + width//8
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '12', x_start, y_start, x_end, y_end)

    def _detect_13(self, cv_image):
        self.direction_adjust(cv_image, '13')
    
    def _detect_14(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '14', x_start, y_start, x_end, y_end)
    
    def _detect_15(self, cv_image):
        self.find_contours(cv_image, '15')

    def _detect_16(self, cv_image):
        global arrow_direction, arrow_lock
        direction = self.ArrowDetector.detect_arrow(cv_image)
        with arrow_lock:
            arrow_direction = direction
        
        # 调试显示
        if self.debug_mode and direction != Direction.UNKNOWN:
            debug_img = cv_image.copy()
            cv2.putText(debug_img, f"Arrow: {direction.name}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_18(self, cv_image):
        if arrow_direction == Direction.Left:
            self.stay_mid(cv_image, '18')
        else:
            pass # 上坡
    
    def _detect_19(self, cv_image):
        if arrow_direction == Direction.Left:# 限高杆
            distance = self.LimitHeightDetector.detect_distance(cv_image)
        else:# 黄灯
            distance = self.YellowLightDetector.detect_distance(cv_image)
        with self._lock:
            if distance is not None:
                self._detected_flags['19'] = True
                self._distance = distance
            else:
                self._detected_flags['19'] = False
                self._distance = None
        if self.debug_mode:
            debug_img = cv_image.copy()
            if distance is not None:
                # 绘制距离信息
                cv2.putText(debug_img, f"Distance: {distance:.2f}m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 显示窗口
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_20(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        if arrow_direction == Direction.Left:
            x_start = 3 * width // 4
            x_end = width
        else:
            x_start = 0
            x_end = width // 4
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '20', x_start, y_start, x_end, y_end)
    
    def _detect_21(self, cv_image):
        self.direction_adjust(cv_image, '21')
    
    def _detect_22(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '22', x_start, y_start, x_end, y_end)
    
    def _detect_23(self, cv_image):
        self.direction_adjust(cv_image, '23')

    def _detect_24(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '24', x_start, y_start, x_end, y_end)
    
    def _detect_26(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        if QR2 == 'B-1':
            x_start = 3 * width // 4
            x_end = width
        else:
            x_start = 0
            x_end = width // 4
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '26', x_start, y_start, x_end, y_end)
    
    def _detect_27(self, cv_image):
        self.direction_adjust(cv_image, '27')

    def _detect_28(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '28', x_start, y_start, x_end, y_end)
    
    def _detect_29(self, cv_image):
        self.direction_adjust(cv_image, '29')
    
    def _detect_30(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '30', x_start, y_start, x_end, y_end)
    
    def _detect_32(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        if QR2 == 'B-1':
            x_start = 0
            x_end = width // 4
        else:
            x_start = 3 * width // 4
            x_end = width
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '32', x_start, y_start, x_end, y_end)
    
    def _detect_33(self, cv_image):
        self.direction_adjust(cv_image, '33')
    
    def _detect_34(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '34', x_start, y_start, x_end, y_end)
        
    def _detect_35(self, cv_image):
        if arrow_direction == Direction.Right:# 限高杆
            distance = self.LimitHeightDetector.detect_distance(cv_image)
        else:# 黄灯
            distance = self.YellowLightDetector.detect_distance(cv_image)
        with self._lock:
            if distance is not None:
                self._detected_flags['35'] = True
                self._distance = distance
            else:
                self._detected_flags['35'] = False
                self._distance = None
        if self.debug_mode:
            debug_img = cv_image.copy()
            if distance is not None:
                # 绘制距离信息
                cv2.putText(debug_img, f"Distance: {distance:.2f}m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 显示窗口
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_36(self, cv_image):
        if arrow_direction == Direction.Right:
            self.stay_mid(cv_image, '36')
        else:
            pass # 上坡
    
    def _detect_37(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        if arrow_direction == Direction.Left:
            x_start = 3 * width // 4
            x_end = width
        else:
            x_start = 0
            x_end = width // 4
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '37', x_start, y_start, x_end, y_end)

    def _detect_38(self, cv_image):
        self.direction_adjust(cv_image, '38')

    def _detect_39(self, cv_image):# 位置待调整
        height, width = cv_image.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        if arrow_direction == Direction.Left:
            x_start = 3 * width // 4
            x_end = width
        else:
            x_start = 0
            x_end = width // 4
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '39', x_start, y_start, x_end, y_end)
    
    def _detect_40(self, cv_image):
        self.find_QR_block(cv_image, '40')
    
    def _detect_41(self, cv_image):
        self.approach_QR(cv_image, '41')
    
    def _detect_42(self, cv_image):
        self.direction_adjust(cv_image, '42')
    
    def _detect_43(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '43', x_start, y_start, x_end, y_end)

    def _detect_45(self, cv_image):
        self.direction_adjust(cv_image, '45')
    
    def _detect_46(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '46', x_start, y_start, x_end, y_end)
    
    def _detect_47(self, cv_image): # 灰色，要改
        height, width = cv_image.shape[:2]
        roi_height = 48
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        y_start = height - roi_height
        y_end = height
        self.find_yellow_area(cv_image, '47', x_start, y_start, x_end, y_end)
    
    def _detect_49(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = width//8 - 10
        x_end = x_start + width//8
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '49', x_start, y_start, x_end, y_end)

    def _detect_50(self, cv_image):
        self.direction_adjust(cv_image, '50')
    
    def _detect_51(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '51', x_start, y_start, x_end, y_end)
    
    def _detect_52(self, cv_image):# 待调整
        height, width = cv_image.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        y_start = height - roi_height1
        y_end = height - roi_height2
        self.find_yellow_area(cv_image, '52', x_start, y_start, x_end, y_end)
    
    @property
    def _0_detected(self):
        with self._lock:
            return self._detected_flags['0']
    
    @property
    def _2_detected(self):
        with self._lock:
            return self._detected_flags['2']
    
    @property
    def _3_detected(self):
        with self._lock:
            return self._detected_flags['3']

    @property
    def _4_detected(self):
        with self._lock:
            return self._detected_flags['4']
        
    @property
    def _5_detected(self):
        with self._lock:
            return self._detected_flags['5']
        
    @property
    def _6_detected(self):
        with self._lock:
            return self._detected_flags['6']

    @property
    def _8_detected(self):
        with self._lock:
            return self._detected_flags['8']

    @property
    def _9_detected(self):
        with self._lock:
            return self._detected_flags['9']

    @property
    def _10_detected(self):
        with self._lock:
            return self._detected_flags['10']
    
    @property
    def _12_detected(self):
        with self._lock:
            return self._detected_flags['12']
        
    @property
    def _13_detected(self):
        with self._lock:
            return self._detected_flags['13']

    @property
    def _14_detected(self):
        with self._lock:
            return self._detected_flags['14']
           
    @property
    def _15_detected(self):
        with self._lock:
            return self._detected_flags['15']

    @property
    def _16_detected(self):
        with self._lock:
            return self._detected_flags['16']
    
    @property
    def _18_detected(self):
        with self._lock:
            return self._detected_flags['18']
    
    @property
    def _19_detected(self):
        with self._lock:
            return self._detected_flags['19']

    @property
    def _20_detected(self):
        with self._lock:
            return self._detected_flags['20']
    
    @property
    def _21_detected(self):
        with self._lock:
            return self._detected_flags['21']

    @property
    def _22_detected(self):
        with self._lock:
            return self._detected_flags['22']
         
    @property
    def _23_detected(self):
        with self._lock:
            return self._detected_flags['23']
    
    @property
    def _24_detected(self):
        with self._lock:
            return self._detected_flags['24']

    @property
    def _25_detected(self):
        with self._lock:
            return self._detected_flags['25']
    
    @property
    def _26_detected(self):
        with self._lock:
            return self._detected_flags['26']
    
    @property
    def _27_detected(self):
        with self._lock:
            return self._detected_flags['27']
    
    @property
    def _28_detected(self):
        with self._lock:
            return self._detected_flags['28']
    
    @property
    def _29_detected(self):
        with self._lock:
            return self._detected_flags['29']
    
    @property
    def _30_detected(self):
        with self._lock:
            return self._detected_flags['30']
    
    @property
    def _31_detected(self):
        with self._lock:
            return self._detected_flags['31']
    
    @property
    def _32_detected(self):
        with self._lock:
            return self._detected_flags['32']
    
    @property
    def _33_detected(self):
        with self._lock:
            return self._detected_flags['33']
    
    @property
    def _34_detected(self):
        with self._lock:
            return self._detected_flags['34']
    
    @property
    def _35_detected(self):
        with self._lock:
            return self._detected_flags['35']
    
    @property
    def _36_detected(self):
        with self._lock:
            return self._detected_flags['36']
    
    @property
    def _37_detected(self):
        with self._lock:
            return self._detected_flags['37']
    
    @property
    def _38_detected(self):
        with self._lock:
            return self._detected_flags['38']
    
    @property
    def _39_detected(self):
        with self._lock:
            return self._detected_flags['39']
    
    @property
    def _40_detected(self):
        with self._lock:
            return self._detected_flags['40']
    
    @property
    def _41_detected(self):
        with self._lock:
            return self._detected_flags['41']
    
    @property
    def _42_detected(self):
        with self._lock:
            return self._detected_flags['42']
    
    @property
    def _43_detected(self):
        with self._lock:
            return self._detected_flags['43']
    
    @property
    def _44_detected(self):
        with self._lock:
            return self._detected_flags['44']
    
    @property
    def _45_detected(self):
        with self._lock:
            return self._detected_flags['45']
    
    @property
    def _46_detected(self):
        with self._lock:
            return self._detected_flags['46']
    
    @property
    def _47_detected(self):
        with self._lock:
            return self._detected_flags['47']
    
    @property
    def _48_detected(self):
        with self._lock:
            return self._detected_flags['48']
    
    @property
    def _49_detected(self):
        with self._lock:
            return self._detected_flags['49']
    
    @property
    def _50_detected(self):
        with self._lock:
            return self._detected_flags['50']
    
    @property
    def _51_detected(self):
        with self._lock:
            return self._detected_flags['51']
    
    @property
    def _52_detected(self):
        with self._lock:
            return self._detected_flags['52']
    
    @property
    def limit_height_distance(self):
        with self._lock:
            return self._distance
    
    @property
    def yellow_light_distance(self):
        with self._lock:
            return self._distance
    
    @property
    def line_angle(self):
        with self._lock:
            return self._line_angle

    @property
    def aligned_duration(self):
        with self._lock:
            return self._aligned_duration

    @property
    def yaw_adjust(self):
        with self._lock:
            return self._yaw_adjust

class LidarNode(Node):#每次获取数据前记得检查更新!!!!
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscriber = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile)
        self.subscriber  # 防止未使用变量的警告
        self.list = []
        self.data_lock = Lock()
        self.update = False

    def scan_callback(self, msg):
        
        ranges = np.array(msg.ranges)
        ranges[ranges < 0.02] = np.nan  # 替换小于0.02的值为NaN
        # 进行线性插值填补NaN
        nans, x = np.isnan(ranges), lambda z: z.nonzero()[0]
        ranges[nans] = np.interp(x(nans), x(~nans), ranges[~nans])
        with self.data_lock:  # 确保数据写入时不会被其他线程访问
            self.list = ranges.tolist()
            self.update = True

    def get_data(self):
        with self.data_lock:  # 确保数据读取时不会被其他线程修改
            self.update = False
            return self.list
    #必定获得新数据
    def get_new_data(self):
        while(True):
            if self.update == False:
                time.sleep(0.1)
            else:
                with self.data_lock:
                    self.update = False
                    return self.list
  
class IMUNode(Node):
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscriber = self.create_subscription(
            Imu,
            "imu",
            self.imu_callback,
            qos_profile)
        self.subscriber  # 防止未使用变量的警告
        self.date_lock = Lock()
    
    def imu_callback(self,msg):
        pass

class DepthCameraNode(Node):
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscriber = self.create_subscription(
            PointCloud2,
            "d435/points",
            self.PointCloud_callback,
            qos_profile)
        self.subscriber  
        self.data_lock = Lock()
        self.update = False
        self.global_map = None  # 用于存储全局地图
        self.T = np.eye(4)
        self.T[:3,:3] = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        self.T[:3,3] = np.array([271.994e-3, 25e-3, 114.912e-3])
        self.transMatrix = np.eye(4)
        self.max_distance = 1.7

    def PointCloud_callback(self,msg):
        dtype = np.dtype([('x',np.float32),('y',np.float32),('z',np.float32),('rgb',np.float32)])
        point_cloud_data = np.frombuffer(msg.data,dtype=dtype)
        points = np.vstack([point_cloud_data['x'], point_cloud_data['y'], point_cloud_data['z']]).T#单位不是很肯定，应该是米
    
        camera_pcd = o3d.geometry.PointCloud()
        camera_pcd.points = o3d.utility.Vector3dVector(points)
        robot_pcd = camera_pcd.transform(self.T)
        #降采样
        robot_pcd = robot_pcd.voxel_down_sample(voxel_size=0.02)
        #过滤
        filtered_pcd = self.CloudFilter(robot_pcd)
        #if self.global_map == None:
            
        
        with self.data_lock:
            #self.elevation_map = self.generate_elevation_map(filtered_points)
            o3d.visualization.draw_geometries([filtered_pcd], window_name='Point Cloud in Robot Frame')

    def CloudFilter(self, pcd):
        return self.DistanceFilter(pcd)
        #return self.passThrough(pcd,-10,0.4,'z')
        
    def DistanceFilter(self,pcd):
        points = np.asarray(pcd.points)
        distances = np.linalg.norm(points, axis=1)
        mask = distances <= self.max_distance  # 保留在最大范围内的点
        filtered_points = points[mask]
        
        # 创建新的点云对象
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        return filtered_pcd

    def InitGlobalMap(self,pcd):
        self.global_map = pcd
               
    # def passThrough(self, pcd, pass_min, pass_max, pass_axis='z'):
    #     points = np.asarray(pcd.points)
    #     # 根据指定轴进行过滤
    #     if pass_axis == 'x':
    #         mask = (points[:, 0] >= pass_min) & (points[:, 0] <= pass_max)
    #     elif pass_axis == 'y':
    #         mask = (points[:, 1] >= pass_min) & (points[:, 1] <= pass_max)
    #     elif pass_axis == 'z':
    #         mask = (points[:, 2] >= pass_min) & (points[:, 2] <= pass_max)
    #     else:
    #         raise ValueError("Axis must be 'x', 'y', or 'z'")

    #     # 应用过滤器
    #     filtered_points = points[mask]

    #     # 创建新的点云对象
    #     filtered_pcd = o3d.geometry.PointCloud()
    #     filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        
    #     return filtered_pcd

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraNode('depth_camera_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

