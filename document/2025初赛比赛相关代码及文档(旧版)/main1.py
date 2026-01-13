'''
This demo show the communication interface of MR813 motion control board based on Lcm
- robot_control_cmd_lcmt.py
- cyberdog2_ctrl.toml
'''
import lcm
import sys
import time
import os
import toml
from threading import Thread, Lock
from sensor_msgs.msg import Image, LaserScan, Imu
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2
import termios
import tty
import select
from enum import Enum, auto
from pyzbar.pyzbar import decode

class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    UNKNOWN = 3

class Direction(Enum):
    UNKNOWN = 0
    Left = 1
    Right = 2

state_id = 0
QR1 = ''
yaw = 0.0
arrow_direction = Direction.UNKNOWN
arrow_lock = Lock()  # 保证线程安全
arrow = None

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if (self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish(self, mode, gait_id, time_count):
        count = 0
        while self.runing and count < time_count * 300:
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20:  # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()

#二维码
class QRCodeDetector:
    def __init__(self, InnerMatrix, distCoeffs):
        length = 200  # 单位毫米
        # 定义二维码3D坐标（顺序：左上、右上、右下、左下）
        self.objectPoints = np.array([
            [-length/2,  length/2, 0],  # 左上
            [ length/2,  length/2, 0],  # 右上
            [ length/2, -length/2, 0],  # 右下
            [-length/2, -length/2, 0]   # 左下
        ], dtype=np.float32)
        self.InnerMatrix = InnerMatrix
        self.distCoeffs = distCoeffs
        self.result = None

    def detect_qrcode(self, origin_frame):
        if origin_frame is None:
            print("fail to grab image")
            return None
        
        # 使用pyzbar解码二维码
        decoded_objs = decode(origin_frame)
        
        if len(decoded_objs) == 0:
            print("fail to detect qrcode!")
            return None

        # 提取第一个检测到的二维码
        qr_data = decoded_objs[0].data.decode("utf-8")
        print(f"information: {qr_data}")
        self.result = qr_data

        # 获取二维码的四个角点（转换为np.array并调整顺序）
        qr_points = decoded_objs[0].polygon
        if len(qr_points) != 4:
            print("QRCode polygon points mismatch")
            return qr_data

        # 将角点转换为顺时针顺序（左上、右上、右下、左下）
        pts = np.array(qr_points, dtype=np.int32)
        rect = np.zeros((4, 2), dtype=np.float32)
        
        # 按坐标排序：x+y最小的是左上，x-y最大的是右上等
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]    # 左上
        rect[2] = pts[np.argmax(s)]    # 右下
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)] # 右上
        rect[3] = pts[np.argmax(diff)] # 左下
        
        # 转换为solvePnP需要的格式
        image_points = rect.reshape(4, 1, 2).astype(np.float32)

        # 计算姿态
        success, rvec, tvec = cv2.solvePnP(
            self.objectPoints,
            image_points,
            self.InnerMatrix,
            self.distCoeffs
        )

        # 可选的姿态可视化（调试用）
        # if success:
        #     cv2.drawFrameAxes(origin_frame, self.InnerMatrix, 
        #                      self.distCoeffs, rvec, tvec, 100)

        return qr_data

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

#黄灯
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

#限高杆
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

#视觉节点
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
        #箭头
        self.ArrowDetector = ArrowDetector()
        #二维码
        self.QRCodeDetector = QRCodeDetector(self.InnerMatrix,self.distCoeffs)
        #黄灯
        self.YellowLightDetector = YellowLightDetector(self.InnerMatrix,self.distCoeffs)
        #限高杆
        self.LimitHeightDetector = LimitHeightDetector(self.InnerMatrix,self.distCoeffs)
        self.lower_yellow = np.array([20, 100, 100])  # HSV阈值下限
        self.upper_yellow = np.array([30, 255, 255])  # HSV阈值上限
        self.detection_threshold = 0.2  # 25%黄色像素占比
        self._yaw_adjust = 0.0 
        self._max_side_length = 0
        self._line_angle = 0.0  # 检测到的线条角度（弧度）
        self._line_threshold = 0.025  # 角度误差阈值（约5.7度）
        self._line_1 = 0.015
        self._alignment_start_time = 0.0  # 对齐开始时间戳
        self._aligned_duration = 0.0      # 已对齐持续时间
        self._state9_threshold = 0.11
        self._state10_threshold = 0.01
        self._limit_height_distance = None
        self._yellow_light_distance = None
        self._detected = False
        # 斜坡检测参数（深灰色阈值）
        self.lower_slope = np.array([0, 0, 30], dtype=np.uint8)   # HSV下限（深灰）
        self.upper_slope = np.array([180, 50, 90], dtype=np.uint8) # HSV上限
        self.min_slope_area = 500  # 最小轮廓面积阈值
        self.slope_top_y = None    # 斜坡顶部的y坐标
        self._lock = Lock()
        self._last_detection_time = 0
        self._detected_flags = {
            '0': False,
            '3': False,
            '4': False,
            '4_5': False,
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
            '18_5': False,
            '19': False,
            '19_5': False,
            '20': False,
            '21': False,
            '22': False,
            '23': False,
            '24': False,
            '25': False,
            '26': False,
            '27': False,
            '28': False,
            '29': False,
            '30': False,
            '32': False,
            '33': False,
            '34': False,
            '34_5': False,
            '35': False,
            '35_5': False,
            '36': False,
            '37': False,
            '38': False,
            '39': False,
            '40': False,
            '41': False,
            '42': False,
            '43': False,
            '44': False,
            '45': False,
            '46': False,
            '47': False,
            '48': False,
            '49': False,
            '50': False,
            '51': False,
            '52': False,
        }
        # 调试窗口（实际部署时可关闭）
        self.debug_mode = True

    def image_callback(self, msg):
        global state_id, QR1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = cv_image.copy()  # 保存当前帧
            current_state = state_id  # 获取当前状态
            if current_state == 0:
                self._detect_0(cv_image)
            elif current_state == 1 or current_state == 2 or current_state == 7 or current_state == 11 or current_state == 17 or current_state == 2 or current_state == 31 or current_state == 44 or current_state == 48 or current_state == 53: 
                self._show(cv_image)
            elif current_state == 3:
                self._detect_3(cv_image)
            elif current_state == 4:
                self._detect_4(cv_image)
            elif current_state == 4.5:
                self._detect_4_5(cv_image)
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
            elif current_state == 18.5:
                self._detect_18_5(cv_image)
            elif current_state == 19:
                self._detect_19(cv_image)
            elif current_state == 19.5:
                self._detect_19_5(cv_image)
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
            elif current_state == 34.5:
                self._detect_34_5(cv_image)
            elif current_state == 35:
                self._detect_35(cv_image)
            elif current_state == 35.5:
                self._detect_35_5(cv_image)
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
        # cv2.destroyAllWindows()
            cv2.imshow("Detection Preview", cv_image)
            cv2.waitKey(1)

    def _detect_0(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = width//2 + width//8
        x_end = x_start + width//4
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = width//2 + width//8
        x_end = x_start + width//4
        y_start = height - roi_height1
        y_end = height - roi_height2
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['0'] = True
                else:
                    self._detected_flags['0'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['0'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_3(self, cv_image):
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
                        self._detected_flags['3'] = True
                        self._yaw_adjust = yaw  # 存储yaw调整值
                    
                    # 调试绘制
                    if self.debug_mode:
                        debug_img = cv2.cvtColor(roi, cv2.COLOR_HSV2BGR)
                        cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
                        cv2.circle(debug_img, (center_x, center_y), 5, (0,0,255), -1)
                        cv2.line(debug_img, (img_center_x,0), (img_center_x,roi_height), (255,0,0), 2)
                        cv2.imshow("Detection Preview", debug_img)
                        cv2.waitKey(1)
                        
                        
        # 未检测到有效目标时重置
        with self._lock:
            self._detected_flags['3'] = False
            self._yaw_adjust = 0.0

    def _detect_4(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
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
                self._detected_flags['4'] = True
            else:
                self._max_side_length = 0
                self._detected_flags['4'] = False

        # 调试显示
        if self.debug_mode and 'max_contour' in locals():
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
            cv2.putText(debug_img, f"Quad Side: {max_side:.1f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_4_5(self, cv_image):
        qr_data = self.QRCodeDetector.detect_qrcode(cv_image)
        with self._lock:
            self._detected_flags['4_5'] = (qr_data is not None)

    def _detect_5(self, cv_image):
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
            self._detected_flags['5'] = False
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
                self._detected_flags['5'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_6(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['6'] = True
                else:
                    self._detected_flags['6'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['6'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_8(self, cv_image):
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
            self._detected_flags['8'] = False
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
                self._detected_flags['8'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_9(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['9'] = True
                else:
                    self._detected_flags['9'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['9'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state9:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_10(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        roi_height = 48
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        roi = hsv[height-roi_height:height, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['10'] = True
                else:
                    self._detected_flags['10'] = False
                self._last_detection_time = current_time
            else:
                self._detected_flags['10'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height),
                        (x_end, height),
                        (0,255,0), 2)
            cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state10:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_12(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = width//8 - 10
        x_end = x_start + width//8
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['12'] = True
                else:
                    self._detected_flags['12'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['12'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state12:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_13(self, cv_image):
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
            self._detected_flags['13'] = False
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
                self._detected_flags['13'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_1:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_14(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['14'] = True
                else:
                    self._detected_flags['14'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['14'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state14:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_15(self, cv_image):
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
            self._detected_flags['15'] = (len(valid_contours) == 1)  # 连通域数量为1时触发
            
        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.drawContours(debug_img, valid_contours, -1, (0,255,0), 2)
            cv2.putText(debug_img, f"Yellow Regions: {len(valid_contours)}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

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
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()

        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[:, roi_start_x:roi_end_x]
            
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 边缘检测
            edges = cv2.Canny(mask, 30, 80)
            
            # 线段检测
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

            # 可视化
            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None

        # 左右检测区域参数
        detect_width = width // 3  # 每个检测区域宽度
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
            self._detected_flags['18'] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * 1.8, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            # 绘制中心线
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            
            # 绘制检测区域
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            
            # 显示控制信息
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_18_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()

        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[0:height//2, roi_start_x:roi_end_x]
            
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 边缘检测
            edges = cv2.Canny(mask, 30, 80)
            
            # 线段检测
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

            # 可视化
            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None

        # 左右检测区域参数
        detect_width = width // 3  # 每个检测区域宽度
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
            self._detected_flags['18_5'] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * 1.8, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            # 绘制中心线
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            
            # 绘制检测区域
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            
            # 显示控制信息
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_19(self, cv_image):
        distance = self.LimitHeightDetector.detect_distance(cv_image)
        with self._lock:
            if distance is not None:
                self._detected_flags['19'] = True
                self._limit_height_distance = distance
            else:
                self._detected_flags['19'] = False
                self._limit_height_distance = None
        if self.debug_mode:
            debug_img = cv_image.copy()
            if distance is not None:
                # 绘制距离信息
                cv2.putText(debug_img, f"Distance: {distance:.2f}m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 显示窗口
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_19_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()

        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[0:height//2, roi_start_x:roi_end_x]
            
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 边缘检测
            edges = cv2.Canny(mask, 30, 80)
            
            # 线段检测
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

            # 可视化
            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None

        # 左右检测区域参数
        detect_width = width // 3  # 每个检测区域宽度
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
            self._detected_flags['19_5'] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * 1.8, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            # 绘制中心线
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            
            # 绘制检测区域
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            
            # 显示控制信息
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_20(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 3 * width // 4
        x_end = width
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['20'] = True
                else:
                    self._detected_flags['20'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['20'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state20:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_21(self, cv_image):
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
            self._detected_flags['21'] = False
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
                self._detected_flags['21'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_1:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_22(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['22'] = True
                else:
                    self._detected_flags['22'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['22'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_23(self, cv_image):
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
            self._detected_flags['23'] = False
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
                self._detected_flags['23'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_24(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['24'] = True
                else:
                    self._detected_flags['24'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['24'] = False
        
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state24:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_26(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 3 * width // 4
        x_end = width
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['26'] = True
                else:
                    self._detected_flags['26'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['26'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state26:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)
    
    def _detect_27(self, cv_image):
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
            self._detected_flags['27'] = False
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
                self._detected_flags['27'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_1:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_28(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['28'] = True
                else:
                    self._detected_flags['28'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['28'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_29(self, cv_image):
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
            self._detected_flags['29'] = False
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
                self._detected_flags['29'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_30(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['30'] = True
                else:
                    self._detected_flags['30'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['30'] = False

        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state30:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)
    
    def _detect_32(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 5
        x_end = width // 6
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.2:
                    self._detected_flags['32'] = True
                else:
                    self._detected_flags['32'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['32'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state32:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    
    def _detect_33(self, cv_image):
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
            self._detected_flags['33'] = False
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
                self._detected_flags['33'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_1:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_34(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['34'] = True
                else:
                    self._detected_flags['34'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['34'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_34_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()

        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[:, roi_start_x:roi_end_x]
            
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 边缘检测
            edges = cv2.Canny(mask, 30, 80)
            
            # 线段检测
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

            # 可视化
            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None

        # 左右检测区域参数
        detect_width = width // 3  # 每个检测区域宽度
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
            self._detected_flags['34_5'] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * 1.8, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            # 绘制中心线
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            
            # 绘制检测区域
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            
            # 显示控制信息
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_35(self, cv_image):
        distance = self.YellowLightDetector.detect_distance(cv_image)
        with self._lock:
            if distance is not None:
                self._detected_flags['35'] = True
                self._yellow_light_distance = distance
            else:
                self._detected_flags['35'] = False
                self._yellow_light_distance = None
        if self.debug_mode:
            debug_img = cv_image.copy()
            if distance is not None:
                cv2.putText(debug_img, f"Yellow: {distance:.2f}m", (10,30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255),2)
                # print(f"Yellow: {distance:.2f}m")
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_35_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        center_x = width // 2
        debug_img = cv_image.copy()

        def find_nearest_line(roi_x, roi_width, color):
            # ROI参数
            roi_start_x = max(0, roi_x - roi_width//2)
            roi_end_x = min(width, roi_x + roi_width//2)
            roi = hsv[:, roi_start_x:roi_end_x]
            
            # 黄色检测
            mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 边缘检测
            edges = cv2.Canny(mask, 30, 80)
            
            # 线段检测
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

            # 可视化
            if nearest_line:
                (gx1, gy1, gx2, gy2), mid_x = nearest_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (255,0,255), 2)
                cv2.circle(debug_img, (int(mid_x), int(mid_y)), 5, (0,255,255), -1)
                dx = gx2 - gx1
                if dx != 0:
                    return np.arctan2((gy2-gy1), dx)
            return None

        # 左右检测区域参数
        detect_width = width // 3  # 每个检测区域宽度
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
            self._detected_flags['35_5'] = len(valid_angles) > 0
            self._yaw_adjust = np.clip(yaw_error * 1.8, -1.5, 1.5)  # 调节系数

        # 调试显示
        if self.debug_mode:
            # 绘制中心线
            cv2.line(debug_img, (center_x,0), (center_x,height), (100,100,100), 1)
            
            # 绘制检测区域
            cv2.rectangle(debug_img, 
                        (center_x - width//8 - detect_width//2, 0),
                        (center_x - width//8 + detect_width//2, height),
                        (0,255,0), 1)
            cv2.rectangle(debug_img,
                        (center_x + width//8 - detect_width//2, 0),
                        (center_x + width//8 + detect_width//2, height),
                        (0,255,0), 1)
            
            # 显示控制信息
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}°"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_36(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_slope, self.upper_slope)
        
        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        height, width = cv_image.shape[:2]
        image_center_x = width // 2  # 图像水平中心坐标

        closest_contour = None
        min_center_distance = float('inf')  # 最小中心距离
        slope_top_y = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_slope_area:
                continue  # 跳过小面积轮廓
                
            # 计算轮廓中心坐标
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # 计算与图像中心的水平距离
            horizontal_distance = abs(cX - image_center_x)
            
            # 选择水平方向最接近中心的轮廓
            if horizontal_distance < min_center_distance:
                min_center_distance = horizontal_distance
                closest_contour = cnt

        # 处理最优轮廓
        if closest_contour is not None:
            # 计算轮廓的顶部y坐标
            top_point = tuple(closest_contour[closest_contour[:,:,1].argmin()][0])
            slope_top_y = top_point[1]
            
            # 调试绘制
            if self.debug_mode:
                debug_img = cv_image.copy()
                # 绘制选中轮廓
                cv2.drawContours(debug_img, [closest_contour], -1, (0,255,0), 2)
                # 绘制轮廓中心
                M = cv2.moments(closest_contour)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                cv2.circle(debug_img, (cX, cY), 5, (0,0,255), -1)
                # 绘制图像中心线
                cv2.line(debug_img, (image_center_x,0), 
                        (image_center_x,height), (255,0,0), 1)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
            
            # 判断顶部是否超过画面中心
            if slope_top_y <= height // 2:
                self._detected_flags['36'] = True
            else:
                self._detected_flags['36'] = False
        else:
            self._detected_flags['36'] = False

        with self._lock:
            self.slope_top_y = slope_top_y
    
    def _detect_37(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = 20
        x_end = width - x_start
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['37'] = True
                else:
                    self._detected_flags['37'] = False
                self._last_detection_time = current_time
            else:
                self._detected_flags['37'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state37:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_38(self, cv_image):
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
            self._detected_flags['38'] = False
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
                self._detected_flags['38'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_39(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 40
        roi_height2 = roi_height1 - 20
        x_start = 20
        x_end = 30
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['39'] = True
                else:
                    self._detected_flags['39'] = False
                self._last_detection_time = current_time
            else:
                self._detected_flags['39'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state39:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)
    
    def _detect_40(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        # 设置上半部分ROI（取上半区域）
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
                        self._detected_flags['40'] = True
                        self._yaw_adjust = yaw  # 存储yaw调整值
                    
                    # 调试绘制
                    if self.debug_mode:
                        debug_img = cv2.cvtColor(roi, cv2.COLOR_HSV2BGR)
                        cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
                        cv2.circle(debug_img, (center_x, center_y), 5, (0,0,255), -1)
                        cv2.line(debug_img, (img_center_x,0), (img_center_x,roi_height), (255,0,0), 2)
                        cv2.imshow("Detection Preview", debug_img)
                        cv2.waitKey(1)
                    
        # 未检测到有效目标时重置
        with self._lock:
            self._detected_flags['40'] = False
            self._yaw_adjust = 0.0
    
    def _detect_41(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
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
                self._detected_flags['41'] = True
            else:
                self._max_side_length = 0
                self._detected_flags['41'] = False

        # 调试显示
        if self.debug_mode and 'max_contour' in locals():
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.drawContours(debug_img, [max_contour], -1, (0,255,0), 2)
            cv2.putText(debug_img, f"Quad Side: {max_side:.1f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_42(self, cv_image):
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
            self._detected_flags['42'] = False
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
                self._detected_flags['42'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_43(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + 319
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['43'] = True
                else:
                    self._detected_flags['43'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['43'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_45(self, cv_image):
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
            self._detected_flags['45'] = False
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
                self._detected_flags['45'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_threshold:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_46(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 25
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['46'] = True
                else:
                    self._detected_flags['46'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['46'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state9:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_47(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        roi_height = 48
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        roi = hsv[height-roi_height:height, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['47'] = True
                else:
                    self._detected_flags['47'] = False
                self._last_detection_time = current_time
            else:
                self._detected_flags['47'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height),
                        (x_end, height),
                        (0,255,0), 2)
            cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state45:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_49(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        # 设置ROI区域
        x_end = 7 * width // 8 + 10
        x_start = x_end - width//8
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['49'] = True
                else:
                    self._detected_flags['49'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['49'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state12:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    def _detect_50(self, cv_image):
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
            self._detected_flags['50'] = False
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
                self._detected_flags['50'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < self._line_1:
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
                debug_img = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    cv2.line(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
                    angle_deg = np.degrees(angle_diff)
                    cv2.putText(debug_img, 
                               f"Angle: {angle_deg:+.1f}° | Hold: {self._aligned_duration:.1f}s", 
                               (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_51(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['51'] = True
                else:
                    self._detected_flags['51'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['51'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state14:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)
    
    def _detect_52(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]# 区域待调整
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = 105
        x_end = 215
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        # print(f"黄色像素占比: {yellow_ratio:.2f}")
        # 更新检测状态（需持续检测500ms）
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self._state9_threshold:
                if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['52'] = True
                else:
                    self._detected_flags['52'] = False  # 重置短暂检测
                self._last_detection_time = current_time
            else:
                self._detected_flags['52'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            # cv2.imwrite(f"state14:{yellow_ratio:.6f}.jpg", debug_img)
            cv2.waitKey(1)

    @property
    def _0_detected(self):
        with self._lock:
            return self._detected_flags['0']
    
    @property
    def _3_detected(self):
        with self._lock:
            return self._detected_flags['3']
    
    @property
    def yaw_adjust(self):
        with self._lock:
            return self._yaw_adjust
    
    @property
    def yaw_adjust1(self):
        with self._lock:
            return self._yaw_adjust1

    @property
    def _4_detected(self):
        with self._lock:
            return self._detected_flags['4']

    @property
    def _4_5_detected(self):
        with self._lock:
            return self._detected_flags['4_5']
        
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
    def _18_5_detected(self):
        with self._lock:
            return self._detected_flags['18_5']
    
    @property
    def _19_detected(self):
        with self._lock:
            return self._detected_flags['19']

    @property
    def _19_5_detected(self):
        with self._lock:
            return self._detected_flags['19_5']
        
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
    def _34_5_detected(self):
        with self._lock:
            return self._detected_flags['34_5']
        
    @property
    def _35_detected(self):
        with self._lock:
            return self._detected_flags['35']
    
    @property
    def _35_5_detected(self):
        with self._lock:
            return self._detected_flags['35_5']
    
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
            return self._limit_height_distance
    
    @property
    def yellow_light_distance(self):
        with self._lock:
            return self._yellow_light_distance
    
    @property
    def line_angle(self):
        with self._lock:
            return self._line_angle

    @property
    def aligned_duration(self):
        with self._lock:
            return self._aligned_duration

#雷达节点
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

#IMU节点
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

def loadtoml(file):
    try:
        steps = toml.load(file)
        for step in steps['step']:
            msg.mode = step['mode']
            msg.value = step['value']
            msg.contact = step['contact']
            msg.gait_id = step['gait_id']
            msg.duration = step['duration']
            msg.life_count += 1
            for i in range(3):
                msg.vel_des[i] = step['vel_des'][i]
                msg.rpy_des[i] = step['rpy_des'][i]
                msg.pos_des[i] = step['pos_des'][i]
                msg.acc_des[i] = step['acc_des'][i]
                msg.acc_des[i+3] = step['acc_des'][i+3]
                msg.foot_pose[i] = step['foot_pose'][i]
                msg.ctrl_point[i] = step['ctrl_point'][i]
            for i in range(2):
                msg.step_height[i] = step['step_height'][i]

            Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(msg.mode, msg.gait_id, 10)
            print('robot_control_cmd lcm publish mode :',msg.mode , "gait_id :",msg.gait_id , "msg.duration=" , msg.duration)
                        
            time.sleep( 0.1 )
        # for i in range(300): #60s Heat beat, maintain the heartbeat when life count is not updated
        Ctrl.Send_cmd(msg)
        time.sleep( 0.27 )
            
    except KeyboardInterrupt:
        msg.mode = 7 #PureDamper before KeyboardInterrupt:
        msg.gait_id = 0
        msg.duration = 0
        pass

def standup():  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count += 1  # Command will take effect when life_count update
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(12, 0, 10)

def walk(x_vel, yaw): 
    msg.mode = 11 
    msg.gait_id = 10
    msg.vel_des = [x_vel, 0.0, yaw]
    msg.rpy_des = [0, -2.5, 0]
    msg.life_count += 1  # Command will take effect when life_count update
    Ctrl.Send_cmd(msg)
    time.sleep( 0.1 )
    # Ctrl.Wait_finish(11, 3, 5)

def down():
    msg.mode = 7
    msg.gait_id = 1
    msg.vel_des = [0.3, 0, 0]
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(7, 1, 5)

def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        rclpy.shutdown()


Ctrl = Robot_Ctrl()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()

vision_node = VisionNode()
executor = MultiThreadedExecutor()
executor.add_node(vision_node)
# 添加节点到执行器

spin_thread = Thread(target=spin_executor)
spin_thread.start()

def main():
    global state_id
    global QR1
    # 状态机初始化
    state_id = 0
    last_cmd_time = time.time()
    cmd_interval = 0.1  # 命令发送间隔（秒）
    standup()
    try:
        while rclpy.ok():
            current_time = time.time()
            
            if state_id == 0:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 10     # 步态类型
                    msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)

                    last_cmd_time = current_time
                
                if vision_node._0_detected:
                    print("进入状态1")
                    state_id = 1

            elif state_id == 1:
                # loadtoml("turnRight1.toml")
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.5, 0.0, -2.25]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep( 1.2 )
                state_id = 2
                print("进入状态2")
            
            elif state_id == 2:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 10     # 步态类型
                    msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                    msg.rpy_des = [0, -2.5, 0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    # print(msg.duration)
                    last_cmd_time = current_time
                state_id = 3
                print("进入状态3")
                time.sleep(1.0)

            elif state_id == 3:
                if current_time - last_cmd_time > cmd_interval:
                    yaw = vision_node.yaw_adjust
                    yaw = np.clip(yaw, -1.5, 1.5)
                    walk(x_vel=0.4, yaw=yaw)
                    last_cmd_time = current_time
                    # print(f"当前Yaw调整量: {yaw:.2f} rad/s")
                    
                if abs(vision_node.yaw_adjust) < 0.05:  # 阈值可调整
                    print("进入状态4")
                    state_id = 4 
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]  # 保持前进速度
                    msg.rpy_des = [0, -2.5, 0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)

            elif state_id == 4:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, -2.5, 0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._4_detected:
                    msg.mode = 21
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 0.45, 0.0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    '''if vision_node.frame is not None:
                        global QR1
                        qr = vision_node.QRCodeDetector.detect_qrcode(vision_node.frame)
                        QR1 = 1 if qr == 'A-2' else -1
                        print(f"Detected QR Code: {qr}")'''

                    time.sleep(1)
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -0.25, 0.0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.3 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.4, 0.0, -2.25]
                    msg.rpy_des = [ 0.0, -0.25, 0.0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    state_id = 5
                    print("进入状态5")
                    
            elif state_id == 5:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -1.2               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._5_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态6")
                            state_id = 6
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.1, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
                # state_id = 6
                # print("进入状态6")     
                # time.sleep(0.05)  # 控制循环频率

            elif state_id == 6:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._6_detected:
                    print("进入状态7")
                    state_id = 7
                    msg.mode = 11 
                    msg.gait_id = 10
                    msg.vel_des = [0.0, 0.0, 0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    
            elif state_id == 7:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.2 )   
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, -2.25]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.14 )   
                state_id = 8
                print("进入状态8")    

            elif state_id == 8:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -0.8               # 比例系数
                KD = 1.5                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._8_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态9")
                            msg.mode = 11
                            msg.gait_id = 10
                            msg.vel_des = [0.2, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 9
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.08, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.08, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 9:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._9_detected:
                    state_id = 10 
                    print("进入状态10")
                    down()
                    time.sleep(1)
                    time.sleep(1)
                    standup()
                    state_id = 10 

            elif state_id == 10:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [-0.3, 0.0, 0.0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._10_detected:
                    print("进入状态11")
                    state_id = 11

            elif state_id == 11:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.5, 0.0, -2.25]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.2 )
                state_id = 12
                print("进入状态12")

            elif state_id == 12:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 10     # 步态类型
                    msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._12_detected:
                    print("进入状态13")
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.5, 0.0, 2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    state_id = 13

            elif state_id == 13:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._13_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态14")
                            state_id = 14
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.15, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.2, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 14:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._14_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.34 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.11 )
                    print("进入状态15")
                    state_id = 15

            elif state_id == 15:
                # 持续发送前进指令
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.1, 0.0, 0.0]  # 前进速度0.3m/s
                    msg.rpy_des = [0, 0.00, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                # 检测到单个黄色连通域时切换状态
                if vision_node._15_detected:
                    state_id = 1
                    msg.mode = 11  # Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.23, 0, 0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(13.5)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.5)

                    msg.mode = 11  # Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.23, 0, -0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(15)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.6)

                    msg.mode = 11  #Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.23, 0, 0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(13)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(0.9)
                    state_id = 16
                    print("进入状态16")
                    
            elif state_id == 16:
                # standup()
                start_time = time.time()
                global arrow
                # 持续运动7秒并检测箭头
                while time.time() - start_time < 6:
                    # 保持运动控制
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.255, 0, -0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    
                    # 打印检测结果
                            
                    # time.sleep(0.1)  # 控制循环频率

                # 转移状态并输出最终方向
                with arrow_lock:
                    print(f"最终箭头方向：{arrow_direction.name}")
                arrow = arrow_direction.name
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, 0.6]
                msg.rpy_des = [0, 0.45, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.2)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, 0.0]
                msg.rpy_des = [0, 0.45, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                state_id = 17
                print("进入状态17")

            elif state_id == 17:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, 2.25]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.1)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(3.5)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, -1.4]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.3)
                print("进入状态18")
                state_id = 18
                state_entry_time = time.time()

            elif state_id == 18:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 1.2    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.05   # 新增积分项（消除稳态误差）
                MAX_YAW = 1.5
                
                integral = 0.0
                last_error = 0.0
                
                
                current_time = time.time()
                time_since_entry = current_time - state_entry_time
                
                yaw_error = vision_node.yaw_adjust
                delta_error = yaw_error - last_error
                integral += yaw_error * cmd_interval

                integral = np.clip(integral, -1.0, 1.0)
                
                yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                
                lateral_speed = 0.0
                msg.mode = 11
                msg.gait_id = 27
                msg.vel_des = [0.5, lateral_speed, yaw_speed]
                msg.step_height = [0.06, 0.06]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                # print(f"Yaw Error: {yaw_error:.2f} | Yaw Speed: {yaw_speed:.2f} | Integral: {integral:.2f}")
                # 更新状态
                last_error = yaw_error
                last_cmd_time = current_time
                
                # 退出条件（示例：持续运行或根据其他传感器判断）
                if time_since_entry > 12.5:  # 12.5秒后退出
                    state_id = 18.5
                    print("进入状态18.5")
                time.sleep(cmd_interval)

            elif state_id == 18.5:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 1.2    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.05   # 新增积分项（消除稳态误差）
                MAX_YAW = 0.5
                
                integral = 0.0
                last_error = 0.0
                state_entry_time = time.time()
                
                # 新增：持续时间跟踪变量
                align_start_time = None  # 首次满足条件的时间戳
                current_time = time.time()
                
                while state_id == 18.5:  # 改为循环结构确保持续判断
                    current_time = time.time()
                    time_since_entry = current_time - state_entry_time
                    
                    yaw_error = vision_node.yaw_adjust
                    delta_error = yaw_error - last_error
                    integral += yaw_error * cmd_interval

                    integral = np.clip(integral, -1.0, 1.0)
                    
                    yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                    yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                    
                    lateral_speed = 0.0
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.01, lateral_speed, yaw_speed]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height = [0.06, 0.06]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    # print(f"Yaw Error: {yaw_error:.2f} | Yaw Speed: {yaw_speed:.2f} | Integral: {integral:.2f}")
                    # 条件判断逻辑
                    if abs(yaw_error) < 0.01:
                        if align_start_time is None:
                            align_start_time = current_time  # 首次满足条件时记录时间
                        else:
                            duration = current_time - align_start_time
                            if duration >= 0.5:  # 持续500ms
                                state_id = 19
                                print("进入状态19")
                                break
                    else:
                        align_start_time = None  # 条件不满足时重置
                    
                    # 更新误差和循环间隔
                    last_error = yaw_error
                    time.sleep(cmd_interval)  # 控制循环频率

            elif state_id == 19:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]  # 保持前进
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    
                    last_cmd_time = current_time
                
                # 获取限高杆距离并判断
                distance = vision_node.limit_height_distance
                if distance is not None:
                    # print(f"限高杆距离: {distance:.2f}米")
                    if distance < 0.5:  # 阈值设为1米
                        # 限高杆步态
                        walk(x_vel=0.3, yaw=0)
                        time.sleep(2.1)
                        msg.mode = 62
                        msg.gait_id = 80
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep(10)
                        # print("进入状态20")
                        walk(x_vel=0.3, yaw=0)
                        time.sleep(4.5)
                        state_id = 19.5
                else:
                    # print("未检测到限高杆")
                    pass
            
            elif state_id == 19.5:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 1.2    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.05   # 新增积分项（消除稳态误差）
                MAX_YAW = 0.5
                
                integral = 0.0
                last_error = 0.0
                state_entry_time = time.time()
                
                # 新增：持续时间跟踪变量
                align_start_time = None  # 首次满足条件的时间戳
                current_time = time.time()
                
                while state_id == 19.5:  # 改为循环结构确保持续判断
                    current_time = time.time()
                    time_since_entry = current_time - state_entry_time
                    
                    yaw_error = vision_node.yaw_adjust
                    delta_error = yaw_error - last_error
                    integral += yaw_error * cmd_interval

                    integral = np.clip(integral, -1.0, 1.0)
                    
                    yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                    yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                    
                    lateral_speed = 0.0
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.01, lateral_speed, yaw_speed]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height = [0.06, 0.06]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    # print(f"Yaw Error: {yaw_error:.2f} | Yaw Speed: {yaw_speed:.2f} | Integral: {integral:.2f}")
                    # 条件判断逻辑
                    if abs(yaw_error) < 0.01:
                        if align_start_time is None:
                            align_start_time = current_time  # 首次满足条件时记录时间
                        else:
                            duration = current_time - align_start_time
                            if duration >= 0.1:  # 持续500ms
                                state_id = 20
                                print("进入状态20")
                                break
                    else:
                        align_start_time = None  # 条件不满足时重置
                    
                    # 更新误差和循环间隔
                    last_error = yaw_error
                    time.sleep(cmd_interval)  # 控制循环频率

            elif state_id == 20:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._20_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.9 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.1 )
                    print("进入状态21")
                    state_id = 21
            
            elif state_id == 21:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._21_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态22")
                            state_id = 22
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.15, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.2, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 22:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._22_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.2 )   
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 2.25]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.14 )
                    print("进入状态23")
                    state_id = 23  

            elif state_id == 23:
                ALIGN_DURATION = 0.2    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._23_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态24")
                            msg.mode = 11
                            msg.gait_id = 10
                            msg.vel_des = [0.2, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 24
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.05, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.05, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 24:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._24_detected:
                    state_id = 25
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(2.4)
                    down()
                    time.sleep(1)
                    time.sleep(1)
                    print("进入状态25")
                    standup()
                    state_id = 25

            elif state_id == 25:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, 2.25]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.8)
                print("进入状态26")
                state_id = 26

            elif state_id == 26:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._26_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 2.7 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.0 )
                    print("进入状态27")
                    state_id = 27
            
            elif state_id == 27:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._27_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态28")
                            state_id = 28
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.15, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.2, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 28:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._28_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.1 )   
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.1 )
                    print("进入状态29")
                    state_id = 29
            
            elif state_id == 29:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._29_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态30")
                            msg.mode = 11
                            msg.gait_id = 10
                            msg.vel_des = [0.2, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 30
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.05, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.05, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 30:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._30_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(2.4)
                    down()
                    time.sleep(1)
                    time.sleep(1)
                    print("进入状态31")
                    standup()
                    state_id = 31

            elif state_id == 31:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, 2.25]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.9)
                print("进入状态32")
                state_id = 32
            
            elif state_id == 32:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._32_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.5 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.0 )
                    print("进入状态33")
                    state_id = 33
            
            elif state_id == 33:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._33_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态34")
                            state_id = 34
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.15, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.2, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 34:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._34_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.4 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    print("进入状态34_5")
                    state_id = 35

            elif state_id == 34.5:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 1.0    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.1   # 新增积分项（消除稳态误差）
                MAX_YAW = 0.5
                
                integral = 0.0
                last_error = 0.0
                state_entry_time = time.time()
                
                # 新增：持续时间跟踪变量
                align_start_time = None  # 首次满足条件的时间戳
                current_time = time.time()
                
                while state_id == 34.5:  # 改为循环结构确保持续判断
                    current_time = time.time()
                    time_since_entry = current_time - state_entry_time
                    
                    yaw_error = vision_node.yaw_adjust
                    delta_error = yaw_error - last_error
                    integral += yaw_error * cmd_interval

                    integral = np.clip(integral, -1.0, 1.0)
                    
                    yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                    yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                    
                    lateral_speed = 0.0
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.01, lateral_speed, yaw_speed]
                    msg.rpy_des = [0, 0, 0]
                    msg.step_height = [0.02, 0.02]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    # print(f"Yaw Error: {yaw_error:.2f} | Yaw Speed: {yaw_speed:.2f} | Integral: {integral:.2f}")
                    # 条件判断逻辑
                    if abs(yaw_error) < 0.1:
                        if align_start_time is None:
                            align_start_time = current_time  # 首次满足条件时记录时间
                        else:
                            duration = current_time - align_start_time
                            if duration >= 0.3:  # 持续500ms
                                state_id = 35
                                print("进入状态35")
                                break
                    else:
                        align_start_time = None  # 条件不满足时重置
                    
                    # 更新误差和循环间隔
                    last_error = yaw_error
                    time.sleep(cmd_interval)  # 控制循环频率
            
            elif state_id == 35:
                # 持续前进
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]
                    msg.rpy_des = [0, 0.0, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                # 检测黄灯距离
                yellow_dist = vision_node.yellow_light_distance
                if yellow_dist is not None and yellow_dist < 0.6:  # 1米阈值
                    print(f"检测到黄灯，距离{yellow_dist:.2f}m,进入等待")
                    msg.mode = 21
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 0.45, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(5)
                    state_id = 35.5
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.1, 0.0, 0.0]
                    msg.rpy_des = [0, 0.0, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    print("进入状态35.5")
            
            elif state_id == 35.5:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 0.8    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.1   # 新增积分项（消除稳态误差）
                MAX_YAW = 0.5
                
                integral = 0.0
                last_error = 0.0
                state_entry_time = time.time()
                
                # 新增：持续时间跟踪变量
                align_start_time = None  # 首次满足条件的时间戳
                current_time = time.time()
                
                while state_id == 35.5:  # 改为循环结构确保持续判断
                    current_time = time.time()
                    time_since_entry = current_time - state_entry_time
                    
                    yaw_error = vision_node.yaw_adjust
                    delta_error = yaw_error - last_error
                    integral += yaw_error * cmd_interval

                    integral = np.clip(integral, -1.0, 1.0)
                    
                    yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                    yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                    
                    lateral_speed = 0.0
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.01, lateral_speed, yaw_speed]
                    msg.rpy_des = [0, 0, 0]
                    msg.step_height = [0.02, 0.02]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    # print(f"Yaw Error: {yaw_error:.2f} | Yaw Speed: {yaw_speed:.2f} | Integral: {integral:.2f}")
                    # 条件判断逻辑
                    if abs(yaw_error) < 0.1:
                        if align_start_time is None:
                            align_start_time = current_time  # 首次满足条件时记录时间
                        else:
                            duration = current_time - align_start_time
                            if duration >= 0.3:  # 持续500ms
                                state_id = 36
                                print("进入状态36")
                                break
                    else:
                        align_start_time = None  # 条件不满足时重置
                    
                    # 更新误差和循环间隔
                    last_error = yaw_error
                    time.sleep(cmd_interval)  # 控制循环频率
            
            elif state_id == 36:
                # 持续前进并检测斜坡
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]  # 直行
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                # 检测到斜坡顶部到达画面中心时切换状态
                if vision_node._36_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]  # 直行
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(0.2)
                    msg.mode = 62
                    msg.gait_id = 81 #上坡
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep(30)
                    walk(x_vel=0.3, yaw=0)
                    time.sleep(1.5)
                    msg.mode = 62
                    msg.gait_id = 82 #下坡
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep(20)
                    print("进入状态37")
                    state_id = 37
            
            elif state_id == 37:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.05, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._37_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 0, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.5 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.1 )
                    print("进入状态38")
                    state_id = 38
            
            elif state_id == 38:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.01      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._38_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态39")
                            state_id = 39
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.05, 0.0, yaw_speed]
                        msg.rpy_des = [0, 0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 39:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.05, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._39_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.05, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(0.1)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.27, 0.0, 0.4]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 10.5 )
                    #S弯
                    msg.mode = 11  # Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.23, 0, -0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(12.7)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.45)

                    msg.mode = 11  # Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.225, 0, 0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(16)

                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.6)

                    msg.mode = 11  #Recovery stand
                    msg.gait_id = 10
                    msg.vel_des = [0.23, 0, -0.4]
                    msg.rpy_des = [0, -0.45, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(13)
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.4, 0.0, 2.25]
                    msg.rpy_des = [ 0.0, -0.25, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    print("进入状态40")
                    state_id = 40
                
            
            elif state_id == 40:
                if current_time - last_cmd_time > cmd_interval:
                    yaw = vision_node.yaw_adjust
                    yaw = np.clip(yaw, -1.5, 1.5)
                    walk(x_vel=0.4, yaw=yaw)
                    last_cmd_time = current_time
                    # print(f"当前Yaw调整量: {yaw:.2f} rad/s")
                    
                if abs(vision_node.yaw_adjust) < 0.05:  # 阈值可调整
                    print("进入状态41")
                    state_id = 41
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, 0.0]  # 保持前进速度
                    msg.rpy_des = [0, -2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
            
            elif state_id == 41:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, -2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._41_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -0.25, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.3 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.4, 0.0, 2.25]
                    msg.rpy_des = [ 0.0, -0.25, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    state_id = 42
                    print("进入状态42")
                    
            elif state_id == 42:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -1.2               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._42_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态43")
                            state_id = 43
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.1, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 43:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._43_detected:
                    print("进入状态44")
                    state_id = 44
                    msg.mode = 11 
                    msg.gait_id = 10
                    msg.vel_des = [0.0, 0.0, 0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    
            elif state_id == 44:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.2 )   
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 2.25]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.14 )   
                state_id = 45
                print("进入状态45")    

            elif state_id == 45:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -0.8               # 比例系数
                KD = 1.5                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._45_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态46")
                            msg.mode = 11
                            msg.gait_id = 10
                            msg.vel_des = [0.2, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 46
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.06, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.06, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 46:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._46_detected:
                    state_id = 47
                    print("进入状态47")
                    down()
                    time.sleep(1)
                    time.sleep(1)
                    standup()

            elif state_id == 47:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [-0.3, 0.0, 0.0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._47_detected:
                    print("进入状态48")
                    state_id = 48

            elif state_id == 48:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.5, 0.0, 2.25]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.2 )
                state_id = 49
                print("进入状态49")

            elif state_id == 49:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 10     # 步态类型
                    msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._49_detected:
                    print("进入状态50")
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.5, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    state_id = 50

            elif state_id == 50:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 1.2           # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.1      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._50_detected:
                        angle_error = vision_node.line_angle
                        delta = angle_0 - angle_error
                        angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态51")
                            state_id = 51
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.15, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 10
                        msg.vel_des = [0.2, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 51:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._51_detected:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.34 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.11 )
                    print("进入状态52")
                    state_id = 52
            
            elif state_id == 52:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [-0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._52_detected:
                    print("进入状态53")
                    state_id = 53
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.34 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.3, 0.0, -2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.11 )
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [-0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 3 )
                    down()
                    time.sleep(1)
                    break
                    

            elif state_id == 53:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [-0.3, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 3 )
                down()
                time.sleep(1)
                break
            else:
                break
    except KeyboardInterrupt:
        print("中断操作")
    finally:
        msg.mode = 12
        Ctrl.Send_cmd(msg)
        Ctrl.quit()
        executor.shutdown()
        cv2.destroyAllWindows()
        sys.exit()

# Main function
if __name__ == '__main__':
    main()
