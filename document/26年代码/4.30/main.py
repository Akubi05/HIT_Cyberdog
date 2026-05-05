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
from threading import Thread, Lock, Event
from sensor_msgs.msg import Image, LaserScan, Imu
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2
import termios
import tty
import select
from enum import Enum
from pyzbar.pyzbar import decode

import math

class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    UNKNOWN = 3

class Direction(Enum):
    UNKNOWN = 0
    Left = 1
    Right = 2

class AppState:
    def __init__(self):
        self._lock = Lock()
        self._state_id = 0
        self._qr_text = ''
        self._yaw = 0.0
        self._arrow_direction = Direction.UNKNOWN
        self.exit_event = Event()
        self.arrow_lock = Lock()
        self.arrow = None

    @property
    def state_id(self):
        with self._lock:
            return self._state_id

    @state_id.setter
    def state_id(self, value):
        with self._lock:
            self._state_id = value

    @property
    def qr_text(self):
        with self._lock:
            return self._qr_text

    @qr_text.setter
    def qr_text(self, value):
        with self._lock:
            self._qr_text = value

    @property
    def yaw(self):
        with self._lock:
            return self._yaw

    @yaw.setter
    def yaw(self, value):
        with self._lock:
            self._yaw = value

    @property
    def arrow_direction(self):
        with self._lock:
            return self._arrow_direction

    @arrow_direction.setter
    def arrow_direction(self, value):
        with self._lock:
            self._arrow_direction = value

    @property
    def exit_requested(self):
        return self.exit_event.is_set()

    def request_exit(self):
        self.exit_event.set()

    def clear_exit(self):
        self.exit_event.clear()


def request_exit(app_state):
    app_state.request_exit()


def keyboard_listener(app_state):
    """
    监听键盘，按 q 触发安全退出
    需要在终端里运行 main1.py
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # 非规范模式，立即读按键
        while not app_state.exit_requested:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if sys.stdin in rlist:
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    print("\n检测到 q，准备安全退出...")
                    app_state.request_exit()
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

class Robot_Ctrl(object):
    def __init__(self, app_state=None):
        self.app_state = app_state
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
        while self.runing and (self.app_state is None or not self.app_state.exit_requested) and count < time_count * 300:
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            time.sleep(0.005)
            count += 1
        return False

    def send_publish(self):
        while self.runing:
            with self.send_lock:
                if self.delay_cnt > 20:  # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
            time.sleep(0.005)

    def Send_cmd(self, msg):
        with self.send_lock:
            self.delay_cnt = 50
            self.cmd_msg = msg

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()



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

#视觉节点
class VisionNode(Node):
    def __init__(self, app_state):
        super().__init__('vision_node')
        self.app_state = app_state
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


        # 识别黑色圆柱
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 60])

        # 识别蓝色障碍物
        self.lower_blue = np.array([100, 80, 50])
        self.upper_blue = np.array([130, 255, 255])
        self._blue_cube_boxes = []   # 保存两个蓝色方块的框

        # 识别虚线
        self._lane_side = "none"   # "left" / "right" / "both" / "none"
        self._lane_lines = {"left": None, "right": None}

        self.detectors = {
            0: self._detect_0,
            'boll': self._detect_boll,
            'cola': self._detect_cola,
            'blue_cube': self._detect_blue_cube,
            'dotted_line': self._detect_dotted_line,

            'lane_center_before_ball': self._detect_lane_center,
            'lane_center_before_cola': self._detect_lane_center,
            'lane_center_before_blue_cube': self._detect_lane_center,
        }
        self.preview_states = {2, 7, 11, 17, 31, 44, 48, 53}

        self._detected_flags = {
            '0': False,
            '1': False,
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

            'boll': False,
            'cola': False,
            'blue_cube':False,
            'dotted_line':False,
            'lane_center':False
        }
        # 调试窗口（实际部署时可关闭）
        self.debug_mode = True

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = cv_image.copy()  # 保存当前帧
            current_state = self.app_state.state_id  # 获取当前状态

            detector = self.detectors.get(current_state)
            if detector is not None:
                detector(cv_image)
            elif current_state in self.preview_states:
                self._show(cv_image)

        except Exception as e:
            self.get_logger().error(f"图像处理失败: {str(e)}")

    def _show(self, cv_image):
        if self.debug_mode:
        # cv2.destroyAllWindows()
            cv2.imshow("Detection Preview", cv_image)
            cv2.waitKey(1)

    # 检测蓝色球 障碍物
    def _get_color_mask(self, hsv_roi, color_name):
        """
        根据颜色名称生成mask
        支持: blue, yellow, green, red
        你也可以继续加别的颜色
        """
        color_ranges = {
            "blue": [
                (np.array([100, 120, 50]), np.array([130, 255, 255]))
            ],
            "yellow": [
                (np.array([20, 100, 100]), np.array([35, 255, 255]))
            ],
            "green": [
                (np.array([35, 80, 50]), np.array([85, 255, 255]))
            ],
            "red": [
                # 红色在HSV里通常跨两段
                (np.array([0, 100, 80]), np.array([10, 255, 255])),
                (np.array([170, 100, 80]), np.array([180, 255, 255]))
            ]
        }

        if color_name not in color_ranges:
            raise ValueError(f"Unsupported color: {color_name}")

        mask = None
        for lower, upper in color_ranges[color_name]:
            part = cv2.inRange(hsv_roi, lower, upper)
            if mask is None:
                mask = part
            else:
                mask = cv2.bitwise_or(mask, part)

        return mask
    
    # 识别虚线的辅助函数
    def _fit_dashed_side(self, points, expected_side):
        """
        points: [(cx, cy), ...]，某一侧所有候选白块的中心点
        expected_side: "left" or "right"

        返回:
            ok, line_param
        其中 line_param = (vx, vy, x0, y0, k)
        k = dx/dy，表示 y 每增加 1，x 改变多少
        """
        if len(points) < 2:
            return False, None

        pts = np.array(points, dtype=np.float32).reshape(-1, 1, 2)

        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx)
        vy = float(vy)
        x0 = float(x0)
        y0 = float(y0)

        if abs(vy) < 1e-6:
            return False, None

        # 用 x = x0 + k * (y - y0)
        k = vx / vy

        # 透视图里：
        # 左虚线：越往下 y 越大，x 越往左，所以 k < 0
        # 右虚线：越往下 y 越大，x 越往右，所以 k > 0
        if expected_side == "left" and k > -0.03:
            print(f"Left line k={k:.4f} not negative enough")
            return False, None
        if expected_side == "right" and k < 0.03:
            print(f"Right line k={k:.4f} not positive enough")
            return False, None

        xs = pts[:, 0, 0]
        ys = pts[:, 0, 1]

        # 点在 y 方向要有足够展开
        y_span = float(np.max(ys) - np.min(ys))
        if y_span < 20:
            print(f"Line y span {y_span:.2f} too small")
            return False, None

        # 拟合残差
        pred_xs = x0 + k * (ys - y0)
        mean_err = float(np.mean(np.abs(xs - pred_xs)))
        if mean_err > 18:
            print(f"Line mean error {mean_err:.2f} too large")
            return False, None

        return True, (vx, vy, x0, y0, k)


    def _detect_lane_center(self, cv_image):
        # 居中算法的视觉检测部分
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # 只看下半部分，更接近机器狗脚下的车道线
        roi_y_start = height // 2
        roi = hsv[roi_y_start:height, :]
        roi_h, roi_w = roi.shape[:2]

        # 黄色二值化
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)

        # 形态学去噪 + 连通
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 边缘
        edges = cv2.Canny(mask, 50, 150)

        debug_img = cv_image.copy()
        img_center_x = width // 2

        def detect_side_line(x_min, x_max, side_name):
            """
            在左/右半区里找一条最可信的车道线。
            返回:
                best_line: (gx1, gy1, gx2, gy2)
                ref_x: 该线在 ROI 底部参考高度处的 x 坐标
            """
            side_edges = edges[:, x_min:x_max]
            lines = cv2.HoughLinesP(
                side_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=25,
                minLineLength=35,
                maxLineGap=25
            )

            if lines is None:
                return None, None

            best_line = None
            best_score = -1
            best_ref_x = None

            # 取 ROI 底部附近作为统一参考高度
            y_ref_local = roi_h - 1

            for line in lines:
                x1, y1, x2, y2 = line[0]

                # 转成整幅图坐标
                gx1 = x1 + x_min
                gx2 = x2 + x_min
                gy1 = y1 + roi_y_start
                gy2 = y2 + roi_y_start

                dx = gx2 - gx1
                dy = gy2 - gy1
                length = np.hypot(dx, dy)

                if length < 30:
                    continue

                # 去掉近水平线，保留更像车道边线的斜线/近竖线
                angle = np.degrees(np.arctan2(dy, dx))
                if abs(angle) < 20:
                    continue

                # 用参考高度 y_ref_local 计算该线的 x
                if dx == 0:
                    ref_x = gx1
                else:
                    # 先在 ROI 局部坐标里算，再换回全局
                    t = (y_ref_local - y1) / (y2 - y1 + 1e-6)
                    x_at_ref_local = x1 + t * (x2 - x1)
                    ref_x = x_at_ref_local + x_min

                # 左线更希望靠左，右线更希望靠右
                if side_name == "left":
                    side_bias = (roi_w / 2 - ref_x)   # 越靠左越大
                else:
                    side_bias = (ref_x - roi_w / 2)   # 越靠右越大

                score = length + 0.3 * side_bias

                if score > best_score:
                    best_score = score
                    best_line = (gx1, gy1, gx2, gy2)
                    best_ref_x = ref_x

            return best_line, best_ref_x

        # 左右区域分别找线
        left_line, left_x = detect_side_line(0, width // 2, "left")
        right_line, right_x = detect_side_line(width // 2, width, "right")

        with self._lock:
            self._detected_flags['lane_center'] = False
            self._yaw_adjust = 0.0

            # 必须左右线都检测到，才计算中线
            if left_x is not None and right_x is not None:
                lane_center_x = (left_x + right_x) / 2.0
                error_x = lane_center_x - img_center_x

                # 像素误差 -> yaw
                gain = 0.01
                self._yaw_adjust = float(np.clip(-gain * error_x, -1.5, 1.5))
                self._detected_flags['lane_center'] = True

        if self.debug_mode:
            # 画左右线
            if left_line is not None:
                gx1, gy1, gx2, gy2 = left_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (0, 255, 0), 2)
            if right_line is not None:
                gx1, gy1, gx2, gy2 = right_line
                cv2.line(debug_img, (gx1, gy1), (gx2, gy2), (0, 255, 0), 2)

            # 画参考点
            y_ref_global = height - 1
            if left_x is not None:
                cv2.circle(debug_img, (int(left_x), y_ref_global), 5, (255, 0, 0), -1)
            if right_x is not None:
                cv2.circle(debug_img, (int(right_x), y_ref_global), 5, (255, 0, 0), -1)

            # 画图像中心线
            cv2.line(debug_img, (img_center_x, 0), (img_center_x, height), (100, 100, 100), 1)

            # 画车道中线
            if left_x is not None and right_x is not None:
                lane_center_x = int((left_x + right_x) / 2.0)
                cv2.line(debug_img, (lane_center_x, 0), (lane_center_x, height), (0, 0, 255), 2)
                cv2.putText(
                    debug_img,
                    f"yaw_adjust: {self._yaw_adjust:+.3f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

            cv2.imshow("Detection Preview", debug_img)
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
            # if yellow_ratio > self.detection_threshold:

            #     if current_time - self._last_detection_time < 0.5:
            #         # self._detected_flags['0'] = True
            #     else:
            #         self._detected_flags['0'] = False  # 重置短暂检测
            #     self._last_detection_time = current_time
            # else:
            #     self._detected_flags['0'] = False

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_boll(self, cv_image, ball_color="blue"):
        """
        检测前方一定范围内是否有指定颜色的球

        ball_color: "blue" / "yellow" / "green" / "red"
        返回: True / False
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # =========================
        # 1. 定义前方ROI
        # =========================
        # 这里取图像中间偏下区域，表示“前方地面附近”
        roi_w = width // 2
        roi_h = height // 2

        x_start = width // 4
        x_end = x_start + roi_w

        y_start = height // 3
        y_end = y_start + roi_h

        roi = hsv[y_start:y_end, x_start:x_end]

        # =========================
        # 2. 颜色分割
        # =========================
        mask = self._get_color_mask(roi, ball_color)

        # =========================
        # 3. 去噪
        # =========================
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # =========================
        # 4. 找轮廓
        # =========================
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        best_circle = None

        # 这些阈值都可以调
        min_area = 80              # 最小面积
        min_radius = 8             # 最小半径（越大表示要求越近）
        min_circularity = 0.65     # 圆度阈值，1最圆

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter < 1e-5:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)

            # 过滤掉不够圆或者太小的目标
            if circularity < min_circularity:
                continue
            if radius < min_radius:
                continue

            detected = True
            best_circle = (int(cx), int(cy), int(radius), area, circularity)
            break

        # =========================
        # 5. 状态更新
        # =========================
        with self._lock:
            current_time = time.time()
            self._detected_flags['boll'] = detected
            self._last_detection_time = current_time

        # =========================
        # 6. 调试显示
        # =========================
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

            # 画ROI框
            cv2.rectangle(debug_img,
                        (x_start, y_start),
                        (x_end, y_end),
                        (0, 255, 0), 2)

            # 把mask贴到旁边看
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debug_roi = debug_img[y_start:y_end, x_start:x_end]
            overlay = cv2.addWeighted(debug_roi, 0.7, mask_bgr, 0.3, 0)
            debug_img[y_start:y_end, x_start:x_end] = overlay

            # 画检测到的球
            if best_circle is not None:
                cx, cy, radius, area, circularity = best_circle
                # 注意这里要加上ROI偏移量
                real_cx = x_start + cx
                real_cy = y_start + cy

                cv2.circle(debug_img, (real_cx, real_cy), radius, (0, 0, 255), 2)
                cv2.circle(debug_img, (real_cx, real_cy), 2, (255, 0, 0), -1)

                text = f"{ball_color} ball r={radius}px cir={circularity:.2f}"
                cv2.putText(debug_img, text, (real_cx - 40, real_cy - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow("Detection Preview", debug_img)
            cv2.imshow("Ball Mask", mask)
            cv2.waitKey(1)

        return detected

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


   
    def _detect_cola(self, cv_image):
        """
        检测前方是否有可乐
        返回: True / False
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # =========================
        # 1. 取前方 ROI
        # =========================
        # 圆柱一般出现在图像中间偏下，但不要取最底部太多，
        # 否则地面阴影容易干扰
        x_start = width // 4
        x_end   = width * 3 // 4

        y_start = height // 5
        y_end   = height
        # y_end   = height * 4 // 5

        roi = hsv[y_start:y_end, x_start:x_end]

        # =========================
        # 2. 黑色分割
        # =========================
        mask = cv2.inRange(roi, self.lower_black, self.upper_black)

        # =========================
        # 3. 去噪
        # =========================
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # =========================
        # 4. 找轮廓
        # =========================
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        best_box = None

        # 这些参数都可以按实际图像调
        min_area = 800          # 最小面积
        min_height = 80         # 最小高度
        min_aspect = 1.5        # 高宽比 h / w 最小值
        max_aspect = 6.0        # 高宽比 h / w 最大值
        min_fill_ratio = 0.55   # 轮廓面积 / 外接矩形面积，太小说明不是实心柱状物

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w <= 0 or h <= 0:
                continue

            aspect_ratio = h / float(w)
            rect_area = w * h
            fill_ratio = area / float(rect_area + 1e-5)

            # 筛掉不够高的
            if h < min_height:
                continue

            # 圆柱投影通常是“高于宽”的竖直块
            if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
                continue

            # 太空洞、太细碎的不要
            if fill_ratio < min_fill_ratio:
                continue

            # 再加一个位置约束：目标中心不要太偏
            cx = x + w / 2.0
            if cx < roi.shape[1] * 0.15 or cx > roi.shape[1] * 0.85:
                continue

            detected = True
            best_box = (x, y, w, h, area, aspect_ratio, fill_ratio)
            break

        # =========================
        # 5. 更新状态
        # =========================
        with self._lock:
            current_time = time.time()
            self._detected_flags['cola'] = detected
            self._last_detection_time = current_time

        # =========================
        # 6. 调试显示
        # =========================
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

            # 画 ROI
            cv2.rectangle(debug_img,
                            (x_start, y_start),
                            (x_end, y_end),
                            (0, 255, 0), 2)

            # 叠加 mask
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debug_roi = debug_img[y_start:y_end, x_start:x_end]
            overlay = cv2.addWeighted(debug_roi, 0.7, mask_bgr, 0.3, 0)
            debug_img[y_start:y_end, x_start:x_end] = overlay

            # 画检测框
            if best_box is not None:
                x, y, w, h, area, aspect_ratio, fill_ratio = best_box
                real_x = x_start + x
                real_y = y_start + y

                cv2.rectangle(debug_img,
                                (real_x, real_y),
                                (real_x + w, real_y + h),
                                (0, 0, 255), 2)

                text = f"black cylinder a={int(area)} ar={aspect_ratio:.2f}"
                cv2.putText(debug_img, text,
                            (real_x, max(20, real_y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 1)

            cv2.imshow("Cylinder Detection Preview", debug_img)
            cv2.imshow("Cylinder Mask", mask)
            cv2.waitKey(1)

        return detected



    def _detect_4_5(self, cv_image):
        qr_data = self.QRCodeDetector.detect_qrcode(cv_image)
        self.app_state.qr_text = qr_data or ''
        with self._lock:
            self._detected_flags['4_5'] = (qr_data is not None)


    def _detect_blue_cube(self, cv_image):
        """
        识别前方两个蓝色正方体
        检测成功时:
            self._detected_flags['blue_cube'] = True
            self._blue_cube_boxes = [(x1,y1,w1,h1), (x2,y2,w2,h2)]
        否则:
            self._detected_flags['blue_cube'] = False
            self._blue_cube_boxes = []
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # =========================
        # 1. 取前方 ROI
        # =========================
        # 只看中间偏下区域，减少天空/远处干扰
        x_start = width // 8
        x_end   = width * 7 // 8
        y_start = height // 5
        y_end   = height * 9 // 10

        roi = hsv[y_start:y_end, x_start:x_end]

        # =========================
        # 2. 蓝色分割
        # =========================
        mask = cv2.inRange(roi, self.lower_blue, self.upper_blue)

        # =========================
        # 3. 去噪
        # =========================
        # 注意不要用太强的 close，不然两个方块可能粘成一个
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # =========================
        # 4. 找轮廓
        # =========================
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []

        # 这些参数都可以调
        min_area = 150       # 最小面积
        min_w = 12           # 最小宽度
        min_h = 18           # 最小高度
        min_aspect = 0.5     # w/h 最小值
        max_aspect = 1.6     # w/h 最大值
        min_fill_ratio = 0.55  # area / (w*h)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w < min_w or h < min_h:
                continue

            aspect_ratio = w / float(h + 1e-5)
            rect_area = w * h
            fill_ratio = area / float(rect_area + 1e-5)

            # 宽高比例不能太离谱
            if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
                continue

            # 填充度太小，说明可能不是完整块状物
            if fill_ratio < min_fill_ratio:
                continue

            # 尽量限制在图像中间道路区域
            cx = x + w / 2.0
            cy = y + h / 2.0

            if cx < roi.shape[1] * 0.1 or cx > roi.shape[1] * 0.9:
                continue

            candidates.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy,
                "area": area,
                "fill_ratio": fill_ratio
            })

        # =========================
        # 5. 在候选中找“一对蓝色方块”
        # =========================
        detected = False
        best_pair = []

        # 按中心 x 从左到右排序
        candidates.sort(key=lambda item: item["cx"])

        n = len(candidates)
        for i in range(n):
            for j in range(i + 1, n):
                a = candidates[i]
                b = candidates[j]

                # 两个块高度应接近
                h_ratio = a["h"] / float(b["h"] + 1e-5)
                if h_ratio < 0.65 or h_ratio > 1.5:
                    continue

                # 宽度也应接近
                w_ratio = a["w"] / float(b["w"] + 1e-5)
                if w_ratio < 0.65 or w_ratio > 1.5:
                    continue

                # 上下位置应接近（底边接近）
                bottom_a = a["y"] + a["h"]
                bottom_b = b["y"] + b["h"]
                if abs(bottom_a - bottom_b) > max(a["h"], b["h"]) * 0.35:
                    continue

                # 两者应左右分开，但不能离太远
                gap = b["x"] - (a["x"] + a["w"])
                if gap < -5:
                    # 明显重叠，不像两个独立方块
                    continue
                if gap > max(a["w"], b["w"]) * 2.0:
                    # 距离太远，也不太像这类并排障碍物
                    continue

                detected = True
                best_pair = [
                    (a["x"], a["y"], a["w"], a["h"]),
                    (b["x"], b["y"], b["w"], b["h"])
                ]
                break

            if detected:
                break

        # =========================
        # 6. 更新状态
        # =========================
        with self._lock:
            self._detected_flags['blue_cube'] = detected
            self._last_detection_time = time.time()

            if detected:
                # 转回整张图坐标
                self._blue_cube_boxes = [
                    (x_start + box[0], y_start + box[1], box[2], box[3])
                    for box in best_pair
                ]
            else:
                self._blue_cube_boxes = []

        # =========================
        # 7. 调试显示
        # =========================
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

            # 画 ROI
            cv2.rectangle(debug_img,
                        (x_start, y_start),
                        (x_end, y_end),
                        (0, 255, 0), 2)

            # 把 mask 叠加到 ROI 上
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debug_roi = debug_img[y_start:y_end, x_start:x_end]
            overlay = cv2.addWeighted(debug_roi, 0.7, mask_bgr, 0.3, 0)
            debug_img[y_start:y_end, x_start:x_end] = overlay

            # 画所有候选框（黄色）
            for c in candidates:
                rx = x_start + c["x"]
                ry = y_start + c["y"]
                cv2.rectangle(debug_img, (rx, ry), (rx + c["w"], ry + c["h"]), (0, 255, 255), 1)

            # 画最终识别结果（红色）
            for box in self._blue_cube_boxes:
                x, y, w, h = box
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 0, 255), 2)

            text = f"blue cubes detected: {self._detected_flags['blue_cube']}"
            cv2.putText(debug_img, text, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow("Blue Cubes Detection", debug_img)
            cv2.imshow("Blue Cubes Mask", mask)
            cv2.waitKey(1)

    def _detect_dotted_line(self, cv_image):
        """
        识别左右两侧虚线
        支持输入:
        1. 原始 BGR 图
        2. 二值图（单通道）
        """
        # =========================
        # 1. 得到二值 mask
        # =========================
        if len(cv_image.shape) == 2:
            # 已经是二值图/灰度图
            mask = cv_image.copy()
            if mask.dtype != np.uint8:
                mask = mask.astype(np.uint8)
            _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        else:
            # 还是彩色图，就先提黄
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        height, width = mask.shape[:2]

        # =========================
        # 2. ROI：忽略最上面一部分
        # =========================
        y_start = int(height * 0.5)
        roi = mask[y_start:height, :]

        # 轻微去噪，不做会把虚线粘死的强操作
        kernel = np.ones((3, 3), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel)

        # =========================
        # 3. 连通域，不检测“矩形”，只提每个白块的中心
        # =========================
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(roi, connectivity=8)

        left_points = []
        right_points = []

        left_boxes = []
        right_boxes = []

        # 左右分区
        left_max_x = width * 0.5
        right_min_x = width * 0.5

        min_area = 12

        for i in range(1, num_labels):  # 0 是背景
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]

            if area < min_area:
                continue

            cx, cy = centroids[i]
            cx = float(cx)
            cy = float(cy)

            # 去掉顶部可能出现的横向大白条干扰
            # 横向太长且位于较上方的，通常不是侧边虚线
            if w > 3 * h and y < roi.shape[0] * 0.35:
                continue

            if cx < left_max_x:
                left_points.append((cx, cy))
                left_boxes.append((x, y, w, h))
            elif cx > right_min_x:
                right_points.append((cx, cy))
                right_boxes.append((x, y, w, h))

        # =========================
        # 4. 对左右分别拟合“倾斜虚线”
        # =========================
        left_ok, left_line = self._fit_dashed_side(left_points, "left")
        right_ok, right_line = self._fit_dashed_side(right_points, "right")

        if left_ok and right_ok:
            lane_side = "both"
        elif left_ok:
            lane_side = "left"
        elif right_ok:
            lane_side = "right"
        else:
            lane_side = "none"

        detected = (lane_side != "none")

        # =========================
        # 5. 更新状态
        # =========================
        with self._lock:
            # self._detected_flags['dotted_line'] = detected
            self._lane_side = lane_side
            self._lane_lines = {
                "left": left_line,
                "right": right_line
            }
            self._last_detection_time = time.time()

        # =========================
        # 6. 打印结果
        # =========================
        if lane_side == "both":
            print("检测到左侧虚线和右侧虚线")
        elif lane_side == "left":
            print("检测到左侧虚线")
        elif lane_side == "right":
            print("检测到右侧虚线")
        else:
            print("未检测到虚线")

        # =========================
        # 7. 调试显示
        # =========================
        if self.debug_mode:
            if len(cv_image.shape) == 2:
                debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # 画 ROI 起始线
            cv2.line(debug_img, (0, y_start), (width, y_start), (0, 255, 0), 1)

            # 画左右分区线
            cv2.line(debug_img, (int(left_max_x), y_start), (int(left_max_x), height), (255, 0, 0), 1)
            cv2.line(debug_img, (int(right_min_x), y_start), (int(right_min_x), height), (255, 0, 0), 1)

            # 画候选白块框与中心
            for (x, y, w, h) in left_boxes:
                cv2.rectangle(debug_img, (x, y + y_start), (x + w, y + h + y_start), (0, 0, 255), 1)
                cv2.circle(debug_img, (x + w // 2, y + h // 2 + y_start), 2, (0, 0, 255), -1)

            for (x, y, w, h) in right_boxes:
                cv2.rectangle(debug_img, (x, y + y_start), (x + w, y + h + y_start), (255, 255, 0), 1)
                cv2.circle(debug_img, (x + w // 2, y + h // 2 + y_start), 2, (255, 255, 0), -1)

            # 画拟合线
            def draw_line(line_param, color):
                if line_param is None:
                    return
                vx, vy, x0, y0, k = line_param

                y1 = 0
                y2 = roi.shape[0] - 1
                x1 = int(x0 + k * (y1 - y0))
                x2 = int(x0 + k * (y2 - y0))

                cv2.line(debug_img,
                        (x1, y1 + y_start),
                        (x2, y2 + y_start),
                        color, 2)

            draw_line(left_line, (0, 0, 255))
            draw_line(right_line, (255, 255, 0))

            cv2.putText(debug_img, f"lane: {lane_side}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Lane Binary", mask)
            cv2.imshow("Lane Detection", debug_img)
            cv2.waitKey(1)


    def is_detected(self, key):
        with self._lock:
            return self._detected_flags.get(key, False)
    
    
    @property
    def yaw_adjust(self):
        with self._lock:
            return self._yaw_adjust
    
    @property
    def yaw_adjust1(self):
        with self._lock:
            return self._yaw_adjust1

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

def loadtoml(file, app):
    try:
        steps = toml.load(file)
        for step in steps['step']:
            app.msg.mode = step['mode']
            app.msg.value = step['value']
            app.msg.contact = step['contact']
            app.msg.gait_id = step['gait_id']
            app.msg.duration = step['duration']
            app.msg.life_count = (app.msg.life_count + 1) % 128
            for i in range(3):
                app.msg.vel_des[i] = step['vel_des'][i]
                app.msg.rpy_des[i] = step['rpy_des'][i]
                app.msg.pos_des[i] = step['pos_des'][i]
                app.msg.acc_des[i] = step['acc_des'][i]
                app.msg.acc_des[i+3] = step['acc_des'][i+3]
                app.msg.foot_pose[i] = step['foot_pose'][i]
                app.msg.ctrl_point[i] = step['ctrl_point'][i]
            for i in range(2):
                app.msg.step_height[i] = step['step_height'][i]

            app.ctrl.Send_cmd(app.msg)
            print('robot_control_cmd lcm publish mode :', app.msg.mode, 'gait_id :', app.msg.gait_id, 'msg.duration=', app.msg.duration)
            time.sleep(0.1)

        app.ctrl.Send_cmd(app.msg)
        time.sleep(0.27)

    except KeyboardInterrupt:
        app.msg.mode = 7
        app.msg.gait_id = 0
        app.msg.duration = 0
        pass


class RobotApp:
    def __init__(self):
        self.app_state = AppState()
        self.ctrl = None
        self.msg = None
        self.vision_node = None
        self.lidar_node = None
        self.imu_node = None
        self.executor = None
        self.spin_thread = None
        self.keyboard_thread = None
        self._runtime_started = False
        self._shutdown_done = False

        # 居中pid参数
        self._lane_center_integral = 0.0
        self._lane_center_last_error = 0.0
        self._lane_center_stable_since = None

        self.state_handlers = {
            0: self._handle_state_0,
            'boll': self._handle_state_boll,
            'cola': self._handle_state_cola,
            'dotted_line': self._handle_state_dotted_line,
            'blue_cube': self._handle_state_blue_cube,

            'lane_center_before_ball': self._handle_state_lane_center,
            'lane_center_before_cola': self._handle_state_lane_center,
            'lane_center_before_blue_cube': self._handle_state_lane_center,
        }

    def _reset_lane_center_controller(self):
        self._lane_center_integral = 0.0
        self._lane_center_last_error = 0.0
        self._lane_center_stable_since = None

    def start(self):
        if self._runtime_started:
            return

        self.app_state.clear_exit()
        self._shutdown_done = False

        self.ctrl = Robot_Ctrl(self.app_state)
        self.ctrl.run()
        self.msg = robot_control_cmd_lcmt()

        if not rclpy.ok():
            rclpy.init()

        self.vision_node = VisionNode(self.app_state)
        self.lidar_node = LidarNode('lidar_node')
        self.imu_node = IMUNode('imu_node')

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.vision_node)
        self.executor.add_node(self.lidar_node)
        self.executor.add_node(self.imu_node)

        self.spin_thread = Thread(target=self.spin_executor, daemon=True)
        self.spin_thread.start()

        self.keyboard_thread = Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self._runtime_started = True

    def spin_executor(self):
        try:
            if self.executor is not None:
                self.executor.spin()
        except Exception as e:
            print(f"spin_executor退出: {e}")

    def keyboard_listener(self):
        """
        监听键盘，按 q 触发安全退出
        需要在终端里运行 main1.py
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self.app_state.exit_requested:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if sys.stdin in rlist:
                    ch = sys.stdin.read(1)
                    if ch.lower() == 'q':
                        print("\n检测到 q，准备安全退出...")
                        self.app_state.request_exit()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _tick_ready(self, current_time, last_cmd_time, cmd_interval):
        return current_time - last_cmd_time > cmd_interval

    def _send_cmd(self, *, mode, gait_id, vel=None, rpy=None, step_height=None):
        self.msg.mode = mode
        self.msg.gait_id = gait_id
        if vel is not None:
            self.msg.vel_des = vel
        if rpy is not None:
            self.msg.rpy_des = rpy
        if step_height is not None:
            self.msg.step_height = step_height
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.ctrl.Send_cmd(self.msg)

    def prepare_robot(self):
        self.msg.mode = 7
        self.msg.gait_id = 0
        self.msg.vel_des = [0.0, 0.0, 0.0]
        self.msg.rpy_des = [0.0, 0.0, 0.0]
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.ctrl.Send_cmd(self.msg)
        time.sleep(0.5)

    def standup(self):
        self.msg.mode = 12
        self.msg.gait_id = 0
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.ctrl.Send_cmd(self.msg)
        self.ctrl.Wait_finish(12, 0, 10)

    def walk(self, x_vel, yaw):
        self.msg.mode = 11
        self.msg.gait_id = 10
        self.msg.vel_des = [x_vel, 0.0, yaw]
        self.msg.rpy_des = [0, -2.5, 0]
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.ctrl.Send_cmd(self.msg)
        time.sleep(0.1)

    def down(self):
        self.msg.mode = 7
        self.msg.gait_id = 1
        self.msg.vel_des = [0.3, 0, 0]
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.ctrl.Send_cmd(self.msg)
        self.ctrl.Wait_finish(7, 1, 5)

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self.app_state.request_exit()

        print('开始安全退出...')

        try:
            if self.msg is not None and self.ctrl is not None:
                self.msg.mode = 7
                self.msg.gait_id = 0
                self.msg.duration = 0
                self.msg.vel_des = [0.0, 0.0, 0.0]
                self.msg.rpy_des = [0.0, 0.0, 0.0]
                self.msg.life_count = (self.msg.life_count + 1) % 128
                self.ctrl.Send_cmd(self.msg)
                time.sleep(0.3)
        except Exception as e:
            print(f'发送退出命令失败: {e}')

        try:
            if self.ctrl is not None:
                self.ctrl.quit()
        except Exception as e:
            print(f'Ctrl.quit() 失败: {e}')

        try:
            if self.executor is not None:
                self.executor.shutdown()
        except Exception as e:
            print(f'executor.shutdown() 失败: {e}')

        for node in (self.vision_node, self.lidar_node, self.imu_node):
            try:
                if node is not None:
                    node.destroy_node()
            except Exception as e:
                print(f'destroy_node 失败: {e}')

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        self._runtime_started = False
        print('已安全退出')

    def _handle_state_0(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=11, gait_id=27, vel=[0.0, 0.0, 0.0])
            last_cmd_time = current_time
        if self.vision_node.is_detected('0'):
            print('进入状态1')
            self.app_state.state_id = 0
        return last_cmd_time

    def _handle_state_boll(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=21, gait_id=0, vel=[0.1, 0.0, 0.0])
            last_cmd_time = current_time
        if self.vision_node.is_detected('boll'):
            print('检测到球，进入状态1')
            self.app_state.state_id = 1
        return last_cmd_time

    def _handle_state_2(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=11, gait_id=10, vel=[0.3, 0.0, 0.0], rpy=[0.0, -2.5, 0.0])
            last_cmd_time = current_time
        self.app_state.state_id = 1
        print('进入状态1')
        time.sleep(1.0)
        return last_cmd_time

    def _handle_state_3(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            yaw_cmd = float(np.clip(self.vision_node.yaw_adjust, -1.5, 1.5))
            self.app_state.yaw = yaw_cmd
            self.walk(x_vel=0.4, yaw=yaw_cmd)
            last_cmd_time = current_time
        if abs(self.vision_node.yaw_adjust) < 0.05:
            print('进入状态4')
            self.app_state.state_id = 4
            self._send_cmd(mode=11, gait_id=10, vel=[0.3, 0.0, 0.0], rpy=[0.0, -2.5, 0.0])
        return last_cmd_time

    def _handle_state_cola(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=21, gait_id=0)
            last_cmd_time = current_time
        if self.vision_node.is_detected('cola'):
            print('检测到可乐')
            self.app_state.state_id = 4
        return last_cmd_time

    def _handle_state_blue_cube(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=21, gait_id=0)
            last_cmd_time = current_time
        if self.vision_node.is_detected('blue_cube'):
            print('检测到蓝色障碍物')
            self.app_state.state_id = 5
        return last_cmd_time

    def _handle_state_dotted_line(self, current_time, last_cmd_time, cmd_interval):
        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            self._send_cmd(mode=21, gait_id=0)
            last_cmd_time = current_time
        if self.vision_node.is_detected('dotted_line'):
            self.app_state.state_id = ''
        return last_cmd_time

    def _handle_state_lane_center(self, current_time, last_cmd_time, cmd_interval):
        kp = 1.2
        kd = 0.5
        ki = 0.05
        max_yaw = 0.5

        current_state = self.app_state.state_id

        if self._tick_ready(current_time, last_cmd_time, cmd_interval):
            yaw_error = self.vision_node.yaw_adjust
            delta_error = yaw_error - self._lane_center_last_error

            self._lane_center_integral = float(
                np.clip(
                    self._lane_center_integral + yaw_error * cmd_interval,
                    -1.0,
                    1.0
                )
            )

            yaw_speed = (
                kp * yaw_error
                + kd * delta_error
                + ki * self._lane_center_integral
            )
            yaw_speed = float(np.clip(yaw_speed, -max_yaw, max_yaw))

            self.app_state.yaw = yaw_speed

            self._send_cmd(
                mode=11,
                gait_id=27,
                vel=[0.1, 0.0, yaw_speed],
                rpy=[0.0, 2.5, 0.0],
                step_height=[0.06, 0.06],
            )

            self._lane_center_last_error = yaw_error
            last_cmd_time = current_time

        # 连续稳定居中一段时间后，执行状态跳转
        stable_threshold = 0.05
        stable_duration = 1.0

        yaw_error = self.vision_node.yaw_adjust
        if abs(yaw_error) < stable_threshold:
            if self._lane_center_stable_since is None:
                self._lane_center_stable_since = current_time
            elif current_time - self._lane_center_stable_since >= stable_duration:
                if current_state == 'lane_center_before_ball':
                    print('lane_center_before_ball 完成，进入 boll')
                    self.app_state.state_id = 'boll'
                elif current_state == 'lane_center_before_cola':
                    print('lane_center_before_cola 完成，进入 cola')
                    self.app_state.state_id = 'cola'
                elif current_state == 'lane_center_before_blue_cube':
                    print('lane_center_before_blue_cube 完成，进入 blue_cube')
                    self.app_state.state_id = 'blue_cube'

                self._reset_lane_center_controller()
        else:
            self._lane_center_stable_since = None

        return last_cmd_time                

    def run(self):
        if not self._runtime_started:
            raise RuntimeError('RobotApp.start() must be called before run().')

        # self.app_state.state_id = 0
        # self.app_state.state_id = 'cola'
        # self.app_state.state_id = 'blue_cube'
        # self.app_state.state_id = 'boll'

        self.app_state.state_id = 'lane_center_before_ball'

        self.app_state.qr_text = ''
        self.app_state.yaw = 0.0
        self.app_state.arrow_direction = Direction.UNKNOWN

        last_cmd_time = time.time()
        cmd_interval = 0.1
        self.prepare_robot()
        self.standup()

        while rclpy.ok() and not self.app_state.exit_requested:
            current_time = time.time()
            current_state = self.app_state.state_id

            handler = self.state_handlers.get(current_state)
            if handler is None:
                print(f'未知状态: {current_state}')
                break

            last_cmd_time = handler(current_time, last_cmd_time, cmd_interval)

def main():
    app = RobotApp()
    try:
        app.start()
        app.run()
    except KeyboardInterrupt:
        print('检测到 Ctrl+C，准备退出...')
        app.app_state.request_exit()
    finally:
        app.shutdown()


if __name__ == '__main__':
    main()
