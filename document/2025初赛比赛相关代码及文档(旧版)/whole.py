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
from enum import Enum, auto
from pyzbar.pyzbar import decode
from NodeBase import RGBCameraNode, LidarNode, Color, Direction, DepthCameraNode, IMUNode, QRCodeDetector, ArrowDetector, YellowLightDetector, LimitHeightDetector
from CtrlBase import Robot_Ctrl,State

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
QR1 = 'A-2'
QR2 = 'B-1'
yaw = 0.0
arrow_direction = Direction.UNKNOWN
arrow_lock = Lock()  # 保证线程安全
arrow = None

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

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
            elif current_state == 2:
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
        global QR1
        QR1 = qr_data
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

def spin_executor():
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

Ctrl = Robot_Ctrl()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()

vision_node = VisionNode()
executor = MultiThreadedExecutor()
executor.add_node(vision_node)

spin_thread = Thread(target=spin_executor)
spin_thread.start()

def main():
    global state_id, QR1, QR2
    state_id = 0
    last_cmd_time = time.time()
    cmd_interval = 0.1  # 命令发送间隔（秒）
    Ctrl.standup()
    try:
        while rclpy.ok():
            current_time = time.time()
            if state_id == 0:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0)
                    last_cmd_time = current_time
                if vision_node._0_detected:
                    print("进入状态1")
                    state_id = 1

            elif state_id == 1:
                Ctrl.walk(vel=0.5,yaw=-2.25,duration=1.2)
                Ctrl.walk(vel=0.3,yaw=0,duration=1.0)
                state_id = 2
                print("进入状态2")
            
            elif state_id == 2:
                if current_time - last_cmd_time > cmd_interval:
                    yaw = vision_node.yaw_adjust
                    yaw = np.clip(yaw, -1.5, 1.5)
                    Ctrl.walk(vel=0.4,yaw=yaw)
                    last_cmd_time = current_time
                    print(f"当前Yaw调整量: {yaw:.2f} rad/s")
                    
                if abs(vision_node.yaw_adjust) < 0.05:  # 阈值可调整
                    print("进入状态3")
                    state_id = 3 
                    Ctrl.walk(vel=0.3,yaw=0,rpy=-2.5)

            elif state_id == 3:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=-2.5)
                    last_cmd_time = current_time

                if vision_node._3_detected:
                    # Ctrl.standup(rpy=0.45)
                    print("进入状态4")
                    state_id = 4

            elif state_id == 4:
                if current_time - last_cmd_time > cmd_interval:
                    # Ctrl.standup(rpy=0.45)
                    msg.mode = 21
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 0.45, 0.0]
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep(5.0)
                    last_cmd_time = current_time

                if vision_node._4_detected:
                    Ctrl.walk(vel=0.2,yaw=0,duration=0.3,rpy=-0.25)
                    if QR1 == 'A-1':
                        Ctrl.walk(vel=0.4,yaw=2.25,duration=1.2,rpy=-0.25)
                    else:
                        Ctrl.walk(vel=0.4,yaw=-2.25,duration=1.2,rpy=-0.25)
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
                        Ctrl.walk(vel=0.1,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.1,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 6:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._6_detected:
                    print("进入状态7")
                    state_id = 7
                    Ctrl.walk(vel=0.0,yaw=0,rpy=2.5)
                    
            elif state_id == 7:
                Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.2)
                Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.14)  
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
                            Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=0.5)
                            state_id = 9
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        Ctrl.walk(vel=0.08,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.08,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 9:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._9_detected:
                    state_id = 10 
                    print("进入状态10")
                    Ctrl.down()
                    time.sleep(2)
                    Ctrl.standup()
                    state_id = 10 

            elif state_id == 10:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=-0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time
                
                if vision_node._10_detected:
                    print("进入状态11")
                    state_id = 11

            elif state_id == 11:
                Ctrl.walk(vel=0.5,yaw=-2.25, duration=1.2)
                state_id = 12
                print("进入状态12")

            elif state_id == 12:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time
                
                if vision_node._12_detected:
                    print("进入状态13")
                    Ctrl.walk(vel=0.5,yaw=2.25, duration=1.2)
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
                        Ctrl.walk(vel=0.15,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.2,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 14:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._14_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=1.34)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.11)
                    print("进入状态15")
                    state_id = 15

            elif state_id == 15:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.1,yaw=0,rpy=0)
                    last_cmd_time = current_time
                
                # 检测到单个黄色连通域时切换状态
                if vision_node._15_detected:
                    Ctrl.walk(vel=0.23,yaw=0.4,rpy=-0.45, duration=13.5)
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.45, duration=1.5)
                    Ctrl.walk(vel=0.23,yaw=-0.4,rpy=-0.45, duration=15)
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.45, duration=1.6)
                    Ctrl.walk(vel=0.23,yaw=0.4,rpy=-0.45, duration=13)
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.45, duration=0.9)
                    state_id = 16
                    print("进入状态16")
                    
            elif state_id == 16:
                # Ctrl.standup()
                start_time = time.time()
                global arrow
                while time.time() - start_time < 6:
                    Ctrl.walk(vel=0.255,yaw=0,rpy=-0.4)
                with arrow_lock:
                    print(f"最终箭头方向：{arrow_direction.name}")
                arrow = arrow_direction.name
                if arrow == 'Left':
                    Ctrl.walk(vel=0.2,yaw=-0.6,rpy=0.45, duration=1.2)
                else:
                    Ctrl.walk(vel=0.2,yaw=0.6,rpy=0.45, duration=0.8)
                Ctrl.walk(vel=0.2,yaw=0,rpy=0.45, duration=0.5)
                state_id = 17
                print("进入状态17")

            elif state_id == 17:
                Ctrl.walk(vel=0.2,yaw=2.25, duration=1.1)
                Ctrl.walk(vel=0.2,yaw=0, duration=3.5)
                Ctrl.walk(vel=0.2,yaw=-1.4, duration=1.3)
                print("进入状态18")
                state_id = 18

            elif state_id == 18:
                # 改进的PID控制器参数（针对虚线场景）
                KP = 1.2    # 比例系数（增大响应速度）
                KD = 0.5    # 微分系数（抑制振荡）
                KI = 0.05   # 新增积分项（消除稳态误差）
                MAX_YAW = 1.5
                
                integral = 0.0
                last_error = 0.0
                state_entry_time = time.time()
                
                while state_id == 18:
                    current_time = time.time()
                    time_since_entry = current_time - state_entry_time
                    
                    yaw_error = vision_node.yaw_adjust
                    delta_error = yaw_error - last_error
                    integral += yaw_error * cmd_interval

                    integral = np.clip(integral, -1.0, 1.0)
                    
                    yaw_speed = KP * yaw_error + KD * delta_error + KI * integral
                    yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                    
                    lateral_speed = 0.0
                    Ctrl.walk(vel=0.5,yaw=yaw_speed)
                    
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
                    Ctrl.walk(vel=0.01,yaw=yaw_speed,rpy=2.5)
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
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time
                
                # 获取限高杆距离并判断
                distance = vision_node.limit_height_distance
                if distance is not None:
                    print(f"限高杆距离: {distance:.2f}米")
                    if distance < 0.5:  # 阈值设为1米
                        # 限高杆步态
                        Ctrl.walk(vel=0.3,yaw=0,duration=2.1)
                        msg.mode = 62
                        msg.gait_id = 80
                        msg.life_count += 1
                        Ctrl.Send_cmd(msg)
                        time.sleep(10)
                        Ctrl.walk(vel=0.3,yaw=0,duration=4.5)
                        print("进入状态19.5")
                        state_id = 19.5
                else:
                    print("未检测到限高杆")

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
                    Ctrl.walk(vel=0.01,yaw=yaw_speed,rpy=2.5)
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
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._20_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.15)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.2)
                    time.sleep( 1.2 )
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
                        Ctrl.walk(vel=0.15,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.2,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 22:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._22_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.2)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.14)
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
                            Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=0.5)
                            state_id = 24
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        Ctrl.walk(vel=0.05,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.05,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 24:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._24_detected:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=2.4)
                    Ctrl.down()
                    time.sleep(1)
                    time.sleep(1)
                    print("进入状态25")
                    state_id = 25

            elif state_id == 25:
                Ctrl.standup()
                time.sleep(1)
                Ctrl.walk(vel=0.0,yaw=2.25,rpy=2.5, duration=1.8)
                print("进入状态26")
                state_id = 26

            elif state_id == 26:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._26_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=2.7)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.0)
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
                        Ctrl.walk(vel=0.15,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.2,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 28:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._28_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.1)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.1)
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
                            Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=0.5)
                            state_id = 30
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        Ctrl.walk(vel=0.05,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.05,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 30:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._30_detected:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=2.4)
                    Ctrl.down()
                    time.sleep(1)
                    time.sleep(1)
                    print("进入状态31")
                    Ctrl.standup()
                    state_id = 31

            elif state_id == 31:
                Ctrl.walk(vel=0.0,yaw=2.25,rpy=2.5, duration=1.9)
                print("进入状态32")
                state_id = 32
            
            elif state_id == 32:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._32_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.5)
                    Ctrl.walk(vel=0.3,yaw=2.25, duration=1.0)
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
                        Ctrl.walk(vel=0.15,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.2,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 34:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._34_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.4)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.2)
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
                    Ctrl.walk(vel=0.01,yaw=yaw_speed)
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

                    Ctrl.walk(vel=0.3,yaw=0,rpy=0)
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
                    Ctrl.walk(vel=0.1,yaw=0,rpy=0)
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
                    Ctrl.walk(vel=0.01,yaw=yaw_speed,rpy=0)
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
                    Ctrl.walk(vel=0.3,yaw=0,rpy=0)
                    last_cmd_time = current_time
                
                # 检测到斜坡顶部到达画面中心时切换状态
                if vision_node._36_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=0,duration=0.2)
                    msg.mode = 62
                    msg.gait_id = 81 #上坡
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep(30)
                    Ctrl.walk(vel=0.3, yaw=0,duration=1.5)
                    msg.mode = 62
                    msg.gait_id = 82 #下坡
                    msg.life_count += 1
                    Ctrl.Send_cmd(msg)
                    time.sleep(20)
                    print("进入状态37")
                    state_id = 37
            
            elif state_id == 37:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.05,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._37_detected:
                    Ctrl.walk(vel=0.1,yaw=0, duration=0.5)
                    Ctrl.walk(vel=0.2,yaw=-2.25, duration=1.1)
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
                        Ctrl.walk(vel=0.05,yaw=yaw_speed)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.1,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 39:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.05,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._39_detected:
                    Ctrl.walk(vel=0.05,yaw=0,rpy=2.5, duration=0.1)
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.27, 0.0, 0.4]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 10.5 )
                    Ctrl.walk(vel=0.27,yaw=0, duration=10.5)
                    #S弯
                    Ctrl.walk(vel=0.23,yaw=-0.4,rpy=-0.45, duration=12.7)
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.45, duration=1.45)
                    Ctrl.walk(vel=0.225,yaw=0.4,rpy=-0.45, duration=16)
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.45, duration=1.6)
                    Ctrl.walk(vel=0.23,yaw=-0.4,rpy=-0.45, duration=13)
                    Ctrl.walk(vel=0.4,yaw=2.25,rpy=-0.45, duration=1.2)
                    print("进入状态40")
                    state_id = 40
                
            
            elif state_id == 40:
                if current_time - last_cmd_time > cmd_interval:
                    yaw = vision_node.yaw_adjust
                    yaw = np.clip(yaw, -1.5, 1.5)
                    Ctrl.walk(vel=0.4, yaw=yaw)
                    last_cmd_time = current_time
                    # print(f"当前Yaw调整量: {yaw:.2f} rad/s")
                    
                if abs(vision_node.yaw_adjust) < 0.05:  # 阈值可调整
                    print("进入状态41")
                    state_id = 41
                    Ctrl.walk(vel=0.3,yaw=0,rpy=-2.5)
            
            elif state_id == 41:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=-2.5)
                    last_cmd_time = current_time

                if vision_node._41_detected:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=-0.25, duration=0.3)
                    Ctrl.walk(vel=0.4,yaw=2.25,rpy=-0.25, duration=1.2)
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
                        Ctrl.walk(vel=0.1,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.1,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 43:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._43_detected:
                    print("进入状态44")
                    state_id = 44
                    Ctrl.walk(vel=0,yaw=0,rpy=2.5)
                    
            elif state_id == 44:
                Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=0.2)
                Ctrl.walk(vel=0.3,yaw=2.25,rpy=2.5, duration=1.14)
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
                            Ctrl.walk(vel=0.2,yaw=0,rpy=2.5, duration=0.5)
                            state_id = 46
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        Ctrl.walk(vel=0.06,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.06,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 46:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._46_detected:
                    state_id = 47
                    print("进入状态47")
                    Ctrl.down()
                    time.sleep(1)
                    time.sleep(1)
                    Ctrl.standup()

            elif state_id == 47:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=-0.3,yaw=0,rpy=2.5)
                    last_cmd_time = current_time
                
                if vision_node._47_detected:
                    print("进入状态48")
                    state_id = 48

            elif state_id == 48:
                Ctrl.walk(vel=0.5,yaw=2.25, duration=1.2)
                state_id = 49
                print("进入状态49")

            elif state_id == 49:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.3,yaw=0)
                    last_cmd_time = current_time
                
                if vision_node._49_detected:
                    print("进入状态50")
                    Ctrl.walk(vel=0.5,yaw=-2.25, duration=1.2)
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
                        Ctrl.walk(vel=0.15,yaw=yaw_speed,rpy=2.5)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}° | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        Ctrl.walk(vel=0.2,yaw=SEARCH_SPEED,rpy=2.5)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 51:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._51_detected:
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=1.34)
                    Ctrl.walk(vel=0.3,yaw=-2.25, duration=1.11)
                    print("进入状态52")
                    state_id = 52
            
            elif state_id == 52:
                if current_time - last_cmd_time > cmd_interval:
                    Ctrl.walk(vel=-0.2,yaw=0,rpy=2.5)
                    last_cmd_time = current_time

                if vision_node._52_detected:
                    print("进入状态53")
                    state_id = 53
                    Ctrl.walk(vel=0.3,yaw=0,rpy=2.5, duration=1.34)
                    Ctrl.walk(vel=0.3,yaw=-2.25,rpy=2.5, duration=1.11)
                    Ctrl.walk(vel=-0.3,yaw=0,rpy=2.5, duration=3)
                    Ctrl.down()
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
                Ctrl.down()
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

if __name__ == '__main__':
    main()