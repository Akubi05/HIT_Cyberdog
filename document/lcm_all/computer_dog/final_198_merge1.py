# coding: utf-8
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
from pyzbar import pyzbar
from NodeBase import SteroCameraNode, DepthNode, QRCodeDetector ,YellowLightDetector
from CtrlBase import Robot_Ctrl,State
from color_config import *
import datetime
import math
import queue
from std_msgs.msg import String
# from protocol.msg import AudioPlayExtend, BmsStatus
import json
import base64
import ssl
import requests
import re
from pc_cmd_send import PC_cmd_send

IMG_SAVE_DIR = "/Users/huangzhicheng/Desktop/lcm/pictures"
if not os.path.exists(IMG_SAVE_DIR):
    os.makedirs(IMG_SAVE_DIR)
    print(f"✅ 已创建图片存储目录: {IMG_SAVE_DIR}")

def save_debug_image(filename, image):
    full_save_path = os.path.join(IMG_SAVE_DIR, filename)
    success = cv2.imwrite(full_save_path, image)
    if success:
        print(f"📸 照片已保存至: {full_save_path}")
    else:
        print(f"保存失败: {full_save_path}")
    return success


class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    UNKNOWN = 3

class Direction(Enum):
    UNKNOWN = 0
    Left = 1
    Right = 2

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def signal_handler(sig, frame):
    global exit_flag
    exit_flag = True
    sys.exit(0) 

class adjustment(Enum):
    near = 1
    faraway = 2
    mid = 3

class PurpleDetector():
    def __init__(self):
        self.lower_purple = np.array([120,50,50])
        self.upper_purple = np.array([170,255,255])
        self.lower_yellow = np.array([20,100,100])
        self.upper_yellow = np.array([30,255,255])
        self.bottom_roi = None
        self.yellow_roi = None
        self.angle = 0

    def get_angle(self,ellipse):
        (center,axes,angle) = ellipse
        if axes[0] < axes[1]:
            angle += 90
        angle = angle % 180
        if angle > 90:
            angle -= 180
        return angle

    def process_purple(self,origin_frame):
        print("get new frame")
        height, width = origin_frame.shape[:2]
        roi_y_start = int(height * 1/2)
        roi_y_end = int(height * 4/5)

        roi_x_start = 0
        roi_x_end = width
        enlarge_factor = 0.8
        # ROI
        self.bottom_roi = origin_frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        roi_height , roi_width = self.bottom_roi.shape[:2]
        hsv = cv2.cvtColor(self.bottom_roi,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.lower_purple,self.upper_purple)
        mask = cv2.GaussianBlur(mask, (5,5), 0)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # 去除小噪点
        if cv2.__version__.startswith('3'):
        # OpenCV 3.x
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
        # OpenCV 4.x
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (contours):
            purple_cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(purple_cnt) > 300:
                return True
                # x, y, w, h = cv2.boundingRect(purple_cnt)
                # new_w = int(w * enlarge_factor)
                # new_h = int(h * enlarge_factor)  
                # # 计算偏移量（中心点不变）
                # dx = int((new_w - w) / 2)
                # dy = int((new_h - h) / 2)
                # # 计算放大后的坐标，确保不超出图像边界
                # new_x = max(0, x - dx)
                # new_y = 0
                # new_x2 = min(roi_width, x + w + dx)
                # new_y2 = min(roi_height, y + h + dy)

                # self.yellow_roi = self.bottom_roi[new_y:new_y2, new_x:new_x2]
                # hsv_roi = cv2.cvtColor(self.yellow_roi, cv2.COLOR_BGR2HSV)
                # yellow_mask = cv2.inRange(hsv_roi, self.lower_yellow, self.upper_yellow)
                # # 形态学处理优化黄色区域 (水平连接)
                # kernel_horizontal = np.ones((1, 15), np.uint8)  
                # yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel_horizontal)        
                # if cv2.__version__.startswith('3'):
                # # OpenCV 3.x
                #     _, yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # else:
                # # OpenCV 4.x
                #     yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)              
                
                # if (yellow_contours):
                #     yellow_cnt = max(yellow_contours, key=cv2.contourArea)
                #     if cv2.contourArea(yellow_cnt) > 100:
                #         self.angle = None
                #         ellipse = cv2.fitEllipse(yellow_cnt)
                #         angle = self.get_angle(ellipse)
                #         if abs(angle) <= 60:
                #             self.angle = angle
                #             return True

        return False


#绿色箭头
class ArrowDetector:
    def __init__(self):
        self.convex_hull = None
        self.center = None
        self.target_point = None
        self.direction = Direction.UNKNOWN
        self.real_direction = Direction.UNKNOWN#真正的结果
        #hsv绿色阈值
        self.lower_green = np.array([40, 40, 40])
        self.upper_green = np.array([85, 255, 255])
        #轮廓拟合越大越粗略
        self.approx_param = 0.02
        #面积阈值
        self.min_area = 350
        #距离相等阈值
        self.equal_param = 0.2
        #
        self.result_frame = None
      
        self.distance = 0
        self.yaw_deg = 0
        #有效方向数
        self.direction_count = 0
        #neican  jibian
        self.InnerMatrix = np.array([
            [
                416.6850297391639,
                0.0,
                319.98128281389444
            ],
            [
                0.0,
                415.51316371220037,
                227.93040503278888
            ],
            [
                0.0,
                0.0,
                1.0
            ]
        ],dtype=np.float32)
        self.distCoeffs = np.array([0.0024461913801336077,
            -0.04339171678852136,
            -0.006551805033021212,
            -0.0003024566454255968,
            0.0],dtype=np.float32)

    def getGreenFrame(self,origin_frame):
        hsv_frame = cv2.cvtColor(origin_frame, cv2.COLOR_BGR2HSV)
    
        mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)
        mask = mask.astype(np.uint8)
        #图像插值放大，原图分辨率过低
        mask = cv2.resize(mask, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        #cv2.imshow("mask1",mask)

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

            if diff_distance > distance_threshold:
                return False

        return True

    def order_points(self,pts):
        x_sorted = pts[np.argsort(pts[:, 0])]
    
        # 将点分为左右两组
        left_points = x_sorted[:2] 
        right_points = x_sorted[2:]  
    
        left_points = left_points[np.argsort(left_points[:, 1])]
    
        # 右侧点：按Y值从小到大（从上到下）
        right_points = right_points[np.argsort(right_points[:, 1])]
    
        tl = left_points[0]  # 左上 (top-left)
        bl = left_points[1]  # 左下 (bottom-left)
        br = right_points[1]  # 右下 (bottom-right)
        tr = right_points[0]  # 右上 (top-right)
    
        # 返回标准顺序：左上、右上、右下、左下
        return np.array([tl, tr, br, bl], dtype="float32")

    def detect_arrow(self,origin_frame):
        if origin_frame is None:
            print("fail to grab image")
            return

        self.result_frame = np.copy(origin_frame)
        self.convex_hull = None
        self.center = None
        self.target_point = None
        self.direction = Direction.UNKNOWN
        
        mask = self.getGreenFrame(origin_frame)
        if cv2.__version__.startswith('3'):
        # OpenCV 3.x
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
        # OpenCV 4.x
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if self.direction != Direction.UNKNOWN:#已经找到了
                break
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            epsilon = self.approx_param * cv2.arcLength(contour, True)
            approx_contour = cv2.approxPolyDP(contour, epsilon, True)
            self.convex_hull = cv2.convexHull(approx_contour)
            # 检查凸包顶点数量是否为五个
            if len(self.convex_hull) == 5:
                
                sorted_indices = self.SortConvexHull()#获取索引
            
                for id in sorted_indices:
                    if self.judgeTarget(id):
                        self.target_point = self.convex_hull[id]
                        center_x = self.center[0][0]
                        self.direction = Direction.Left if self.target_point[0][0] < center_x else Direction.Right
                        id_1 = (id + 1 - 5) if (id + 1) > 4 else (id + 1)
                        id_2 = (id - 1 + 5) if (id - 1) < 0 else (id - 1)
                        id_3 = (id + 2 - 5) if (id + 2) > 4 else (id + 2)
                        id_4 = (id - 2 + 5) if (id - 2) < 0 else (id - 2)
                        
                        point1 = self.convex_hull[id_1][0]
                        point2 = self.convex_hull[id_2][0]
                        point3 = self.convex_hull[id_3][0]
                        point4 = self.convex_hull[id_4][0]
                        pts = np.array([point1,point2,point3,point4])
                        self.rect = self.order_points(pts) 
                        self.rect = self.rect/2
                        break    

        if self.direction != Direction.UNKNOWN:
            #计算距离
            if self.direction == Direction.Left:
                self.object_points = np.array([
                    [-0.058,0.043,0],
                    [0.058,0.0215,0],
                    [0.058,-0.0215,0],
                    [-0.058,-0.043,0]
                ],dtype = np.float32)
            else:
                self.object_points = np.array([
                    [-0.058,0.0215,0],
                    [0.058,0.043,0],
                    [0.058,-0.043,0],
                    [-0.058,-0.0215,0]
                ],dtype = np.float32)

            success, rvec, tvec = cv2.solvePnP(
                self.object_points,  # 3D点
                self.rect,   # 2D点
                self.InnerMatrix,
                self.distCoeffs
            )
            if success:
                self.distance = tvec[2]
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                yaw_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                self.yaw_deg = np.degrees(yaw_rad)

        return self.direction

    #返回箭头方向
    def get_arrow_direction(self,real_rgb_camera_node):
        left_count = 0
        right_count = 0
        for i in range(11):
            frame = real_rgb_camera_node.get_new_frame()
            direction = self.detect_arrow(frame)
            if direction == Direction.Left:
                left_count += 1
            if direction == Direction.Right:
                right_count += 1

        if left_count > right_count:
            most_frequent_direction = Direction.Left
        elif left_count < right_count:
            most_frequent_direction = Direction.Right
        else:
            most_frequent_direction = Direction.UNKNOWN

        if most_frequent_direction == Direction.UNKNOWN:
            #print("Direction UNKNOWN!")
            return False,most_frequent_direction

        else:
            return True,most_frequent_direction

    #利用箭头调整朝向和距离
    def get_arrow_adjust(self,real_rgb_camera_node):
        yaw_deg_list = []
        distance_list = []
        for i in range(7):
            frame = real_rgb_camera_node.get_new_frame()
            direction = self.detect_arrow(frame)
            if direction != Direction.UNKNOWN:
                yaw_deg_list.append(self.yaw_deg)
                distance_list.append(self.distance)
        
        if not yaw_deg_list or not distance_list:
            #print("未检测到有效箭头数据，使用默认值")
            return 10, 1.5  # 返回默认值
        
        yaw_deg_np = np.array(yaw_deg_list)
        distance_np = np.array(distance_list)
        median_yaw_deg = np.median(yaw_deg_np)
        median_distance = np.median(distance_np)

        return median_yaw_deg , median_distance
class duiqi():
    def __init__(self):
        self.angle = 0
        self.angle_iny = 0
        self.yellow_y = 0
        self.yellow_y_iny = 0
        #self.yellow_x = 0
        self.distance = 0
        self.distance_iny = 0
        #self.x_distance = 0
        #直线距离参数
        self.ratio = 0.008
        self.ratio_iny = 0.008
        #没用
        self.x_right_ratio = 0.0027
        self.x_left_ratio = 0.0027
        self.find_line = True
    
        # 黄色检测参数
        # self.lab_lower = np.array([0, 0, 80])  # Lab颜色空间下限
        # self.lab_upper = np.array([255, 140, 255])  # Lab颜色空间上限
        
        # 边缘检测和直线检测参数
        self.canny_threshold1 = 50
        self.canny_threshold2 = 150
        self.hough_threshold = 50
        self.min_line_length = 30
        self.max_line_gap = 10
        self.horizontal_angle_tol = 20  # 角度容差范围(度)
        self.horizontal_angle_tol_iny = 15

        
        # 形态学处理参数
        self.kernel_size = (10, 1)  # 水平方向的核
        self.min_contour_area = 100

        # 调试图像路径
        self.debug_img_path = IMG_SAVE_DIR
        os.makedirs(self.debug_img_path, exist_ok=True)
        self.img_counter = 0 
    def get_yellow_line(self, frame,Right):
        """检测黄色横线并计算角度和Y坐标平均值"""
        # 截取图像下三分之二区域
        self.find_line = True
        height, width = frame.shape[:2]
        roi_height = int(height * 1 / 3)
        roi = frame[roi_height:, :].copy()
        roi_width = roi.shape[1]
        
        # 1. 增强处理（适度优化）
        # HSV亮度增强（解决暗区）
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv[:,:,2] = cv2.equalizeHist(hsv[:,:,2])  # 增强亮度
        hsv[:,:,1] = cv2.add(hsv[:,:,1], 30)       # 轻微增强饱和度
        
        # 2. 直接在HSV空间检测黄色（无需转换为Lab）
        # 黄色在HSV空间的范围：H在15-30度之间
        yellow_mask = cv2.inRange(hsv, (15, 80, 80), (30, 255, 255))
        
        # 3. 后处理优化
        # 应用形态学操作替代高斯模糊+阈值
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        
        # 4. 后续处理保持不变
        # 形态学处理增强特征
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.kernel_size)
        morph = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)        
        # Canny边缘检测
        edges = cv2.Canny(morph, self.canny_threshold1, self.canny_threshold2)
        
        # 霍夫变换检测所有直线
        lines = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        # 保存调试图像
        # self._save_debug_images(roi, yellow_mask, morph, edges)
        
        # 如果没有检测到直线则返回
        if lines is None:
            print("No lines detected")
            self.angle = 0
            self.yellow_y = 0
            self.distance = 0
            self.find_line = False
            return morph, None
        
        # 筛选横线并计算中点和角度
        candidates = []  # 存储候选横线信息 (中点x, 中点y, 角度, 线段)
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 计算线段中点
            if Right == True:
                mid_x = min(x1, x2)
            else: 
                mid_x = max(x1, x2)
            mid_y = (y1 + y2)/2
            
            # 计算线段角度（区分正负）
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            # 归一化到[-90, 90]范围
            if angle > 90:
                angle -= 180
            elif angle < -90:
                angle += 180
            
            # 筛选横线（考虑容差范围）
            if abs(angle) < self.horizontal_angle_tol:
                # 保存候选信息
                candidates.append({
                    'mid_x': mid_x,
                    'mid_y': mid_y,
                    'angle': angle,
                    'line': line[0]
                })
        
        # 如果没有符合的横线则返回
        if not candidates:
            print("No horizontal lines detected")
            self.angle = 0
            self.yellow_y = 0
            self.distance = 0
            #self.yellow_x = 0
            self.find_line = False
            return morph, None
        
        # 按mid_X坐标排序（从左到右）
        candidates.sort(key=lambda c: c['mid_x'])
        
        # 选择最左(右)侧的5条横线（或所有少于5条的）
        if Right:
            selected = candidates[:5] if len(candidates) > 5 else candidates

        # 检查是否在合理范围内：距离最左(右)横线不超过半屏
            min_x = selected[0]['mid_x']
            max_allowed_x = min_x + roi_width / 2
        # 过滤超出范围的线段
            filtered = [c for c in selected if c['mid_x'] <= max_allowed_x]
        else:
            selected = candidates[-5:] if len(candidates) > 5 else candidates
            max_x = selected[-1]['mid_x']
            max_allowed_x = max_x - roi_width / 2
            filtered = [c for c in selected if c['mid_x'] >= max_allowed_x]
        

        
        # 如果没有符合的线段则使用原始选择
        if not filtered:
            print("All selected lines are too far, using all candidates")
            filtered = selected
        
        # 提取角度和Y坐标
        angles = [c['angle'] for c in filtered]
        y_coords = [c['mid_y'] + roi_height for c in filtered]  # 转换为全图Y坐标
        # x_coords = [c['mid_x'] for c in filtered]   保存X坐标
        # x_coords = min_x + np.array([c['mid_x'] - min_x for c in filtered]) 
        # 计算平均值
        self.angle = -np.mean(angles) if angles else 0
        self.yellow_y = np.mean(y_coords) if y_coords else 0
        # if Right:
        #     self.yellow_x = min_x
        # else:
        #     self.yellow_x = max_x
        # 计算距离
        self.distance = (frame.shape[0] - self.yellow_y) * self.ratio if self.yellow_y else 0
        # if  Right:
        #     self.x_distance = -(self.yellow_x - 155) * self.x_right_ratio if self.yellow_x else 0    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!155是直接测的 摆正狗 直接测yellow_x x_ratio也是直接测的     #!!!!!!!!!!155是直接测的 摆正狗 直接测yellow_x x_ratio也是直接测的 
        # else:
        #     self.x_distance = -(self.yellow_x - 490) * self.x_left_ratio if self.yellow_x else 0    #!!!!!!!!!!155是直接测的 摆正狗 直接测yellow_x x_ratio也是直接测的     #!!!!!!!!!!155是直接测的 摆正狗 直接测yellow_x x_ratio也是直接测的 

        # # 提取所有选中的线段（包含ROI坐标系中）
        selected_lines = [c['line'] for c in filtered]
        
        # # 创建可视化图像
        # vis_img = frame.copy()
        
        # # 绘制所有候选线（灰色）
        # for c in candidates:
        #     x1, y1, x2, y2 = c['line']
        #     cv2.line(vis_img, 
        #              (x1, y1 + roi_height), 
        #              (x2, y2 + roi_height), 
        #              (150, 150, 150), 1)  # 灰色表示所有候选线
            
        # # 绘制最终选中的线（绿色）
        # for i, c in enumerate(filtered):
        #     x1, y1, x2, y2 = c['line']
        #     cv2.line(vis_img, 
        #              (x1, y1 + roi_height), 
        #              (x2, y2 + roi_height), 
        #              (0, 255, 0), 2)  # 绿色表示最终选中的线
            
        #     # 标记角度值
        #     angle_str = f"{c['angle']:.1f}°"
        #     mid_x = int(x1)
        #     mid_y = int((y1 + y2) / 2) + roi_height
        #     cv2.putText(vis_img, angle_str, 
        #                (mid_x, mid_y), 
        #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # # 标记平均角度和Y值
        # cv2.putText(vis_img, f"Avg Angle: {self.angle:.1f}°", 
        #            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # cv2.putText(vis_img, f"Avg Y: {self.yellow_y:.1f}", 
        #            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # cv2.putText(vis_img, f"Dist: {self.distance:.1f}mm", 
        #            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # # 绘制选择范围（红色半屏线）
        # cv2.line(vis_img, 
        #          (int(max_allowed_x), roi_height), 
        #          (int(max_allowed_x), height), 
        #          (0, 0, 255), 1)
        # cv2.putText(vis_img, "Selection Boundary", 
        #            (int(max_allowed_x) + 5, roi_height + 30), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # cv2.imwrite(f"{self.debug_img_path}result_{self.img_counter:04d}.png", vis_img)
        # self.img_counter += 1
        
        # 返回所有选中的线段（在ROI坐标系
        return morph, selected_lines
    
    def get_yellow_line_y(self, frame):
            """检测黄色横线并选择最上方的5条横线"""
            self.find_line = True
            height, width = frame.shape[:2]
            
            # 截取图像下三分之二区域
            roi_height = int(height * 1 / 4)
            roi = frame[roi_height:, :].copy()
            roi_height_px = roi.shape[0]  # ROI区域的高度（像素）
            
            # 1. 增强处理（与get_yellow_line相同）
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            hsv[:,:,2] = cv2.equalizeHist(hsv[:,:,2])
            hsv[:,:,1] = cv2.add(hsv[:,:,1], 30)
            
            # 2. HSV空间检测黄色 正式的
            yellow_mask = cv2.inRange(hsv, (15, 80, 80), (30, 255, 255))
            #test 早上
            # yellow_mask = cv2.inRange(hsv, (20, 50, 80), (40, 255, 255))
            # 3. 后处理优化
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
            
            # 4. 形态学处理增强特征
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.kernel_size)
            morph = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)        
            
            # 5. Canny边缘检测
            edges = cv2.Canny(morph, self.canny_threshold1, self.canny_threshold2)
            
            # 6. 霍夫变换检测直线
            lines = cv2.HoughLinesP(
                edges, 
                rho=1, 
                theta=np.pi/180, 
                threshold=self.hough_threshold,
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap
            )
            
            # 7. 如果没有检测到直线则返回
            if lines is None:
                print("No lines_y detected")
                self.angle_iny = 0
                self.yellow_y_iny = 0
                self.distance_iny= 0
                self.find_line = False
                return morph, None
            
            # 8. 筛选横线并计算中点和角度
            candidates = []
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_y = (y1 + y2)/2  # 只关注Y坐标
                
                # 计算线段角度
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                if angle > 90: angle -= 180
                elif angle < -90: angle += 180
                
                # 筛选横线（考虑容差范围）
                if abs(angle) < self.horizontal_angle_tol_iny:
                    candidates.append({
                        'mid_y': mid_y,
                        'angle': angle,
                        'line': line[0]
                    })
            
            if not candidates:
                print("No horizontal lines detected")
                self.angle_iny = 0
                self.yellow_y_iny = 0
                self.distance_iny = 0
                self.find_line = False
                return morph, None
            
            # 9. 按Y坐标排序（从小到大，即从高到低）
            candidates.sort(key=lambda c: c['mid_y'])
            
            # 10. 选择最上方的线作为基准
            highest_y = candidates[0]['mid_y']
            
            # 11. 确定最大允许Y坐标差（1/8图像高度）
            max_y_diff = roi_height_px * 0.0625
            
            # 12. 筛选与最高线相近的线（最多5条）
            filtered = []
            for c in candidates:
                # 检查是否超过最大Y差
                if c['mid_y'] - highest_y > max_y_diff:
                    break
                
                filtered.append(c)
                
                # 最多取5条符合条件的线
                if len(filtered) >= 5:
                    break
            
            # 13. 提取角度和Y坐标
            angles = [c['angle'] for c in filtered]
            y_coords = [c['mid_y'] + roi_height for c in filtered]  # 转换为全图Y坐标
            
            # 14. 计算平均值
            self.angle_iny = -np.mean(angles) if angles else 0
            self.yellow_y_iny = np.mean(y_coords) if y_coords else 0
            self.distance_iny = (frame.shape[0] - self.yellow_y_iny) * self.ratio_iny if self.yellow_y_iny else 0
            
            # 15. 提取选中的线段（包含ROI坐标系中）
            selected_lines = [c['line'] for c in filtered]

            # # 创建可视化图像
            # vis_img = frame.copy()
            
            # # 绘制所有候选线（灰色）
            # for c in candidates:
            #     x1, y1, x2, y2 = c['line']
            #     cv2.line(vis_img, 
            #             (x1, y1 + roi_height), 
            #             (x2, y2 + roi_height), 
            #             (150, 150, 150), 1)  # 灰色表示所有候选线
            
            # # 绘制最终选中的线（绿色）
            # for i, c in enumerate(filtered):
            #     x1, y1, x2, y2 = c['line']
            #     cv2.line(vis_img, 
            #             (x1, y1 + roi_height), 
            #             (x2, y2 + roi_height), 
            #             (0, 255, 0), 2)  # 绿色表示最终选中的线
                
            #     # 标记角度值
            #     angle_str = f"{c['angle']:.1f}°"
            #     mid_x = int((x1 + x2) / 2)
            #     mid_y = int((y1 + y2) / 2) + roi_height
            #     cv2.putText(vis_img, angle_str, 
            #             (mid_x, mid_y), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # # 标记平均角度和Y值
            # cv2.putText(vis_img, f"Avg Angle: {self.angle:.1f}°", 
            #         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # cv2.putText(vis_img, f"Avg Y: {self.yellow_y:.1f}", 
            #         (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # cv2.putText(vis_img, f"Dist: {self.distance:.1f}mm", 
            #         (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # # 绘制最高线位置（红色线）
            # cv2.line(vis_img, 
            #         (0, int(highest_y + roi_height)), 
            #         (width, int(highest_y + roi_height)), 
            #         (0, 0, 255), 1)
            # cv2.putText(vis_img, "Highest Line", 
            #         (10, int(highest_y + roi_height) - 10), 
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # # 绘制最大允许Y差范围（蓝色线）
            # max_y_line = int(highest_y + max_y_diff + roi_height)
            # cv2.line(vis_img, 
            #         (0, max_y_line), 
            #         (width, max_y_line), 
            #         (255, 0, 0), 1)
            # cv2.putText(vis_img, "Max Y Diff Boundary", 
            #         (10, max_y_line - 10), 
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # # 保存可视化图像
            # cv2.imwrite(f"{self.debug_img_path}result_y_{self.img_counter:04d}.png", vis_img)
            
            # # 保存中间处理图像
            # cv2.imwrite(f"{self.debug_img_path}roi_y_{self.img_counter:04d}.png", roi)
            # cv2.imwrite(f"{self.debug_img_path}mask_y_{self.img_counter:04d}.png", yellow_mask)
            # cv2.imwrite(f"{self.debug_img_path}morph_y_{self.img_counter:04d}.png", morph)
            # cv2.imwrite(f"{self.debug_img_path}edges_y_{self.img_counter:04d}.png", edges)
            
            # self.img_counter += 1

            return morph, selected_lines

    def _save_debug_images(self, roi, mask, morph, edges):
        """保存调试过程图像"""
        save_debug_image(f"roi_{self.img_counter:04d}.png", roi)
        save_debug_image(f"mask_{self.img_counter:04d}.png", mask)
        save_debug_image(f"morph_{self.img_counter:04d}.png", morph)
        save_debug_image(f"edges_{self.img_counter:04d}.png", edges)
    
    # def get_distance(self, frame):
    #     """计算黄色横线与图像底部的距离"""
    #     _, lines = self.get_yellow_line(frame)
        
    #     # 如果没有检测到直线，返回-1表示失败
    #     if lines is None:
    #         self.distance = -1
    #         return self.distance
        
    #     # 计算距离（使用已计算的self.yellow_y）
    #     self.distance = (frame.shape[0] - self.yellow_y) * self.ratio
    #     return self.distance
    
    # def get_angle(self, frame):
    #     self.get_yellow_line(frame)
    #     print(f"self.angle: {self.angle}")
    #     return self.angle

def jiaozheng_bianxian(yaw): #弧度
    w_bia = 0
    msg.mode=11
    msg.gait_id=27
    yaw_speed = 20/180*3.14     
    msg.duration= int(abs(yaw/yaw_speed)*1000)
    if yaw < 0:
        yaw_speed = -yaw_speed - w_bia
    elif yaw > 0:
        yaw_speed = yaw_speed - w_bia
    elif yaw == 0:
        pass
    print(f"yaw_speed: {yaw_speed}")
    msg.vel_des=[0.0,0.0,yaw_speed]
    msg.step_height = [0.01, 0.01]
    msg.life_count=(msg.life_count+1)%128    
    Ctrl.Send_cmd(msg)
    time.sleep(3)
#new!!!!!!!!!!
def go_short_bianxian(vel, duration):  # 前进
    msg.mode = 11
    msg.gait_id = 27
    msg.vel_des = [vel, 0.0, 0.0]
    msg.step_height = [0.01, 0.01]
    msg.duration = duration
    msg.life_count = (msg.life_count + 1) % 128
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_5(11, 27)
    time.sleep(1.3)

def duiqi_control(Right):
    count = 0
    while(True):
        count += 1
        frame = real_rgb_camera.get_new_frame()
        duiqi_node.get_yellow_line(frame,Right)
        print(f"Angle: {duiqi_node.angle}")
        print(f"find_line:{duiqi_node.find_line}")

        if duiqi_node.find_line == False:
            if count <= 2:
                if Right == True:
                    jiaozheng_bianxian(4*15*3.14/180)
                    frame = real_rgb_camera.get_new_frame()
                    duiqi_node.get_yellow_line(frame,Right)
                else:
                    jiaozheng_bianxian(-4*15*3.14/180)
                    frame = real_rgb_camera.get_new_frame()
                    duiqi_node.get_yellow_line(frame,Right)
            else:
                break

        # print(f"Distance: {duiqi_node.distance}")
        #time.sleep(100000)

        if duiqi_node.angle > 0.2 and duiqi_node.angle < 6:
            jiaozheng_bianxian(4.5*duiqi_node.angle*3.14/180)
        elif duiqi_node.angle >= 6:
            jiaozheng_bianxian(4.4*duiqi_node.angle*3.14/180)
        elif duiqi_node.angle < -0.2 and duiqi_node.angle > -6:
            jiaozheng_bianxian(4.4*duiqi_node.angle*3.14/180)
        elif duiqi_node.angle <= -6:
            jiaozheng_bianxian(4.8*duiqi_node.angle*3.14/180)
        print("angle_over")
        # frame = real_rgb_camera.get_new_frame()
        # duiqi_node.get_yellow_line(frame,Right)
        # vel = (duiqi_node.x_distance)/2
        # print(f"X_distance: {duiqi_node.x_distance}")
        # go_x_bianxian(vel,2000)


        frame = real_rgb_camera.get_new_frame()
        duiqi_node.get_yellow_line(frame,Right)
        print("distance_start")

        if duiqi_node.distance <= 0.6 and duiqi_node.distance > 0:
            vel = (duiqi_node.distance- 0.2)/1
            print(f"Distance: {duiqi_node.distance}, Vel: {vel}")
            #time.sleep(1)
            #standup()
            #print("standup over")
            go_short_bianxian(vel,1000)
        elif duiqi_node.distance > 0.6: 
            print(f"Distance: {duiqi_node.distance},vel: 0.2")
            #time.sleep(1)
            #standup()
            #print("standup over")
            go_short_bianxian(0.2, 1500)
        if(duiqi_node.distance <= 0.2 or duiqi_node.distance == -1 or duiqi_node.distance -0.3 <= 0.2):
            print("duiqi_over")
            break

duiqi_node = duiqi()

class s_curve_processor():
    def __init__(self):
        #处理对象
        self.left = False
        #原始图
        self.origin_frame = None
        self.origin_frame_change = None
        #重心参考位置
        self.reference_y = 0
        #roi/绘制图
        self.bottom_roi = None
        self.roi_change = None
        #黄色阈值
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        #重心位置
        self.center_x = 0
        self.center_y = 0
        #调整
        self.tolerance_ratio = 1/4
        self.adjust = adjustment.mid
        #椭圆斜率
        self.angle = None
        self.rotate_speed = 0
        #
        self.coeffs = 0
        self.fit_y = 0
        self.points = 0

    def find_target_contour(self,image):
        target_contour = None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
    
        kernel = np.ones((5,5), np.uint8)
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        if cv2.__version__.startswith('3'):
        # OpenCV 3.x
            _, contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
        # OpenCV 4.x
            contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #找重心
        target = None
        center_x = 0
        center_y = 0
        for index, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            M = cv2.moments(cnt)
            if M["m00"] != 0:  
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if cY > center_y:
                    center_y = cY
                    center_x = cX
                    target = index

        if target == None:
            return False,target_contour
        else:
            target_contour = contours[target]
            return True,target_contour


    def image_process(self,frame):
        self.origin_frame = frame.copy()
        height, width = frame.shape[:2]
        
        roi_y_start = int(height * 2/3)
        roi_y_end = height
        # 1/3和3/4
        if self.left:
            roi_x_start = int(width * 1/4)
            roi_x_end = width
        else:
            roi_x_start = 0
            roi_x_end = int(width * 3/4)
        # ROI
        self.bottom_roi = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        roi_height , roi_width = self.bottom_roi.shape[:2]
        self.reference_y = roi_height//2

        findflag,target_contour = self.find_target_contour(self.bottom_roi)
        if not findflag:
            return False
        
        M = cv2.moments(target_contour)
        if M["m00"] != 0:  
            self.center_x = int(M["m10"] / M["m00"])
            self.center_y = int(M["m01"] / M["m00"])
        #找到了
        self.angle = None
        ellipse = cv2.fitEllipse(target_contour)
        angle = self.get_angle(ellipse)
        if abs(angle) <= 45:
            self.angle = angle

        return True


    def fit_yellow_curve(self, frame, flag_change, num_points=5, point_interval=30):     #参数:frame: 输入图像 num_points: 左右各取点的数量 point_interval: 点之间的间隔(像素列数)
        self.origin_frame_change = frame.copy()
        height, width = frame.shape[:2]
    
        roi_y_start = int(height * 2/3)
        roi_y_end = height
        if flag_change == 0:
            roi_x_start = int(width * 1/4)
            roi_x_end = width
        else:
            roi_x_start = 0
            roi_x_end = int(width * 3/4)
        
        self.roi_change = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        roi_height, roi_width = self.roi_change.shape[:2]
        
        findflag,target_contour = self.find_target_contour(self.roi_change)
        if not findflag:
            return False
            
        center_x = roi_width // 2
        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
        cv2.drawContours(mask, [target_contour], contourIdx=-1, color=255, thickness=-1)
        # 在左右方向等间隔取样点
        points = []
        
        # 向右取样点
        for i in range(num_points):
            x = center_x + i * point_interval
            if x >= roi_width: 
                break
                
            # 在x位置取一个垂直列
            col = mask[:, x]
            
            # 计算当前列的黄色区域中心Y坐标
            if np.any(col > 0):
                y_center = np.mean(np.where(col > 0)[0])
                points.append([x, y_center])
        
        # 向左取样点
        for i in range(num_points):
            x = center_x - (i + 1) * point_interval  
            if x < 0: 
                break
                
            # 在x位置取一个垂直列
            col = mask[:, x]
            
            # 计算当前列的黄色区域中心Y坐标
            if np.any(col > 0):
                y_center = np.mean(np.where(col > 0)[0])
                points.append([x, y_center])
           
        if len(points) < 10: 
            return False
        
        # 二次多项式拟合 (y = ax² + bx + c)
        self.points = np.array(points)
        x_coords = self.points[:, 0]
        y_coords = self.points[:, 1]
        self.coeffs = np.polyfit(x_coords, y_coords, 2)

        a, b, c = self.coeffs
        
        fit_x = np.arange(0, roi_width, 1)
        self.fit_y = a * fit_x**2 + b * fit_x + c
        return True
        
    def get_angle(self,ellipse):
        (center, axes, angle) = ellipse
    
        if axes[0] < axes[1]:
            angle += 90
    
        # 将角度转换到[-90,90]范围
        angle = angle % 180
        if angle > 90:
            angle -= 180
    
        return angle


    def main_process(self,image):
        target = self.image_process(image)
        if target == False:
            return False
    
        upper_y = int(self.reference_y * (1 + self.tolerance_ratio*1.2))
        lower_y = int(self.reference_y * (1 - self.tolerance_ratio*0.7))
        if self.center_y < upper_y and self.center_y > lower_y:
            self.adjust = adjustment.mid
        if self.center_y <= lower_y:
            self.adjust = adjustment.near
        if self.center_y >= upper_y:
            self.adjust = adjustment.faraway
        
        return True

class SteroCameraNode(Node):
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.is_node_running = False
        self.left = False
        
        self.bridge = CvBridge()

        self.data_lock_left = Lock()
        self.data_lock_right = Lock()

        self.frame_left = None
        self.frame_right = None

        self.update_left = False
        self.update_right = False
        # self.InnerMatrix = np.array([[175.25,0,160],[0,175.25,90],[0,0,1]],dtype=np.float32)
        # self.distCoeffs = np.array([0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        # self.cameraToRobot = [275.76,0,125.794]
        self.subscriber_left = self.create_subscription(Image,"/mi_desktop_48_b0_2d_5f_be_5c/image_left",self.ImageCallBackLeft,qos_profile)
        self.subscriber_right = self.create_subscription(Image,"/mi_desktop_48_b0_2d_5f_be_5c/image_right",self.ImageCallBackRight,qos_profile)

    def get_new_frame(self,left):
        if left == True:
            while(True):
                if self.update_left == False or self.frame_left.all == None:
                    time.sleep(0.01)
                else:
                    with self.data_lock_left:
                        self.update_left = False
                        return self.frame_left
        else:            
            while(True):
                if self.update_right == False or self.frame_right.all == None:
                    time.sleep(0.01)
                else:
                    with self.data_lock_right:
                        self.update_right = False
                        return self.frame_right
                

    def ImageCallBackLeft(self,receive_msg):
        if(self.is_node_running):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(receive_msg, desired_encoding='bgr8')
                with self.data_lock_left:
                    self.frame_left = cv_image
                    self.update_left = True

            except Exception as e:
                self.get_logger().error(f"Exception in callback: {e}")
                self.is_node_running = False
        else:
            pass

    def ImageCallBackRight(self,receive_msg):
        if(self.is_node_running):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(receive_msg, desired_encoding='bgr8')
                with self.data_lock_right:
                    self.frame_right = cv_image
                    self.update_right = True

            except Exception as e:
                self.get_logger().error(f"Exception in callback: {e}")
                self.is_node_running = False
        else:
            pass

#rgb for s_cur
class RGBCameraNode(Node):
    def __init__(self,name):
        super().__init__(name)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.is_node_running = False
        #self.rgb = False
        
        self.bridge = CvBridge()

        self.data_lock_rgb = Lock()
        #self.data_lock_right = Lock()

        self.frame_rgb = None
        #self.frame_right = None

        self.update_rgb = False
        #self.update_right = False
        # self.InnerMatrix = np.array([[175.25,0,160],[0,175.25,90],[0,0,1]],dtype=np.float32)
        # self.distCoeffs = np.array([0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        # self.cameraToRobot = [275.76,0,125.794]
        self.subscriber_rgb = self.create_subscription(Image,"/mi_desktop_48_b0_2d_5f_be_5c/image_rgb",self.ImageCallBackrgb,qos_profile)
        #self.subscriber_right = self.create_subscription(Image,"/mi_desktop_48_b0_2d_5f_be_5c/image_right",self.ImageCallBackRight,qos_profile)

    def get_new_frame(self):
        while(True):
            #print("getnewframe")
            if self.update_rgb == False or self.frame_rgb.all == None:
                time.sleep(0.01)
            else:
                with self.data_lock_rgb:
                    self.update_rgb = False
                    return self.frame_rgb

                

    def ImageCallBackrgb(self,receive_msg):
        if(self.is_node_running):
            #print("nodeisrunning")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(receive_msg, desired_encoding='bgr8')
                with self.data_lock_rgb:
                    self.frame_rgb = cv_image
                    self.update_rgb = True

            except Exception as e:
                self.get_logger().error(f"Exception in callback: {e}")
                self.is_node_running = False
        else:
            pass

#语音节点
class SpeechProcessor(Node):
    """保留完整语音和电池检测功能的ROS2节点"""
    def __init__(self):
        super().__init__('speech_processor')
        
        # 初始化运动控制器（与官方例程相同的初始化顺序）
        
        # ROS2通信接口（保持原有功能）
        self.asr_subscription = self.create_subscription(
            String, '/mi_desktop_48_b0_2d_5f_be_5c/asr_text',
            self.asr_callback, 10)
        # self.bms_subscription = self.create_subscription(
        #     BmsStatus, '/mi_desktop_48_b0_2d_5f_be_5c/bms_status',
        #     self.bms_callback, 10)
        # self.speech_publisher = self.create_publisher(
        #     AudioPlayExtend, '/mi_desktop_48_b0_2d_5f_be_5c/speech_play_extend', 10)
        self.speech_publisher = None
        
        # 状态标志（保持原有功能）
        self.countdown_active = False
        self.current_count = 0
        self.countdown_timer = None
        self.is_charging = False
        self.was_charging = False

    def asr_callback(self, msg):
        global now_state
        """保持原有语音处理逻辑"""
        text = msg.data.strip()
        print(text)
        print(now_state)
        
        
        if text == "启动":
            now_state = 1
            self.play_speech("启动成功")
            
        elif text == "黄灯":
            self.start_countdown()
            
        else:
            self.play_speech(text)
        

    def bms_callback(self, msg):
        """保持原有电池检测逻辑"""
        self.was_charging = self.is_charging
        self.is_charging = msg.power_wired_charging
        
        if self.was_charging and not self.is_charging:
            self.handle_disconnection()
            
        elif not self.was_charging and self.is_charging:
            self.play_speech("充电中")

    # 以下为新增的功能方法（保持原有业务逻辑）
    def start_countdown(self):
        """黄灯倒计时处理"""
        if self.countdown_timer:
            self.countdown_timer.cancel()
        self.countdown_active = True
        self.current_count = 5
        self.countdown_timer = self.create_timer(1.0, self.countdown_callback)

    def handle_disconnection(self):
        """充电线拔出处理流程"""
        self.play_speech("充电线拔出")
        
        # 严格遵循官方例程的动作顺序
        msg = robot_control_cmd_lcmt()
        msg.mode = 12  # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1
        self.motion_controller.Send_cmd(msg)
        self.motion_controller.Wait_finish(12, 0)
        
        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_SLOW
        msg.vel_des = [0.2, 0, 0]
        msg.duration = 4000
        msg.life_count += 1
        self.motion_controller.Send_cmd(msg)
        self.motion_controller.Wait_finish(11, 27)
        
        msg.mode = 7  # PureDamper
        msg.life_count += 1
        self.motion_controller.Send_cmd(msg)
        self.motion_controller.Wait_finish(7, 0)

    def play_speech(self, text):
        """统一语音播报接口"""
        # msg = AudioPlayExtend()
        # msg.module_name = "voice_interaction"
        # msg.is_online = True
        # msg.text = text
        # self.speech_publisher.publish(msg)
        self.get_logger().info(f'语音播报(已禁用协议发布): {text}')

    def countdown_callback(self):
        """倒计时处理"""
        if self.current_count > 0:
            self.play_speech(str(self.current_count))
            self.current_count -= 1
        else:
            self.countdown_timer.cancel()
            self.countdown_active = False

    def destroy_node(self):
        """资源清理（与官方例程一致）"""
        self.motion_controller.stop()
        super().destroy_node()


#视觉节点
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10)
        self.frame = None
        self.frame_nose = None
        self.update = False
        self.update_nose = False
        self.vis_running = True
        #相机内参
        self.InnerMatrix = np.array([
            [
                416.6850297391639,
                0.0,
                319.98128281389444
            ],
            [
                0.0,
                415.51316371220037,
                227.93040503278888
            ],
            [
                0.0,
                0.0,
                1.0
            ]
        ],dtype=np.float32)
        self.distCoeffs = np.array([0.0024461913801336077,
            -0.04339171678852136,
            -0.006551805033021212,
            -0.0003024566454255968,
            0.0],dtype=np.float32)
        #相机外参
        self.cameraToRobot = [275.76,0,125.794]#单位毫米
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        # 只初始化rgb订阅
        self.subscription = self.create_subscription(
            Image,
            '/mi_desktop_48_b0_2d_5f_be_5c/image_rgb',
            self.rgb_callback,
            qos_profile)
        self.sub_nose = None
        self.current_mode = 'rgb'  # 当前订阅模式
        # self.ArrowDetector = ArrowDetector()
        # self.QRCodeDetector = QRCodeDetector(self.InnerMatrix,self.distCoeffs)
        # self.YellowLightDetector = YellowLightDetector(self.InnerMatrix,self.distCoeffs)
        # self.LimitHeightDetector = LimitHeightDetector(self.InnerMatrix,self.distCoeffs)
        self.lower_yellow = np.array([20, 100, 100])    # 原 [20,100,100]
        self.upper_yellow = np.array([30, 255, 255])  # 原 [30,255,255]
        self.detection_threshold = 0.15  # 25%黄色像素占比
        self._yaw_adjust = 0.0
        self._lateral_error = 0.0
        self._max_side_length = 0
        self._line_angle = 0.0  # 检测到的线条角度（弧度）
        self._line_threshold = 0.05 # 角度误差阈值（弧度）
        self._line_1 = 0.015
        self._alignment_start_time = 0.0  # 对齐开始时间戳
        self._aligned_duration = 0.0      # 已对齐持续时间
        self._state9_threshold = 0.15
        self._state10_threshold = 0.15
        self._limit_height_distance = None
        self._yellow_light_distance = None
        self._detected = False
        # 斜坡检测参数（深灰色阈值）
        self.lower_slope = np.array([0, 0, 30], dtype=np.uint8)   # HSV下限（深灰）
        self.upper_slope = np.array([180, 50, 90], dtype=np.uint8) # HSV上限
        self.min_slope_area = 500  # 最小轮廓面积阈值
        self.slope_top_y = None    # 斜坡顶部的y坐标
        self.prev_angle = 0
        self.integral = 0
        self.last_error = 0
        self.angle_0 = 0
        self._lock = Lock()
        self.real_debug_mode = False
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
            '17': False,
            '17.5': False,
            '18': False,
            '18_5': False,
            '19': False,
            '19_5': False,
            '19_55': False,
            '19_6': False,
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
            '36_5': False,
            '37': False,
            '38': False,
            '39': False,
            '39_55': False,
            '40': True,
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
        self.debug_mode = False
        # 定时检查并切换订阅
        self.check_camera_timer = self.create_timer(0.2, self.check_camera_subscription)
        # 队列与线程
        self.image_queue = queue.Queue(maxsize=5)
        from threading import Thread
        self.worker_thread = Thread(target=self.image_worker, daemon=True)
        self.worker_thread.start()

    def check_camera_subscription(self):
        global state_id
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        # 只在3,4,4.5时用nose
        if state_id in [99, 100]:
            if self.current_mode != 'nose':
                # 切换到nose
                if self.subscription is not None:
                    self.destroy_subscription(self.subscription)
                    self.subscription = None
                if self.sub_nose is None:
                    self.sub_nose = self.create_subscription(
                        Image,
                        '/mi_desktop_48_b0_2d_5f_be_5c/image',
                        self.nose_callback,
                        qos_profile)
                self.current_mode = 'nose'
        else:
            if self.current_mode != 'rgb':
                # 切换到rgb
                if self.sub_nose is not None:
                    self.destroy_subscription(self.sub_nose)
                    self.sub_nose = None
                if self.subscription is None:
                    self.subscription = self.create_subscription(
                        Image,
                        '/mi_desktop_48_b0_2d_5f_be_5c/image_rgb',
                        self.rgb_callback,
                        qos_profile)
                self.current_mode = 'rgb'

    def rgb_callback(self, msg):
        # print("[LOG] rgb_callback 收到新图像")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = cv_image
            self.update = True 
            # cv_image = cv2.undistort(cv_image, self.InnerMatrix, self.distCoeffs)
            if not self.image_queue.full():
                # print(f"[LOG] rgb_callback 放入队列, 队列长度: {self.image_queue.qsize()}")
                self.image_queue.put(("rgb", cv_image))
            else:
                # print("[WARN] image_queue 已满，丢弃图像")
                pass
        except Exception as e:
            # print(f"[ERROR] RGB图像处理失败: {str(e)}")
            self.get_logger().error(f"RGB图像处理失败: {str(e)}")

    def nose_callback(self, msg):
        """处理 /image 图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # cv_image = cv2.undistort(cv_image, self.InnerMatrix, self.distCoeffs)
            # self.frame_nose = cv_image.copy()
            # self.update_nose = True
            # self.process_image()
            if not self.image_queue.full():
                self.image_queue.put(("nose", cv_image))
        except Exception as e:
            self.get_logger().error(f"Nose图像处理失败: {str(e)}")

    def image_worker(self):
        # print("[LOG] image_worker 线程启动")
        while True:
            try:
                # print(f"[LOG] image_worker 等待队列，当前长度: {self.image_queue.qsize()}")
                img_type, cv_image = self.image_queue.get()
                if img_type == "nose":
                    cv_image = cv2.undistort(cv_image, self.InnerMatrix, self.distCoeffs)
                # print(f"[LOG] image_worker 取出图像, 类型: {img_type}")
                self.process_image(img_type, cv_image)
            except Exception as e:
                # print(f"[ERROR] 图像处理线程异常: {str(e)}")
                self.get_logger().error(f"图像处理线程异常: {str(e)}")

    def get_new_frame(self):
        while(True):
            if self.update == False:
                time.sleep(0.01)
                #print("getframebutfalse")
            else:
                self.update == False
                return self.frame

    def process_image(self, img_type, cv_image):
        global state_id, QR1, QR2
        try:
            # print(f"[LOG] process_image 被调用, img_type: {img_type}, state_id: {state_id}")
            # 只在state_id和img_type匹配时处理
            if state_id in [99]:
                if img_type != "nose":
                    # print("[LOG] process_image: state_id在[3,4,4.5]但img_type不是nose，返回")
                    return
            else:
                if img_type != "rgb":
                    # print("[LOG] process_image: state_id不在[3,4,4.5]但img_type不是rgb，返回")
                    return
            current_state = state_id  # 获取当前状态
            # print(f"[LOG] process_image: current_state={current_state}")
            if current_state == 0:
                # print("[LOG] process_image: 调用 _detect_0")
                self._detect_0(cv_image)
            elif current_state == 1 or current_state == 2 or current_state == 7 or current_state == 31 or current_state == 44 or current_state == 48 or current_state == 53 or current_state == 15: 
                pass
            elif current_state == 2.5:
                self._detect_2_5(cv_image)
            elif current_state == 3:
                self._detect_3(cv_image)
            elif current_state == 4:
                self._detect_4(cv_image)
            # elif current_state == 4.5:
            #     self._detect_4_5(cv_image)
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
            # elif current_state == 11:
                # self._detect_11(cv_image)
            elif current_state == 12:
                self._detect_12(cv_image)
            elif current_state == 13:
                self._detect_13(cv_image)
            elif current_state == 14:
                self._detect_14(cv_image)
            # elif current_state == 15:
            #     self._detect_15(cv_image)
            elif current_state == 16:
                self._detect_16(cv_image)
            elif current_state == 17:
                self._detect_17(cv_image)
            elif current_state == 17.5:
                pass
            elif current_state == 18:
                self._detect_18(cv_image)
            elif current_state == 18.5:
                self._detect_18_5(cv_image)
            # elif current_state == 19:
            #     self._detect_19(cv_image)
            elif current_state == 19.5:
                self._detect_19_5(cv_image)
            elif current_state == 19.55:
                self._detect_19_55(cv_image)
            elif current_state == 19.6:
                self._detect_19_6(cv_image)
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
            # elif current_state == 36:
            #     self._detect_36(cv_image)
            elif current_state == 36.5:
                self._detect_36_5(cv_image)
            elif current_state == 37:
                self._detect_37(cv_image)
            elif current_state == 38:
                self._detect_38(cv_image)
            elif current_state == 39:
                self._detect_39(cv_image)
            elif current_state == 39.55:
                self._detect_39_55(cv_image)
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
        # === 1. 预处理并转换为 HSV ===
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # === 2. 提取右下角 ROI 区域 ===
        roi_height1 = height // 2 - 60
        roi_height2 = roi_height1 - 10
        x_start = 7 * width // 8
        x_end = x_start + width // 8
        y_start = height - roi_height1
        y_end = height - roi_height2
        roi_hsv = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        # === 3. 使用模块函数生成“黄+泛白黄”掩码 ===
        '''mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        
        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)'''
        mask = cv2.inRange(roi_hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        yellow_ratio = np.count_nonzero(mask) / (mask.size + 1e-5)
        # print(yellow_ratio)
        # === 6. 检测逻辑（需持续500ms）===
        with self._lock:
            current_time = time.time()
            if yellow_ratio > YELLOW_RATIO_THRESHOLD:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['0'] = True
                self._last_detection_time = current_time
            else:
                self._detected_flags['0'] = False
        
        # === 7. 可视化调试 ===
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.imshow("Mask", mask_combined)
            cv2.waitKey(1)

        if self.real_debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, (x_start, y_start), (x_end, y_end), (0, 255, 255), 2)
            cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(debug_img, f"Detected: {self._detected_flags['0']}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['0'] else (0, 255, 0), 2)
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect0_{timestamp}.jpg"

            save_debug_image(filename, debug_img)
        
    def _detect_3(self, cv_image):
        print("进入检测3")

    def _detect_4(self, cv_image):
        
        print("进入检测4")

        #cv2.imwrite("test.jpg", cv_image)

        global QR1
        global QR1_cnt
        if cv_image is None:
            print("QRCodeDetector Error: no frame")
            return

        # 保证兼容 python2 和 python3
        IS_PY3 = sys.version_info.major == 3
        if IS_PY3:
            from urllib.request import urlopen, Request
            from urllib.error import URLError
            from urllib.parse import urlencode
        else:
            from urllib2 import urlopen, Request, URLError
            from urllib import urlencode

        # 防止 https 证书校验不正确
        ssl._create_default_https_context = ssl._create_unverified_context

        # API Key 和 Secret Key
        API_KEY = 'ZyIjArFIzoRtgnT8WAUw5tCv'
        SECRET_KEY = '9JlkvkYt7r0C0GsJGcuRyoMhWaJx12yF'

        # 获取 access_token
        def fetch_token():
            url = "https://aip.baidubce.com/oauth/2.0/token"
            params = {
                "grant_type": "client_credentials",
                "client_id": API_KEY,
                "client_secret": SECRET_KEY
            }
            return str(requests.post(url, params=params).json().get("access_token"))

        def cv2_to_base64(cv_img):
            _, buffer = cv2.imencode(".jpg", cv_img)
            return base64.b64encode(buffer).decode()

        def ocr_text(img_base64, token):
            url = "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic?access_token=" + token
            headers = {'content-type': 'application/x-www-form-urlencoded'}
            data = {"image": img_base64}
            response = requests.post(url, data=data, headers=headers)
            result_json = response.json()
            if "words_result" in result_json:
                return "".join([item["words"] for item in result_json["words_result"]])
            else:
                return None

        def ocr_qrcode(img_base64, token):
            url = "https://aip.baidubce.com/rest/2.0/ocr/v1/qrcode?access_token=" + token
            headers = {'content-type': 'application/x-www-form-urlencoded'}
            data = {"image": img_base64}
            response = requests.post(url, data=data, headers=headers)
            result_json = response.json()
            if "codes_result" in result_json:
                codes = []
                for code in result_json["codes_result"]:
                    text = code.get("text", "")
                    if isinstance(text, list):
                        text = " ".join(text)
                    codes.append(str(text))
                return "\n".join(codes)
            else:
                return None
        
        # 获取 token
        token = fetch_token()

        # qr_result = cv_qrcode(cv_image)

        # 待识别的图片
        img_base64 = cv2_to_base64(cv_image)

        # 先尝试识别二维码
        dir_result = None
        qr_result = ocr_qrcode(img_base64, token) ##API识别二维码
        QR1_cnt = QR1_cnt + 1 
        # end_QR1_time = time.time()
        print("识别总次数:", QR1_cnt)

        if qr_result == "A-1" or qr_result == "A-2" :
            print("【二维码识别结果】\n", qr_result)
            QR1 = qr_result
            dir_result = qr_result
        
        elif (QR1_cnt % 3 == 1):
            print("【未检测到二维码，尝试文字识别】")
            text_result = ocr_text(img_base64, token)
            text_state = 0
            normalized_string = ""

            if not text_result:
                print("\n未识别到任何有效文字。")
            else:
                #sorted_texts = sorted(text_result, key = lambda item: item['box'][0])
                #full_detected_string = "".join([item['text'] for item in sorted_texts])

                #更严格的清洗规则
                normalized_string = re.sub(r'[^AB12]', '', text_result.upper())
                print(f"严格清洗后 (只保留A,B,1,2): '{normalized_string}'")

                #映射逻辑
                # 检查是否同时包含 'A' 和 '1'
            if 'A' in normalized_string and '1' in normalized_string:
                text_state = 1
                QR1 = "A-1"
                dir_result = "A-1"
            # 检查是否同时包含 'A' 和 '2'
            elif 'A' in normalized_string and '2' in normalized_string:
                text_state = 2
                QR1 = "A-2"
                dir_result = "A-2"
            # 检查是否同时包含 'B' 和 '1'
            elif 'B' in normalized_string and '1' in normalized_string:
                text_state = 3
                QR1 = "B-1"
                dir_result = "B-1"
            # 检查是否同时包含 'B' 和 '2'
            elif 'B' in normalized_string and '2' in normalized_string:
                text_state = 4
                QR1 = "B-2"
                dir_result = "B-2"

            print("\n--- 最终状态 ---")
            if text_state == 0:
                print(f"未能匹配到目标状态 (清洗后结果: '{normalized_string}')。")
                dir_result = None
            else:
                # 【修改】打印信息更清晰
                print(f"成功匹配到模式 '{dir_result}' (来自: '{normalized_string}')，设置状态为: {text_state}")

            print("【文字识别结果】\n", dir_result)
        with self._lock:
            self._detected_flags['4'] = (dir_result is not None )
            print(f"A区库位方向检测标识位更新为: {self._detected_flags['4']}")

    def _detect_5(self, cv_image):
        print(5)
        # 预处理
        global QR1
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        '''mask = cv2.inRange(roi_hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)'''
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_x_start = width // 2
        roi_mask = np.zeros_like(mask_combined)
        if QR1 == 'A-1':
            roi_mask[roi_y_start:, roi_x_start:3 * width // 4] = 255
        else:
            roi_mask[roi_y_start:, width // 4:roi_x_start] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width // 4, 3 * width // 4):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['5'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['5'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['5']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect5_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1) 


    def _detect_6(self, cv_image):
        #print('d6')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = width // 4
        x_end = 3 * width // 4
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        print(str(yellow_ratio))
        #debuginfo
        with self._lock:
            current_time = time.time()
            print(str(yellow_ratio))
            if yellow_ratio > 0.09:
                self._detected_flags['6'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['6'] = True
                else:
                    self._detected_flags['6'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['6'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, height-roi_height1),           # 左上角
                    (x_end, height - roi_height2),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['14']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['14'] else (0, 255, 0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect6_{timestamp}_{yellow_ratio}.jpg", debug_img)

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
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_x_start =2 * width // 5
        roi_x_end = 3 * width // 5
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, roi_x_start:roi_x_end] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(2 * width // 5, 3 * width // 5):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['8'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['8'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['8']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect8_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_9(self, cv_image):
        # === 1. 预处理并转换为 HSV ===
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

       # 定义 ROI 位置：位于底部往上 1/4 高度的位置
        quarter_height = height // 6
        roi_center_y = height - quarter_height  # 距离底部 1/4 的位置（即整体的 3/4 处）

        # 定义 ROI 高度（例如 20 像素高）
        roi_height = height // 32

        # 计算上下边界
        y_start = roi_center_y - roi_height // 2
        y_end = roi_center_y + roi_height // 2

        # 确保不越界
        y_start = max(y_start, 0)
        y_end = min(y_end, height)

        # 横向：整幅宽度
        x_start = width // 5 * 2
        x_end = width //5 * 3 # 或直接用 width

        # 提取 ROI
        roi_hsv = hsv[y_start:y_end, x_start:x_end]

        # === 3. 使用模块函数生成“黄+泛白黄”掩码 ===
        mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            if yellow_ratio > 0.5:
                self._detected_flags['9'] = True
            else:
                self._detected_flags['9'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, y_start),           # 左上角
                    (x_end, y_end),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['9']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['9'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect9_{timestamp}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

        if self.real_debug_mode:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect9_{timestamp}.jpg"

            save_debug_image(filename, debug_img)

    def _detect_10(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        roi_height = height // 3
        roi_height1 = roi_height - 20
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        roi = hsv[height-roi_height:height-roi_height1, x_start:x_end]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)'''

        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height),
                    (x_end, height),
                    (0,255,0), 2)
        cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect10_{timestamp}_{yellow_ratio}.jpg", debug_img)

        with self._lock:
            current_time = time.time()
            if yellow_ratio < 0.1:
                if current_time - self._last_detection_time < 3:
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
            cv2.waitKey(1)

    def _detect_11(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # ===== 新增：只保留最大的两个连通域 =====
        # 1. 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        
        # 2. 创建新的干净掩码
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:  # 至少有一个非背景区域
            # 获取面积排序的索引（跳过背景0）
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1  # 降序排列
            
            # 只保留前两个最大区域
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        # ===== 新增部分结束 =====
        
        # 使用新的clean_mask替代原来的mask_combined
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/4区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # 边缘检测
        edges = cv2.Canny(masked_roi, 50, 150)
        
        # === 核心修改：只提取最下方边缘的点 ===
        # 1. 找到图像最底部的边缘点
        bottom_edge_points = []
        # 从底部向上扫描，寻找边缘点
        for y in range(height-1, roi_y_start-1, -1):  # 从底部向上扫描
            for x in range(width):
                if edges[y, x] != 0:
                    # 检查这个点是否是连续边缘的一部分
                    if len(bottom_edge_points) == 0:
                        bottom_edge_points.append([x, y])
                    else:
                        # 只添加与现有点相连或接近的点
                        last_x, last_y = bottom_edge_points[-1]
                        if abs(x - last_x) < 50:  # 水平距离阈值
                            bottom_edge_points.append([x, y])
            
            # 如果已经找到一条连续的边缘线，停止扫描
            if len(bottom_edge_points) > width//4:  # 找到足够长的边缘线
                break
        
        # 2. 如果没有找到底部边缘，尝试寻找主要边缘线
        if len(bottom_edge_points) < 10:
            # 使用连通组件分析找到主要边缘线
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges)
            
            # 找到最底部的连通组件
            max_bottom_y = 0
            target_label = -1
            for i in range(1, num_labels):  # 跳过背景标签0
                bottom_y = stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] - 1
                if bottom_y > max_bottom_y:
                    max_bottom_y = bottom_y
                    target_label = i
            
            # 提取目标连通组件的点
            if target_label != -1:
                y_coords, x_coords = np.where(labels == target_label)
                bottom_edge_points = [[x, y] for x, y in zip(x_coords, y_coords)]
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['11'] = False
            self._line_angle = 0.0
            
            if len(bottom_edge_points) >= 10:  # 确保有足够的点进行拟合
                # 使用最小二乘法拟合直线
                points_array = np.array(bottom_edge_points)
                vx, vy, cx, cy = cv2.fitLine(points_array, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                print(self._line_angle)
                self._detected_flags['11'] = True
                
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
                self._detected_flags['11'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            if self._detected_flags['11']:
                # 1. 绘制所有底部边缘点（绿色点）
                for point in bottom_edge_points:
                    cv2.circle(debug_img, tuple(point), 3, (0, 255, 0), -1)  # 绿色点表示底部边缘
                
                # 2. 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 3. 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(bottom_edge_points)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果点数不足，显示警告信息
            elif len(bottom_edge_points) > 0:
                cv2.putText(debug_img, 
                            f"WARNING: Only {len(bottom_edge_points)} points (min 10 needed)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            "WARNING: No bottom edge detected", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect11_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)


    def _detect_12(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 130
        roi_height2 = roi_height1 - 10
        x_start = 0 if QR1 == 'A-1' else 15 * width // 16
        x_end = x_start + width//16
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            current_time = time.time()
            print("yr" + str(yellow_ratio))
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['12'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['12'] = True
                else:
                    self._detected_flags['12'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['12'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, height-roi_height1),           # 左上角
                    (x_end, height - roi_height2),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['12']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['12'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect12_{timestamp}.jpg", debug_img)

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)


    def _detect_13(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, width // 5:3 * width // 5] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width // 5, 3 * width // 5):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['13'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['13'] = True
                
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
                self._detected_flags['13'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['13']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect13_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_14(self, cv_image):
        # print("14")
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # 定义 ROI 位置：位于底部往上 1/4 高度的位置
        quarter_height = height // 3
        roi_center_y = height - quarter_height  # 距离底部 1/4 的位置（即整体的 3/4 处）

        # 定义 ROI 高度（例如 20 像素高）
        roi_height = height // 32

        # 计算上下边界
        y_start = roi_center_y - roi_height // 2
        y_end = roi_center_y + roi_height // 2

        # 确保不越界
        y_start = max(y_start, 0)
        y_end = min(y_end, height)

        # 横向：整幅宽度
        x_start = 2 * width // 5
        x_end = 3 * width // 5  # 或直接用 width

        # 提取 ROI
        roi_hsv = hsv[y_start:y_end, x_start:x_end]

        mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)
        
        with self._lock:
            self._alignment_start_time = 0.0
            self._aligned_duration = 0.0
            if yellow_ratio > 0.5:
                self._detected_flags['14'] = True
            else:
                self._detected_flags['14'] = False 

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, y_start),           # 左上角
                    (x_end, y_end),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['14']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['14'] else (0, 255, 0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect14_{timestamp}_{yellow_ratio}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

        if self.real_debug_mode:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect14_{timestamp}.jpg"

            save_debug_image(filename, debug_img)

    def _detect_15(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 形态学处理去噪
        kernel = np.ones((5,5), np.uint8)
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找连通域
        contour_result = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
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
        # 预处理
        print(16)
        global arrow
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        if arrow == Direction.Left:
            roi_mask[roi_y_start:, width // 2:] = 255
        else:
            roi_mask[roi_y_start:, :width // 2] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            if arrow == Direction.Right:
                for x in range(0, width // 2):
                    for y in range(height - 2, roi_y_start - 1, -1):
                        if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                            boundary_points.append([x, y])
                            break  # 找到该列的第一个边界点即停止
            else:
                for x in range(width // 2, width):
                    for y in range(height - 2, roi_y_start - 1, -1):
                        if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                            boundary_points.append([x, y])
                            break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['16'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['16'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < 0.05:
                    if self._alignment_start_time == 0:
                        self._alignment_start_time = current_time
                    self._aligned_duration = current_time - self._alignment_start_time
                else:
                    self._alignment_start_time = 0.0
                    self._aligned_duration = 0.0
            else:
                self._alignment_start_time = 0.0
                self._aligned_duration = 0.0
                self._detected_flags['16'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['16']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect16_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_17(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 8
        roi_height2 = roi_height1 - 20
        x_start = 3 *width // 8
        x_end = 5 * width // 8
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio > 0.1:
                self._detected_flags['17'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['17'] = True
                else:
                    self._detected_flags['17'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['17'] = False
        '''debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, height-roi_height1),           # 左上角
                    (x_end, height - roi_height2),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['17']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['17'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        cv2.imwrite(f"detect17_{timestamp}.jpg", debug_img)'''

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
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
            info_text = f"Yaw: {np.degrees(self._yaw_adjust):+.1f}\u00B0"
            cv2.putText(debug_img, info_text, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_18_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        debug_img = cv_image.copy()
        
        # 颜色阈值定义
        YELLOW = (self.lower_yellow, self.upper_yellow)
        GRAY = (np.array([0, 0, 40]), np.array([180, 30, 90]))  # 深灰色范围
        
        # 扫描参数
        SCAN_START_Y = height // 2
        SCAN_STEP = 5              # 水平扫描步长
        WINDOW_SIZE = 20           # 颜色采样窗口大小
        
        def find_boundary(start_x, direction):
            """从中心向指定方向扫描寻找颜色边界"""
            boundary_x = None
            for x in range(start_x, width if direction==1 else 0, direction*SCAN_STEP):
                # 垂直采样窗口
                sample_area = hsv[SCAN_START_Y:SCAN_START_Y+WINDOW_SIZE, 
                                max(0,x-WINDOW_SIZE//2):min(width,x+WINDOW_SIZE//2)]
                
                # 计算颜色占比
                yellow_mask = cv2.inRange(sample_area, *YELLOW)
                gray_mask = cv2.inRange(sample_area, *GRAY)
                
                # 当黄色占比突增且灰色减少时判定为边界
                if cv2.countNonZero(yellow_mask) > 0.5*WINDOW_SIZE**2 and \
                cv2.countNonZero(gray_mask) < 0.2*WINDOW_SIZE**2:
                    boundary_x = x
                    break
            return boundary_x

        # 从中心向左右扫描
        center_x = width//2
        left_edge = find_boundary(center_x, -1)  # 向左扫描
        right_edge = find_boundary(center_x, 1)  # 向右扫描

        # 计算控制参数
        yaw_error = 0.0
        lateral_error = 0.0  # 新增横向位置误差
        
        if left_edge and right_edge:
            # 计算道路中心点
            target_center = (left_edge + right_edge) // 2
            
            # 角度误差（保持原有）
            center_offset = center_x - target_center
            yaw_error = np.arctan(center_offset / (width/2))
            
            # 新增横向位置误差（归一化到[-1,1]）
            lane_width = right_edge - left_edge
            lateral_error = (center_x - target_center) / (lane_width / 2)
            
            # 可视化道路中心线
            cv2.line(debug_img, (target_center, SCAN_START_Y-30),
                    (target_center, SCAN_START_Y+30), (255,0,0), 2)
        elif left_edge:
            yaw_error = np.radians(-15)
            lateral_error = -0.3  # 只有左边界时向右调整
        elif right_edge:
            yaw_error = np.radians(15)
            lateral_error = 0.3   # 只有右边界时向左调整
        
        # 可视化扫描线
        cv2.line(debug_img, (0, SCAN_START_Y), (width, SCAN_START_Y), (0,255,255), 2)
        if left_edge:
            cv2.line(debug_img, (left_edge, SCAN_START_Y-20),
                    (left_edge, SCAN_START_Y+20), (0,255,0), 3)
        if right_edge:
            cv2.line(debug_img, (right_edge, SCAN_START_Y-20),
                    (right_edge, SCAN_START_Y+20), (0,255,0), 3)
        
        # 保存误差值
        with self._lock:
            self._detected_flags['18_5'] = left_edge or right_edge
            self._yaw_adjust = np.clip(yaw_error * 0.3, -1.5, 1.5)
            self._lateral_error = np.clip(lateral_error, -1.0, 1.0)
        
        # 调试显示
        # if self.debug_mode:
        info_text = f"Yaw: {np.degrees(yaw_error):+.1f}\u00B0 Lat: {lateral_error:.2f}"
        cv2.putText(debug_img, info_text, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect185_{timestamp}.jpg", debug_img)        
        # cv2.imshow("Detection Preview", debug_img)
        # cv2.waitKey(1)

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
                cv2.putText(debug_img, f"Distance: {distance:.2f}m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_19_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        debug_img = cv_image.copy()
        
        # 颜色阈值定义
        YELLOW = (self.lower_yellow, self.upper_yellow)
        GRAY = (np.array([0, 0, 40]), np.array([180, 30, 90]))  # 深灰色范围
        
        # 扫描参数
        SCAN_START_Y = 3 * height // 5
        SCAN_STEP = 5              # 水平扫描步长
        WINDOW_SIZE = 20           # 颜色采样窗口大小
        
        def find_boundary(start_x, direction):
            """从中心向指定方向扫描寻找颜色边界"""
            boundary_x = None
            for x in range(start_x, width if direction==1 else 0, direction*SCAN_STEP):
                # 垂直采样窗口
                sample_area = hsv[SCAN_START_Y:SCAN_START_Y+WINDOW_SIZE, 
                                max(0,x-WINDOW_SIZE//2):min(width,x+WINDOW_SIZE//2)]
                
                # 计算颜色占比
                yellow_mask = cv2.inRange(sample_area, *YELLOW)
                gray_mask = cv2.inRange(sample_area, *GRAY)
                
                # 当黄色占比突增且灰色减少时判定为边界
                if cv2.countNonZero(yellow_mask) > 0.5*WINDOW_SIZE**2 and \
                cv2.countNonZero(gray_mask) < 0.2*WINDOW_SIZE**2:
                    boundary_x = x
                    break
            return boundary_x

        # 从中心向左右扫描
        center_x = width//2
        left_edge = find_boundary(center_x, -1)  # 向左扫描
        right_edge = find_boundary(center_x, 1)  # 向右扫描

        # 计算控制参数
        yaw_error = 0.0
        lateral_error = 0.0  # 新增横向位置误差
        
        if left_edge and right_edge:
            # 计算道路中心点
            target_center = (left_edge + right_edge) // 2
            
            # 角度误差（保持原有）
            center_offset = center_x - target_center
            yaw_error = np.arctan(center_offset / (width/2 + 1e-5))
            
            # 新增横向位置误差（归一化到[-1,1]）
            lane_width = right_edge - left_edge
            lateral_error = (center_x - target_center) / (lane_width / 2 + 1e-5)
            
            # 可视化道路中心线
            cv2.line(debug_img, (target_center, SCAN_START_Y-30),
                    (target_center, SCAN_START_Y+30), (255,0,0), 2)
        elif left_edge:
            yaw_error = np.radians(-15)
            lateral_error = 0.1  # 只有左边界时向右调整
        elif right_edge:
            yaw_error = np.radians(15)
            lateral_error = -0.1   # 只有右边界时向左调整
        
        # 可视化扫描线
        cv2.line(debug_img, (0, SCAN_START_Y), (width, SCAN_START_Y), (0,255,255), 2)
        if left_edge:
            cv2.line(debug_img, (left_edge, SCAN_START_Y-20),
                    (left_edge, SCAN_START_Y+20), (0,255,0), 3)
        if right_edge:
            cv2.line(debug_img, (right_edge, SCAN_START_Y-20),
                    (right_edge, SCAN_START_Y+20), (0,255,0), 3)
        
        # 保存误差值
        with self._lock:
            self._detected_flags['19_5'] = left_edge or right_edge
            self._yaw_adjust = np.clip(yaw_error * 0.3, -1.5, 1.5)
            self._lateral_error = np.clip(lateral_error, -1.0, 1.0)
        
        # 调试显示
        # if self.debug_mode:
        info_text = f"Yaw: {np.degrees(yaw_error):+.1f}\u00B0 Lat: {lateral_error:.2f}"
        cv2.putText(debug_img, info_text, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect195_{timestamp}.jpg", debug_img)  
        # cv2.imshow("Detection Preview", debug_img)
        # cv2.waitKey(1)

    def _detect_19_6(self, cv_image):
        print("进入检测19.6")

        global QR2
        global QR2_cnt
        if cv_image is None:
            print("QRCodeDetector Error: no frame")
            return

        # 保证兼容 python2 和 python3
        IS_PY3 = sys.version_info.major == 3
        if IS_PY3:
            from urllib.request import urlopen, Request
            from urllib.error import URLError
            from urllib.parse import urlencode
        else:
            from urllib2 import urlopen, Request, URLError
            from urllib import urlencode

        # 防止 https 证书校验不正确
        ssl._create_default_https_context = ssl._create_unverified_context

        # API Key 和 Secret Key
        API_KEY = 'ZyIjArFIzoRtgnT8WAUw5tCv'
        SECRET_KEY = '9JlkvkYt7r0C0GsJGcuRyoMhWaJx12yF'

        # 获取 access_token
        def fetch_token():
            url = "https://aip.baidubce.com/oauth/2.0/token"
            params = {
                "grant_type": "client_credentials",
                "client_id": API_KEY,
                "client_secret": SECRET_KEY
            }
            return str(requests.post(url, params=params).json().get("access_token"))

        def cv2_to_base64(cv_img):
            _, buffer = cv2.imencode(".jpg", cv_img)
            return base64.b64encode(buffer).decode()

        def ocr_text(img_base64, token):
            url = "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic?access_token=" + token
            headers = {'content-type': 'application/x-www-form-urlencoded'}
            data = {"image": img_base64}
            response = requests.post(url, data=data, headers=headers)
            result_json = response.json()
            if "words_result" in result_json:
                return "".join([item["words"] for item in result_json["words_result"]])
            else:
                return None

        def ocr_qrcode(img_base64, token):
            url = "https://aip.baidubce.com/rest/2.0/ocr/v1/qrcode?access_token=" + token
            headers = {'content-type': 'application/x-www-form-urlencoded'}
            data = {"image": img_base64}
            response = requests.post(url, data=data, headers=headers)
            result_json = response.json()
            if "codes_result" in result_json:
                codes = []
                for code in result_json["codes_result"]:
                    text = code.get("text", "")
                    if isinstance(text, list):
                        text = " ".join(text)
                    codes.append(str(text))
                return "\n".join(codes)
            else:
                return None
        
        
        # 获取 token
        token = fetch_token()

        # qr_result = cv_qrcode(cv_image)

        # 待识别的图片
        img_base64 = cv2_to_base64(cv_image)

        # 先尝试识别二维码
        dir_result = None
        qr_result = ocr_qrcode(img_base64, token) ##API识别二维码
        QR2_cnt = QR2_cnt + 1 
        # end_QR1_time = time.time()
        print("识别总次数:", QR2_cnt)

        if qr_result == "B-1" or qr_result == "B-2" :
            print("【二维码识别结果】\n", qr_result)
            QR2 = qr_result
            dir_result = qr_result
        
        elif (QR2_cnt % 3 == 1):
            print("【未检测到二维码，尝试文字识别】")
            text_result = ocr_text(img_base64, token)
            text_state = 0
            normalized_string = ""

            if not text_result:
                print("\n未识别到任何有效文字。")
            else:
                #sorted_texts = sorted(text_result, key = lambda item: item['box'][0])
                #full_detected_string = "".join([item['text'] for item in sorted_texts])

                #更严格的清洗规则
                normalized_string = re.sub(r'[^AB12]', '', text_result.upper())
                print(f"严格清洗后 (只保留A,B,1,2): '{normalized_string}'")

                #映射逻辑
                # 检查是否同时包含 'A' 和 '1'
            if 'A' in normalized_string and '1' in normalized_string:
                text_state = 1
                QR2 = "A-1"
                dir_result = "A-1"
            # 检查是否同时包含 'A' 和 '2'
            elif 'A' in normalized_string and '2' in normalized_string:
                text_state = 2
                QR2 = "A-2"
                dir_result = "A-2"
            # 检查是否同时包含 'B' 和 '1'
            elif 'B' in normalized_string and '1' in normalized_string:
                text_state = 3
                QR2 = "B-1"
                dir_result = "B-1"
            # 检查是否同时包含 'B' 和 '2'
            elif 'B' in normalized_string and '2' in normalized_string:
                text_state = 4
                QR2 = "B-2"
                dir_result = "B-2"

            print("\n--- 最终状态 ---")
            if text_state == 0:
                print(f"未能匹配到目标状态 (清洗后结果: '{normalized_string}')。")
                dir_result = None
            else:
                # 【修改】打印信息更清晰
                print(f"成功匹配到模式 '{dir_result}' (来自: '{normalized_string}')，设置状态为: {text_state}")

            print("【文字识别结果】\n", dir_result)
        
        with self._lock:
            self._detected_flags['19_6'] = (dir_result is not None)
            print(f"二维码检测标识位更新为: {self._detected_flags['19_6']}")

    def _detect_19_55(self, cv_image):
        # print(20)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 2
        roi_height2 = roi_height1 - 20
        x_start = 12 * width // 14 if arrow == Direction.Left else 1 * width // 14
        x_end = 13 * width // 14 if arrow == Direction.Left else 2 * width // 14
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (roi.size + 1e-5)
        with self._lock:
            current_time = time.time()
            if yellow_ratio > 0.1:
                self._detected_flags['19_55'] = True
                '''if current_time - self._last_detection_time < 0.1:
                    self._detected_flags['20'] = True
                else:
                    self._detected_flags['20'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['19_55'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['19_55']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['9'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect19_55_{timestamp}_{yellow_ratio}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_20(self, cv_image):
        # print(20)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        '''roi_height1 = height // 4
        roi_height2 = roi_height1 - 10
        x_start = 13 * width // 14 if arrow == Direction.Left else 0
        x_end = width if arrow == Direction.Left else width // 14'''
        roi_height1 = height // 2 
        roi_height2 = roi_height1 - 5
        x_start = 3 * width // 7
        x_end = 4 * width // 7
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''

        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        
        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['20'] = True
                '''if current_time - self._last_detection_time < 0.1:
                    self._detected_flags['20'] = True
                else:
                    self._detected_flags['20'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['20'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect20_{timestamp}_{yellow_ratio}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_21(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['21'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['21'] = True
                
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
                self._detected_flags['21'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['21']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect21_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_22(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = 0
        x_end = x_start + width - 1
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (roi.size + 1e-5)
        with self._lock:
            current_time = time.time()
            self._aligned_duration = 0.0
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['22'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['22'] = True
                else:
                    self._detected_flags['22'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['22'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect22_{timestamp}_{yellow_ratio}.jpg", debug_img)
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
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask_combined = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 使用配置模块提取黄色掩码
        '''mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)'''
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, 3 * width // 7:4 * width // 7] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)
        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(3 * width // 7, 4 * width // 7):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['23'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['23'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['23']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect23_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_24(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = width // 3
        x_end = 2 * width // 3
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            self._alignment_start_time = 0
            self._aligned_duration = 0.0
            if yellow_ratio > self._state9_threshold:
                '''if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['24'] = True
                else:
                    self._detected_flags['24'] = False  # 重置短暂检测'''
                self._detected_flags['24'] = True
                self._last_detection_time = current_time
            else:
                self._detected_flags['24'] = False
        
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect24_{timestamp}_{yellow_ratio}.jpg", debug_img)

        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_26(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 2 - 20
        roi_height2 = roi_height1 - 5
        x_start = 13 * width // 14
        x_end = width
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)
        with self._lock:
            current_time = time.time()
            self._alignment_start_time = 0
            self._aligned_duration = 0.0
            if yellow_ratio > 0.15:
                self._detected_flags['26'] = True
                '''if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['26'] = True
                else:
                    self._detected_flags['26'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['26'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect26_{timestamp}_{yellow_ratio}.jpg", debug_img)

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_27(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['27'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['27'] = True
                
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
                self._detected_flags['27'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['27']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect27_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_28(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 7
        roi_height2 = roi_height1 - 20
        x_start = width // 3
        x_end = 2 * width // 3
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            self._alignment_start_time = 0
            self._aligned_duration = 0.0
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['28'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['28'] = True
                else:
                    self._detected_flags['28'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['28'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect28_{timestamp}_{yellow_ratio}.jpg", debug_img)

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
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # !!!!!!!!!
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask_combined = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        

        # 使用配置模块提取黄色掩码
        '''mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)'''
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, 3 * width // 7:4 * width // 7] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 10:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(3 * width // 8, 5 * width // 8):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 6.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['29'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['29'] = True
                
                # 持续对齐计时逻辑
                if abs(self._line_angle) < 0.05:
                    if self._alignment_start_time == 0:
                        self._alignment_start_time = current_time
                    self._aligned_duration = current_time - self._alignment_start_time
                else:
                    self._alignment_start_time = 0.0
                    self._aligned_duration = 0.0
            else:
                self._alignment_start_time = 0.0
                self._aligned_duration = 0.0
                self._detected_flags['29'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['29']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect29_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_30(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 50
        roi_height2 = roi_height1 - 20
        x_start = width // 3
        x_end = 2 * width // 3
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            self._alignment_start_time = 0
            self._aligned_duration = 0.0
            if yellow_ratio > self._state9_threshold:
                self._detected_flags['30'] = True
                '''if current_time - self._last_detection_time < 0.3:
                    self._detected_flags['30'] = True
                else:
                    self._detected_flags['30'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['30'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect30_{timestamp}_{yellow_ratio}.jpg", debug_img)

        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)
    
    def _detect_32(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 2 - 20
        roi_height2 = roi_height1 - 5
        x_start = width // 20
        x_end = width // 12
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)
        with self._lock:
            current_time = time.time()
            if yellow_ratio > 0.15:
                '''if current_time - self._last_detection_time < 0.2:
                    self._detected_flags['32'] = True
                else:
                    self._detected_flags['32'] = False  # 重置短暂检测'''
                self._detected_flags['32'] = True
                self._last_detection_time = current_time
            else:
                self._detected_flags['32'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect32_{timestamp}_{yellow_ratio}.jpg", debug_img)

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height - roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    
    def _detect_33(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['33'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['33'] = True
                
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
                self._detected_flags['33'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['33']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect33_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_34(self, cv_image):
        # print(34)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 30
        roi_height2 = roi_height1 - 20
        x_start = width // 4
        x_end = 3 * width // 4
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            if yellow_ratio > 0.1:
                self._detected_flags['34'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['34'] = True
                else:
                    self._detected_flags['34'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['34'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect34_{timestamp}_{yellow_ratio}.jpg", debug_img)
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
        debug_img = cv_image.copy()
        
        # 颜色阈值定义
        YELLOW = (self.lower_yellow, self.upper_yellow)
        GRAY = (np.array([0, 0, 40]), np.array([180, 30, 90]))  # 深灰色范围
        
        # 扫描参数
        SCAN_START_Y = height // 2
        SCAN_STEP = 5              # 水平扫描步长
        WINDOW_SIZE = 20           # 颜色采样窗口大小
        
        def find_boundary(start_x, direction):
            """从中心向指定方向扫描寻找颜色边界"""
            boundary_x = None
            for x in range(start_x, width if direction==1 else 0, direction*SCAN_STEP):
                # 垂直采样窗口
                sample_area = hsv[SCAN_START_Y:SCAN_START_Y+WINDOW_SIZE, 
                                max(0,x-WINDOW_SIZE//2):min(width,x+WINDOW_SIZE//2)]
                
                # 计算颜色占比
                yellow_mask = cv2.inRange(sample_area, *YELLOW)
                gray_mask = cv2.inRange(sample_area, *GRAY)
                
                # 当黄色占比突增且灰色减少时判定为边界
                if cv2.countNonZero(yellow_mask) > 0.5*WINDOW_SIZE**2 and \
                cv2.countNonZero(gray_mask) < 0.2*WINDOW_SIZE**2:
                    boundary_x = x
                    break
            return boundary_x

        # 从中心向左右扫描
        center_x = width//2
        left_edge = find_boundary(center_x, -1)  # 向左扫描
        right_edge = find_boundary(center_x, 1)  # 向右扫描

        # 计算控制参数
        yaw_error = 0.0
        lateral_error = 0.0  # 新增横向位置误差
        
        if left_edge and right_edge:
            # 计算道路中心点
            target_center = (left_edge + right_edge) // 2
            
            # 角度误差（保持原有）
            center_offset = center_x - target_center
            yaw_error = np.arctan(center_offset / (width/2))
            
            # 新增横向位置误差（归一化到[-1,1]）
            lane_width = right_edge - left_edge
            lateral_error = (center_x - target_center) / (lane_width / 2)
            
            # 可视化道路中心线
            cv2.line(debug_img, (target_center, SCAN_START_Y-30),
                    (target_center, SCAN_START_Y+30), (255,0,0), 2)
        elif left_edge:
            yaw_error = np.radians(-15)
            lateral_error = -0.1  # 只有左边界时向右调整
        elif right_edge:
            yaw_error = np.radians(15)
            lateral_error = 0.1   # 只有右边界时向左调整
        
        # 可视化扫描线
        cv2.line(debug_img, (0, SCAN_START_Y), (width, SCAN_START_Y), (0,255,255), 2)
        if left_edge:
            cv2.line(debug_img, (left_edge, SCAN_START_Y-20),
                    (left_edge, SCAN_START_Y+20), (0,255,0), 3)
        if right_edge:
            cv2.line(debug_img, (right_edge, SCAN_START_Y-20),
                    (right_edge, SCAN_START_Y+20), (0,255,0), 3)
        
        # 保存误差值
        with self._lock:
            self._detected_flags['34_5'] = left_edge or right_edge
            self._yaw_adjust = np.clip(yaw_error * 0.3, -1.5, 1.5)
            self._lateral_error = np.clip(lateral_error, -1.0, 1.0)
        
        # 调试显示
        info_text = f"Yaw: {np.degrees(yaw_error):+.1f}\u00B0 Lat: {lateral_error:.2f}"
        cv2.putText(debug_img, info_text, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect345_{timestamp}.jpg", debug_img)    

    def _detect_35(self, cv_image):
        pass
        '''distance = self.YellowLightDetector.detect_distance(cv_image)
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
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)'''
    
    def _detect_35_5(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        debug_img = cv_image.copy()
        
        # 颜色阈值定义
        # YELLOW = YELLOW_COMBINED_KEYS
        YELLOW = (self.lower_yellow, self.upper_yellow)
        GRAY = (np.array([0, 0, 40]), np.array([180, 30, 90]))  # 深灰色范围
        
        # 扫描参数
        SCAN_START_Y = height // 2
        SCAN_STEP = 5              # 水平扫描步长
        WINDOW_SIZE = 20           # 颜色采样窗口大小
        
        def find_boundary(start_x, direction):
            """从中心向指定方向扫描寻找颜色边界"""
            boundary_x = None
            for x in range(start_x, width if direction==1 else 0, direction*SCAN_STEP):
                # 垂直采样窗口
                sample_area = hsv[SCAN_START_Y:SCAN_START_Y+WINDOW_SIZE, 
                                max(0,x-WINDOW_SIZE//2):min(width,x+WINDOW_SIZE//2)]
                
                # 计算颜色占比
                yellow_mask = cv2.inRange(sample_area, *YELLOW)
                gray_mask = cv2.inRange(sample_area, *GRAY)
                
                # 当黄色占比突增且灰色减少时判定为边界
                if cv2.countNonZero(yellow_mask) > 0.5*WINDOW_SIZE**2 and \
                cv2.countNonZero(gray_mask) < 0.2*WINDOW_SIZE**2:
                    boundary_x = x
                    break
            return boundary_x

        # 从中心向左右扫描
        center_x = width//2
        left_edge = find_boundary(center_x, -1)  # 向左扫描
        right_edge = find_boundary(center_x, 1)  # 向右扫描

        # 计算控制参数
        yaw_error = 0.0
        lateral_error = 0.0  # 新增横向位置误差
        
        if left_edge and right_edge:
            # 计算道路中心点
            target_center = (left_edge + right_edge) // 2
            
            # 角度误差（保持原有）
            center_offset = center_x - target_center
            yaw_error = np.arctan(center_offset / (width/2))
            
            # 新增横向位置误差（归一化到[-1,1]）
            lane_width = right_edge - left_edge
            lateral_error = (center_x - target_center) / (lane_width / 2)
            
            # 可视化道路中心线
            cv2.line(debug_img, (target_center, SCAN_START_Y-30),
                    (target_center, SCAN_START_Y+30), (255,0,0), 2)
        elif left_edge:
            yaw_error = np.radians(-15)
            lateral_error = -0.1  # 只有左边界时向右调整
        elif right_edge:
            yaw_error = np.radians(15)
            lateral_error = 0.1   # 只有右边界时向左调整
        
        # 可视化扫描线
        cv2.line(debug_img, (0, SCAN_START_Y), (width, SCAN_START_Y), (0,255,255), 2)
        if left_edge:
            cv2.line(debug_img, (left_edge, SCAN_START_Y-20),
                    (left_edge, SCAN_START_Y+20), (0,255,0), 3)
        if right_edge:
            cv2.line(debug_img, (right_edge, SCAN_START_Y-20),
                    (right_edge, SCAN_START_Y+20), (0,255,0), 3)
        
        # 保存误差值
        with self._lock:
            self._detected_flags['35_5'] = left_edge or right_edge
            self._yaw_adjust = np.clip(yaw_error * 0.3, -1.5, 1.5)
            self._lateral_error = np.clip(lateral_error, -1.0, 1.0)
        
        # 调试显示
        info_text = f"Yaw: {np.degrees(yaw_error):+.1f}\u00B0 Lat: {lateral_error:.2f}"
        cv2.putText(debug_img, info_text, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect355_{timestamp}.jpg", debug_img)  

    def _detect_36(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_slope, self.upper_slope)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        height, width = cv_image.shape[:2]
        image_center = (width // 2, height // 2)

        closest_contour = None
        min_distance_sq = float('inf')
        slope_top_y = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_slope_area:
                continue
                
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            dx = cX - image_center[0]
            dy = cY - image_center[1]
            distance_sq = dx**2 + dy**2
            
            if distance_sq < min_distance_sq:
                min_distance_sq = distance_sq
                closest_contour = cnt

        # 处理最优轮廓
        if closest_contour is not None:
            top_point = tuple(closest_contour[closest_contour[:,:,1].argmin()][0])
            slope_top_y = top_point[1]
            
            if self.debug_mode:
                debug_img = cv_image.copy()
                cv2.drawContours(debug_img, [closest_contour], -1, (0,255,0), 2)
                M = cv2.moments(closest_contour)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                cv2.circle(debug_img, (cX, cY), 5, (0,0,255), -1)
                cv2.circle(debug_img, image_center, 5, (255,0,0), -1)
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
            
            if slope_top_y <= image_center[1]:
                self._detected_flags['36'] = True
            else:
                self._detected_flags['36'] = False
        else:
            self._detected_flags['36'] = False

        with self._lock:
            self.slope_top_y = slope_top_y
    
    def _detect_36_5(self, cv_image):
        # 预处理
        global arrow
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        if arrow == Direction.Left:
            roi_mask[roi_y_start:, width // 2:] = 255
        else:
            roi_mask[roi_y_start:, :width // 2] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['36_5'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['36_5'] = True
                
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
                self._detected_flags['36_5'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['36_5']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect365_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_37(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = width // 3
        x_end = width - x_start
        roi = hsv[height-roi_height1:height - roi_height2, x_start:width]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio < self._state10_threshold:
                self._detected_flags['37'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['37'] = True
                else:
                    self._detected_flags['37'] = False
                self._last_detection_time = current_time'''
            else:
                self._detected_flags['37'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect37_{timestamp}_{yellow_ratio}.jpg", debug_img)
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
            cv2.waitKey(1)

    def _detect_38(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(5):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['38'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['38'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['38']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect38_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_39(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = height // 2 + height // 10
        roi_height2 = roi_height1 - 5
        '''x_start = width // 5 if arrow == Direction.Right else 4 * width // 5
        x_end = 2 * width // 5 if arrow == Direction.Right else width'''
        x_start = 3 * width // 7
        x_end = 4 * width // 7
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        
        with self._lock:
            current_time = time.time()
            if yellow_ratio > 0.05:
                if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['39'] = True
                else:
                    self._detected_flags['39'] = False
                self._last_detection_time = current_time
            else:
                self._detected_flags['39'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height1),
                    (x_end, height - roi_height2),
                    (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect39_{timestamp}_{yellow_ratio}.jpg", debug_img)

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
            cv2.waitKey(1)
    
    def _detect_39_55(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        debug_img = cv_image.copy()
        
        # 颜色阈值定义
        YELLOW = (self.lower_yellow, self.upper_yellow)
        GRAY = (np.array([0, 0, 40]), np.array([180, 30, 90]))  # 深灰色范围
        
        # 扫描参数
        SCAN_START_Y = height // 2 - height // 8
        SCAN_STEP = 5              # 水平扫描步长
        WINDOW_SIZE = 20           # 颜色采样窗口大小
        
        def find_boundary(start_x, direction):
            """从中心向指定方向扫描寻找颜色边界"""
            boundary_x = None
            for x in range(start_x, width if direction==1 else 0, direction*SCAN_STEP):
                # 垂直采样窗口
                sample_area = hsv[SCAN_START_Y:SCAN_START_Y+WINDOW_SIZE, 
                                max(0,x-WINDOW_SIZE//2):min(width,x+WINDOW_SIZE//2)]
                
                # 计算颜色占比
                yellow_mask = cv2.inRange(sample_area, *YELLOW)
                gray_mask = cv2.inRange(sample_area, *GRAY)
                
                # 当黄色占比突增且灰色减少时判定为边界
                if cv2.countNonZero(yellow_mask) > 0.5*WINDOW_SIZE**2 and \
                cv2.countNonZero(gray_mask) < 0.2*WINDOW_SIZE**2:
                    boundary_x = x
                    break
            return boundary_x

        # 从中心向左右扫描
        center_x = width//2
        left_edge = find_boundary(center_x, -1)  # 向左扫描
        right_edge = find_boundary(center_x, 1)  # 向右扫描

        # 计算控制参数
        yaw_error = 0.0
        lateral_error = 0.0  # 新增横向位置误差
        
        if left_edge and right_edge:
            # 计算道路中心点
            target_center = (left_edge + right_edge) // 2
            
            # 角度误差（保持原有）
            center_offset = center_x - target_center
            yaw_error = np.arctan(center_offset / (width/2))
            
            # 新增横向位置误差（归一化到[-1,1]）
            lane_width = right_edge - left_edge
            lateral_error = (center_x - target_center) / (lane_width / 2)
            
            # 可视化道路中心线
            cv2.line(debug_img, (target_center, SCAN_START_Y-30),
                    (target_center, SCAN_START_Y+30), (255,0,0), 2)
        elif left_edge:
            yaw_error = np.radians(-15)
            lateral_error = -0.1  # 只有左边界时向右调整
        elif right_edge:
            yaw_error = np.radians(15)
            lateral_error = 0.1   # 只有右边界时向左调整
        
        # 可视化扫描线
        cv2.line(debug_img, (0, SCAN_START_Y), (width, SCAN_START_Y), (0,255,255), 2)
        if left_edge:
            cv2.line(debug_img, (left_edge, SCAN_START_Y-20),
                    (left_edge, SCAN_START_Y+20), (0,255,0), 3)
        if right_edge:
            cv2.line(debug_img, (right_edge, SCAN_START_Y-20),
                    (right_edge, SCAN_START_Y+20), (0,255,0), 3)
        
        # 保存误差值
        with self._lock:
            self._detected_flags['39_55'] = left_edge or right_edge
            self._yaw_adjust = np.clip(yaw_error * 0.3, -1.5, 1.5)
            self._lateral_error = np.clip(lateral_error, -1.0, 1.0)
        
        # 调试显示
        info_text = f"Yaw: {np.degrees(yaw_error):+.1f}\u00B0 Lat: {lateral_error:.2f}"
        cv2.putText(debug_img, info_text, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect3955_{timestamp}.jpg", debug_img)  

    def _detect_40(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_x_start =2 * width // 5
        roi_x_end = 3 * width // 5
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, roi_x_start:roi_x_end] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(2 * width // 5, 3 * width // 5):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['40'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['40'] = True
                
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
                self._detected_flags['40'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['40']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect40_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)
    
    def _detect_41(self, cv_image):
        # === 1. 预处理并转换为 HSV ===
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

       # 定义 ROI 位置：位于底部往上 1/4 高度的位置
        quarter_height = height // 6
        roi_center_y = height - quarter_height  # 距离底部 1/4 的位置（即整体的 3/4 处）

        # 定义 ROI 高度（例如 20 像素高）
        roi_height = height // 32

        # 计算上下边界
        y_start = roi_center_y - roi_height // 2
        y_end = roi_center_y + roi_height // 2

        # 确保不越界
        y_start = max(y_start, 0)
        y_end = min(y_end, height)

        # 横向：整幅宽度
        x_start = width // 5 * 2
        x_end = width //5 * 3 # 或直接用 width

        # 提取 ROI
        roi_hsv = hsv[y_start:y_end, x_start:x_end]

        # === 3. 使用模块函数生成“黄+泛白黄”掩码 ===
        mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            if yellow_ratio > 0.5:
                self._detected_flags['41'] = True
            else:
                self._detected_flags['41'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, y_start),           # 左上角
                    (x_end, y_end),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['9']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['9'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect41_{timestamp}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

        if self.real_debug_mode:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect41_{timestamp}.jpg"

            save_debug_image(filename, debug_img)
    
    def _detect_42(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_x_start = width // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, :width // 2] = 255
        if QR1 == 'A-2':
            roi_mask[roi_y_start:, roi_x_start:3 * width // 4] = 255
        else:
            roi_mask[roi_y_start:, width // 4:roi_x_start] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width // 4, 3 * width // 4):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['42'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['42'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['42']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect42_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_43(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 20
        roi_height2 = roi_height1 - 20
        x_start = width // 4
        x_end = 3 * width // 4
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零
        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['43'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['43'] = True
                else:
                    self._detected_flags['43'] = False  # 重置短暂检测'''
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
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_x_start =2 * width // 5
        roi_x_end = 3 * width // 5
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, roi_x_start:roi_x_end] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(2 * width // 5, 3 * width // 5):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(2):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['45'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
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
                self._detected_flags['45'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['45']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect45_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_46(self, cv_image):
        # === 1. 预处理并转换为 HSV ===
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

       # 定义 ROI 位置：位于底部往上 1/4 高度的位置
        quarter_height = height // 6
        roi_center_y = height - quarter_height  # 距离底部 1/4 的位置（即整体的 3/4 处）

        # 定义 ROI 高度（例如 20 像素高）
        roi_height = height // 32

        # 计算上下边界
        y_start = roi_center_y - roi_height // 2
        y_end = roi_center_y + roi_height // 2

        # 确保不越界
        y_start = max(y_start, 0)
        y_end = min(y_end, height)

        # 横向：整幅宽度
        x_start = width // 5 * 2
        x_end = width //5 * 3 # 或直接用 width

        # 提取 ROI
        roi_hsv = hsv[y_start:y_end, x_start:x_end]

        # === 3. 使用模块函数生成“黄+泛白黄”掩码 ===
        mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            if yellow_ratio > 0.5:
                self._detected_flags['46'] = True
            else:
                self._detected_flags['46'] = False

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, y_start),           # 左上角
                    (x_end, y_end),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['9']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['9'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect46_{timestamp}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

        if self.real_debug_mode:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect46_{timestamp}.jpg"

            save_debug_image(filename, debug_img)

    def _detect_47(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        
        roi_height = height // 5
        roi_height1 = roi_height - 20
        x_start = 2 * width // 5
        x_end = 3 * width // 5
        roi = hsv[height-roi_height:height-roi_height1, x_start:x_end]
        
        mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img, 
                    (x_start, height-roi_height),
                    (x_end, height),
                    (0,255,0), 2)
        cv2.putText(debug_img, f"Y10: {yellow_ratio:.2f}", (x_start+10, height-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect47_{timestamp}_{yellow_ratio}.jpg", debug_img)

        with self._lock:
            current_time = time.time()
            if yellow_ratio < 0.01:
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
            cv2.waitKey(1)

    def _detect_49(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        roi_height1 = 130
        roi_height2 = roi_height1 - 10
        x_start = 0 if QR1 == 'A-2' else 15 * width // 16
        x_end = x_start + width//16
        roi = hsv[height-roi_height1:height - roi_height2, x_start:x_end]
        
        '''mask = cv2.inRange(roi, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        yellow_ratio = np.count_nonzero(mask) / (roi.size + 1e-5)  # 防止除零'''
        
        mask_combined = get_combined_color_mask(roi, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)

        with self._lock:
            current_time = time.time()
            if yellow_ratio > self.detection_threshold:
                self._detected_flags['49'] = True
                '''if current_time - self._last_detection_time < 0.5:
                    self._detected_flags['49'] = True
                else:
                    self._detected_flags['49'] = False  # 重置短暂检测'''
                self._last_detection_time = current_time
            else:
                self._detected_flags['49'] = False
        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, height-roi_height1),           # 左上角
                    (x_end, height - roi_height2),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['49']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['9'] else (0, 255, 0), 2)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
        save_debug_image(f"detect49_{timestamp}.jpg", debug_img)

        # 调试显示
        if self.debug_mode:
            debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            cv2.rectangle(debug_img, 
                        (x_start, height-roi_height1),
                        (x_end, height-roi_height2),
                        (0,255,0), 2)
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

    def _detect_50(self, cv_image):
        # 预处理
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 使用配置模块提取黄色掩码
        mask_combined = get_combined_color_mask(hsv, YELLOW_COMBINED_KEYS)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)
        
        # 只保留最大的两个连通域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_combined, connectivity=8)
        clean_mask = np.zeros_like(mask_combined)
        if num_labels > 1:
            areas = stats[1:, cv2.CC_STAT_AREA]
            sorted_idx = np.argsort(areas)[::-1] + 1
            for i in sorted_idx[:min(2, len(sorted_idx))]:
                clean_mask[labels == i] = 255
        mask_combined = clean_mask

        # 创建ROI聚焦图像底部 (高度1/2区域)
        height, width = mask_combined.shape[:2]
        roi_y_start = height - height // 2
        roi_mask = np.zeros_like(mask_combined)
        roi_mask[roi_y_start:, width // 5:3 * width // 5] = 255
        masked_roi = cv2.bitwise_and(mask_combined, roi_mask)

        # === 专业方法：检测黄色边线的实际边界 ===
        # 1. 找到黄色区域的轮廓
        contour_result = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:  # OpenCV 3.x
            contours = contour_result[1]
        elif len(contour_result) == 2:  # OpenCV 4.x
            contours = contour_result[0]
        else:
            contours = contour_result
        # 2. 提取所有边界点
        boundary_points = []
        
        # 3. 对每个轮廓，提取其底部边界点
        for contour in contours:
            # 创建轮廓的边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 只处理ROI区域内的轮廓
            if y + h < roi_y_start:
                continue
            
            # 创建轮廓的掩码
            contour_mask = np.zeros_like(mask_combined)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            
            # 对轮廓进行多边形近似
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 提取轮廓的底部边缘点
            for point in approx.squeeze():
                px, py = point
                # 只考虑ROI区域内的点
                if py < roi_y_start:
                    continue
                
                # 检查是否是底部点：该点下方不是黄色区域
                if py < height - 1:
                    if mask_combined[py + 1, px] == 0:  # 下方像素是背景
                        boundary_points.append([px, py])
        
        # 4. 如果没有足够边界点，尝试备用方法
        if len(boundary_points) < 20:
            # 备用方法：逐列扫描底部边缘
            boundary_points = []
            for x in range(width // 5, 3 * width // 5):
                for y in range(height - 2, roi_y_start - 1, -1):
                    if mask_combined[y, x] == 255 and mask_combined[y + 1, x] == 0:
                        boundary_points.append([x, y])
                        break  # 找到该列的第一个边界点即停止
        
        # 5. 使用RANSAC算法进行稳健直线拟合
        best_line = None
        best_inliers = []
        max_inliers = 0
        
        if len(boundary_points) >= 10:
            points_array = np.array(boundary_points)
            
            # 使用RANSAC多次迭代寻找最佳拟合线
            for _ in range(5):
                # 随机选择两个点
                idx = np.random.choice(len(points_array), 2, replace=False)
                p1, p2 = points_array[idx[0]], points_array[idx[1]]
                
                # 计算直线参数
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx == 0:  # 避免除以零
                    continue
                    
                # 直线方程: y = mx + c
                m = dy / dx
                c = p1[1] - m * p1[0]
                
                # 计算所有点到直线的距离
                distances = []
                for point in points_array:
                    px, py = point
                    # 点到直线的距离公式
                    dist = abs(m * px - py + c) / np.sqrt(m**2 + 1)
                    distances.append(dist)
                
                # 统计内点（距离小于阈值的点）
                inlier_threshold = 2.0  # 像素距离阈值
                inliers = [i for i, dist in enumerate(distances) if dist < inlier_threshold]
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_inliers = inliers
                    best_line = (m, c, p1, p2)
        
        current_time = time.time()
        with self._lock:
            self._detected_flags['50'] = False
            self._line_angle = 0.0
            
            if best_line is not None and len(best_inliers) >= 10:
                m, c, p1, p2 = best_line
                
                # 使用内点重新拟合直线（更精确）
                inlier_points = points_array[best_inliers]
                vx, vy, cx, cy = cv2.fitLine(inlier_points, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 计算直线角度（相对于水平轴）
                line_angle = np.arctan2(vy, vx)[0]
                angle_deg = np.degrees(line_angle)
                target_angle = 0
                angle_diff = angle_deg - target_angle
                
                self._line_angle = np.radians(angle_diff)
                self._detected_flags['50'] = True
                
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
                self._detected_flags['50'] = False

            # 调试图像处理
            debug_img = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_y_start), (width, height), (255, 0, 0), 2)  # 绘制ROI区域
            
            # 绘制所有边界点
            for point in boundary_points:
                cv2.circle(debug_img, tuple(point), 2, (0, 255, 0), -1)  # 绿色点表示边界点
            
            # 如果检测到直线，绘制最佳拟合线和内点
            if self._detected_flags['50']:
                # 绘制内点（蓝色）
                for idx in best_inliers:
                    point = boundary_points[idx]
                    cv2.circle(debug_img, tuple(point), 3, (255, 0, 0), -1)  # 蓝色点表示内点
                
                # 绘制拟合直线（红色粗线）
                length = 500
                pt1 = (int(cx - vx * length), int(cy - vy * length))
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.line(debug_img, pt1, pt2, (0, 0, 255), 3)
                
                # 显示角度信息
                cv2.putText(debug_img, 
                            f"Points: {len(boundary_points)} | Inliers: {len(best_inliers)} | Angle: {angle_diff:+.1f}\u00B0 | Hold: {self._aligned_duration:.1f}s", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 如果没有检测到合适的直线
            elif len(boundary_points) >= 10:
                cv2.putText(debug_img, 
                            f"WARNING: No suitable line found ({len(boundary_points)} boundary points)", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(debug_img, 
                            f"WARNING: Not enough boundary points ({len(boundary_points)})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制所有轮廓
            cv2.drawContours(debug_img, contours, -1, (255, 255, 0), 2)  # 青色轮廓
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_debug_image(f"detect50_{timestamp}.jpg", debug_img)
            
            # 实时调试显示
            if self.debug_mode:
                cv2.imshow("Detection Preview", debug_img)
                cv2.waitKey(1)

    def _detect_51(self, cv_image):
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # 定义 ROI 位置：位于底部往上 1/4 高度的位置
        quarter_height = height // 3
        roi_center_y = height - quarter_height  # 距离底部 1/4 的位置（即整体的 3/4 处）

        # 定义 ROI 高度（例如 20 像素高）
        roi_height = height // 32

        # 计算上下边界
        y_start = roi_center_y - roi_height // 2
        y_end = roi_center_y + roi_height // 2

        # 确保不越界
        y_start = max(y_start, 0)
        y_end = min(y_end, height)

        # 横向：整幅宽度
        x_start = 2 * width // 5
        x_end = 3 * width // 5  # 或直接用 width

        # 提取 ROI
        roi_hsv = hsv[y_start:y_end, x_start:x_end]

        mask_combined = get_combined_color_mask(roi_hsv, YELLOW_COMBINED_KEYS)

        # === 4. 形态学去噪 ===
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, MORPH_KERNEL)

        # === 5. 黄色像素占比计算 ===
        yellow_ratio = np.count_nonzero(mask_combined) / (mask_combined.size + 1e-5)
        
        with self._lock:
            if yellow_ratio > 0.5:
                self._detected_flags['51'] = True
            else:
                self._detected_flags['51'] = False 

        debug_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(debug_img,
                    (x_start, y_start),           # 左上角
                    (x_end, y_end),             # 右下角
                    (0, 255, 0), 2)          # 绿色框，线宽 2
        cv2.putText(debug_img, f"Yellow Ratio: {yellow_ratio:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Detected: {self._detected_flags['14']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if not self._detected_flags['14'] else (0, 255, 0), 2)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_debug_image(f"detect51_{timestamp}_{yellow_ratio}.jpg", debug_img)
        # 调试显示
        if self.debug_mode:
            cv2.imshow("Detection Preview", debug_img)
            cv2.waitKey(1)

        if self.real_debug_mode:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 精确到微秒
            filename = f"detect51_{timestamp}.jpg"

            save_debug_image(filename, debug_img)
    
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
    def lateral_error(self):
        with self._lock:
            return self._lateral_error

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
    def _17_detected(self):
        with self._lock:
            return self._detected_flags['17']

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
    def _19_55_detected(self):
        with self._lock:
            return self._detected_flags['19_55']

    @property
    def _19_6_detected(self):
        with self._lock:
            return self._detected_flags['19_6']
        
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
    def _36_5_detected(self):
        with self._lock:
            return self._detected_flags['36_5']
    
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
    def _39_55_detected(self):
        with self._lock:
            return self._detected_flags['39_55']

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


def loadtoml(file):
    try:
        steps = toml.load(file)
        for step in steps['step']:
            msg.mode = step['mode']
            msg.value = step['value']
            msg.contact = step['contact']
            msg.gait_id = step['gait_id']
            msg.duration = step['duration']
            msg.life_count = (msg.life_count + 1) % 128
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

def main_detection_loop(depth_node, ctrl, msg, trigger_distance, detection_type):
    """主检测循环"""
    print(f"\n===== 开始{detection_type}检测 =====")
    
    # 确保节点正在运行
    depth_node.set_node_running(True)
    
    # 控制机器人直行
    msg.mode = 11
    msg.gait_id = 3
    msg.vel_des = [0.2, -0.005, 0]
    # msg.step_height = [0.04, 0.04]
    msg.life_count =(msg.life_count + 1) %128

    while rclpy.ok():
        detected, distance = depth_node.get_step_info()

        if detected and distance < trigger_distance:
            print(f"**检测到{detection_type}!** 距离: {distance:.2f}米。")
            # 检测到目标后暂停节点处理
            depth_node.set_node_running(False)
            return True
        else:
            status = f"距离 {distance:.2f}m" if detected else "无"
            print(f"前方{detection_type}: {status}。")

        ctrl.Send_cmd(msg)
        time.sleep(0.1)
    
    return False

def standup():  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count = (msg.life_count + 1) % 128 # Command will take effect when life_count update
    Ctrl.Send_cmd(msg)
    time.sleep(1)
    # Ctrl.Wait_finish(12, 0, 10)

def stone_road(duration):  # 石板路
    msg.mode = 62
    msg.gait_id = 81
    msg.life_count = (msg.life_count + 1) % 128
    msg.duration = 0
    Ctrl.Send_cmd(msg)
    time.sleep(duration)


def limit(ctrl, msg, duration=9):
    """限高杆动作"""
    msg.mode = 62
    msg.gait_id = 80
    msg.life_count = (msg.life_count + 1) % 128
    msg.duration = 0
    ctrl.Send_cmd(msg)
    ctrl.Wait_finish(62, 80, duration)


def upslope(step = 14):  
    msg.mode = 62
    msg.gait_id = 92  
    msg.life_count = (msg.life_count + 1) % 128
    msg.duration = 0
    Ctrl.Send_cmd(msg)
    # time.sleep(2.4 * step - 1.2 + 0.001)
    time.sleep(2.4 * step - 1.8 + 0.001)
    # Ctrl.Wait_finish(62, 92, 2.4 * step - 1.2 + 0.001)

def downslope():  
    msg.mode = 62
    msg.gait_id = 93  
    msg.life_count = (msg.life_count + 1) % 128
    msg.duration = 0
    Ctrl.Send_cmd(msg)
    time.sleep(40)


def stand():
    msg.mode = 112  # Recovery stand
    msg.gait_id = 1
    msg.life_count += 1  # Command will take effect when life_count update
    msg.step_height = [0.03, 0.03, 0.03, 0.03]
    msg.rpy_des = [0.0, 0.0, 0.0]
    msg.duration = 0

    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(112, 1, 2)


def new_move(left=None,acc=None):
    if left != None:
        if left == True:
            rgb_camera.left = True
            processor.left = True
        else:
            rgb_camera.left = False
            processor.left = False

    angle_0 = 0.0
    Kp = 6.0
    Kd = 0.8
    max_yaw = 30
    if(acc == None): acc = 5
    #print("start_align")
    while(1):
        frame = rgb_camera.get_new_frame(left)
        # if frame == None:
        #     print('yjm')
        if(processor.main_process(frame) == False):
            continue
        
        current_angle = processor.angle
        if current_angle == None:
            continue
        if abs(current_angle) < acc:
            break
        delta_angle = current_angle - angle_0
        angle_0 = current_angle
        yaw_speed = np.clip(current_angle * Kp + delta_angle * Kd,-max_yaw,max_yaw)
        msg.mode=11
        msg.gait_id=27
        yaw_speed = yaw_speed/180*3.14
        msg.vel_des=[0.03,0.0,-yaw_speed]
        msg.life_count=(msg.life_count+1)%128    
        Ctrl.Send_cmd(msg)
        time.sleep(0.5)
    #print("aligh_over")

def move(flag,adjust):
    vel = 0.06
    if(flag == 0):
        if adjust == adjustment.faraway:
            vel = - vel
        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.03,vel,0]
        msg.life_count=(msg.life_count+1)%128    
        Ctrl.Send_cmd(msg)
        time.sleep(0.6)
    else :
        if adjust == adjustment.near:
            vel = - vel
        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.03,vel,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.6)
    print(f"vel:{vel}")
    
def go_center(left):
    if left == True:
        rgb_camera.left = True
        processor.left = True
    else:
        rgb_camera.left = False
        processor.left = False

    prev_error = 0.0
    Kp = 0.05
    Kd = 0.1
    max_vel = 0.1
    if left == True:
        sign = 1
    else:
        sign = -1
    #print("start_center")
    while(1):
        frame = rgb_camera.get_new_frame(left)
        if(processor.main_process(frame) == False):
            continue
        
        error = processor.center_y - processor.reference_y
        if abs(error) < processor.reference_y * processor.tolerance_ratio * 0.8:
            break

        derivative = error - prev_error
        vel = -(Kp * error + Kd * derivative)
        vel = max(min(vel, max_vel), -max_vel)

        prev_error = error

        msg.mode=11
        msg.gait_id=27
        vel = vel * sign
        print(vel)
        msg.vel_des=[0.05,vel,0]
        msg.life_count=(msg.life_count+1)%128    
        Ctrl.Send_cmd(msg)
        time.sleep(0.3)
    #print("center_over")

def process(rotate_speed):
    msg.mode=11
    msg.gait_id=27
    msg.step_height = [0.03,0.03]
    msg.vel_des=[0.23*1/2,0,rotate_speed*1/2]
    msg.rpy_des=[0,-0.45,0]
    msg.duration = 0
    msg.life_count=(msg.life_count+1)%128
    Ctrl.Send_cmd(msg)

purple_detector = PurpleDetector()
arrow_detector = ArrowDetector()
processor = s_curve_processor()
order = 1

#flag for left or right 0 for left 1 for right
def detecte(flag,index,flag_change):   
    if(flag == 0): #left camera
        rgb_camera.left = True
        processor.left = True
    else:
        rgb_camera.left = False
        processor.left = False
    chang_count = 0
    start_change_count = 0
    count = 0
    while(1):
        start_change_count += 1
        if flag == 0:
            frame = rgb_camera.get_new_frame(True) #move
        else: 
            frame = rgb_camera.get_new_frame(False)
        
        if flag_change == 0:
            frame_change =  rgb_camera.get_new_frame(True) # change
        else:
            frame_change =  rgb_camera.get_new_frame(False)

        if(processor.main_process(frame) == False):
            print("no result")
            continue
        #print(processor.adjust!=adjustment.mid)
        print(f"adjust: {processor.adjust}")
        if (processor.adjust!=adjustment.mid):
            # print(adjustment.mid)
            print(f"adjust: {processor.adjust}")           
            move(flag,processor.adjust)
            if flag == 0 :
                new_move(True,7)
            elif flag == 1:
                new_move(False,7)
            print("move over") 
            processor.adjust = adjustment.mid
            process(processor.rotate_speed)
 
        if  (order == 1 and ((start_change_count >=230 and index==2) or (start_change_count >=205 and index==3) ) ) or (start_change_count >=205 and index < 4 and index > 1 and order == 2) or (start_change_count >=20 and index == 1) :
            # print('ssss')
            if processor.fit_yellow_curve(frame_change,flag_change):
                a,b,c = processor.coeffs
                #a  b 阈值
                if chang_count >=5:
                    print("change!!!!!!!!!!!!!!!!!!")
                    break
                print(f"a, b, c = {a},{b},{c} ")
                if abs(a) < 0.001:
                    chang_count = chang_count + 1
                    print(f"chang_count:{chang_count}")


        if index == 4 and order == 1:
            print(f"order:{order}")
            frame_real_rgb = real_rgb_camera.get_new_frame()
            #print("have gotten rgb fram")
            direction = arrow_detector.detect_arrow(frame_real_rgb)
            print(direction)
            if direction!= Direction.UNKNOWN:
                arrow_detector.direction_count += 1
                print(f"direction_count:{arrow_detector.direction_count}")
            
            if arrow_detector.direction_count > 6: #before 6
                arrow_detector.direction_count = 0
                #new
                print("find and move")

                angle = 12
                w = angle*3.14/180
                msg.mode=11
                msg.gait_id=27
                msg.vel_des=[0.03,0.0,-w]
                msg.life_count=(msg.life_count+1)%128    
                Ctrl.Send_cmd(msg)
                time.sleep(2.3)
            
                msg.mode=11
                msg.gait_id=27
                msg.vel_des=[-0.03,0.0,0]
                msg.life_count=(msg.life_count+1)%128    
                Ctrl.Send_cmd(msg)
                time.sleep(0.6)

                print("move over")

                #停止
                # msg.mode = 21  
                # msg.gait_id = 0
                # msg.rpy_des=[0,-0.3,0]
                # msg.duration = 0
                # msg.life_count += 1
                # Ctrl.Send_cmd(msg)
                # time.sleep(4)

                msg.mode = 3
                msg.gait_id = 0
                msg.vel_des = [0.0, 0.0, 0.0]
                msg.rpy_des = [ 0.0, -1.8, 0.0]
                msg.pos_des = [0.0, 0.0, 0.235]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(4)
                print("抬头结束")

                findflag,most_frequent_direction = arrow_detector.get_arrow_direction(real_rgb_camera)
                if findflag == False:
                    process(processor.rotate_speed)
                else:
                    #print(most_frequent_direction)
                    arrow_detector.real_direction = most_frequent_direction
                    print(f"last_direction:{arrow_detector.real_direction}")
                    break

        if index == 4 and order == 2:
            #print(f"order:{order}")
            frame_real_rgb = real_rgb_camera.get_new_frame()
            #print("have got frame in purplr")

            
            if purple_detector.process_purple(frame_real_rgb):
                count += 1
                print("find purple count:{count}!")
                if count >= 7:
                    standup()
                    break
                # #停止
                # msg.mode = 21  
                # msg.gait_id = 0
                # msg.rpy_des=[0,-0.3,0]
                # msg.duration = 0
                # msg.life_count += 1
                # Ctrl.Send_cmd(msg)
                # time.sleep(2)                

 
def run_cur():  # s
    print(f"order in runcur:{order}")
    if order == 1:#第一次走s弯
        processor.rotate_speed = 0.35
        process(processor.rotate_speed)
        time.sleep(4)#初始位置必要
        detecte(1,1,0)
        print('det')

        
        new_move(True,5)
        go_center(True)
        print("直行1")
        msg.mode=11
        msg.gait_id=27
        msg.step_height = [0.03,0.03]
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.5)
        print("直行结束")

        processor.rotate_speed = -0.34
        process(processor.rotate_speed)
        detecte(0,2,1)

        new_move(True,5)
        go_center(True)
        print("直行2")
        msg.mode=11
        msg.gait_id=27
        msg.step_height = [0.03,0.03]
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.2)
 
        print("直行结束")
        processor.rotate_speed = 0.35
        process(processor.rotate_speed)
        detecte(1,3,0)

        real_rgb_camera.is_node_running = True
        print(f"real_rgb_camera.is_node_running:{real_rgb_camera.is_node_running}")
        
        new_move(False,5)
        go_center(False)
        """
        print("直行3")
        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.1)
        print("直行结束")
        """
        processor.rotate_speed = -0.35
        #print("wandao3")
        process(processor.rotate_speed)
        print(real_rgb_camera.is_node_running)
        detecte(1,4,1)

    elif order == 2:#第二次走s弯
        print(f"order in order == 2 {order}")

        processor.rotate_speed = 0.24
        process(processor.rotate_speed)
        time.sleep(1.5)
        detecte(1,1,0)

        new_move(True,5)
        go_center(True)
        #print("返回直行1")
        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.2)
        #print("直行结束")

        processor.rotate_speed = -0.33
        process(processor.rotate_speed)
        detecte(0,2,1)

        new_move(False,5)
        go_center(False)
        #print("返回直行2")
        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.5)
        #print("直行结束")

        processor.rotate_speed = 0.3
        process(processor.rotate_speed)
        detecte(1,3,0)

        new_move(False,5)
        go_center(False)
        #print("返回直行3")

        real_rgb_camera.is_node_running = True
        print(f"real_rgb_camera.is_node_running:{real_rgb_camera.is_node_running}")

        msg.mode=11
        msg.gait_id=27
        msg.vel_des=[0.2,0,0]
        msg.rpy_des=[0,-0.45,0]
        msg.life_count=(msg.life_count+1)%128
        Ctrl.Send_cmd(msg)
        time.sleep(0.1)
        #print("直行结束")


        processor.rotate_speed = -0.29
        process(processor.rotate_speed)
        detecte(0,4,1)
    else:
        print('yjmxhlxw')

def jiaozheng1():
    angle,distance = arrow_detector.get_arrow_adjust(real_rgb_camera)
    print(f"angle,distance:{angle,distance}")
    angle += 5 
    msg.mode=11
    msg.gait_id=27
    msg.step_height = [0.03,0.03]
    yaw_speed = 0.2  #以千为单位
    msg.duration = int(1000*(angle)*3.14/(180*yaw_speed))
    if angle < 0:
        yaw_speed = -yaw_speed
    msg.vel_des=[0.05,0,yaw_speed]
    msg.life_count=(msg.life_count+1)%128    
    Ctrl.Send_cmd(msg)
    time.sleep(8)
    print("have jiaozheng1")


    #distance = 1.2

    msg.mode=11
    msg.gait_id=27
    msg.step_height = [0.03,0.03]
    msg.duration = 8000
    vel = (distance-0.4)*1000/msg.duration
    msg.vel_des=[vel,0,0]
    msg.life_count=(msg.life_count+1)%128     
    Ctrl.Send_cmd(msg)
    time.sleep(6)

# def jiaozheng2():
#     angle_0 = 0.0
#     Kp = 1.2
#     Kd = 1.0
#     max_yaw = 30#角度制
#     acc = 5
#     #print("jiaozheng2 start")
#     while(1):
#         frame = real_rgb_camera.get_new_frame()
#         if not purple_detector.process_purple(frame):
#             continue

#         current_angle = purple_detector.angle
#         if current_angle == None:
#             continue
#         if abs(current_angle) < acc:
#             break
#         delta_angle = current_angle - angle_0
#         angle_0 = current_angle
#         yaw_speed = np.clip(current_angle * Kp + delta_angle * Kd,-max_yaw,max_yaw)
#         msg.mode=11
#         msg.gait_id=27
#         msg.step_height = [0.03,0.03]
#         yaw_speed = yaw_speed/180*3.14
#         msg.vel_des=[0.05,0.0,-yaw_speed]
#         msg.life_count=(msg.life_count+1)%128    
#         Ctrl.Send_cmd(msg)
#         time.sleep(0.5)
#     #print("jiaozheng2 over")
#     msg.mode=11
#     msg.gait_id=27
#     msg.duration = 3000
#     vel = 1.0*1000/msg.duration
#     msg.vel_des=[vel,0,0]
#     msg.life_count=(msg.life_count+1)%128    
#     Ctrl.Send_cmd(msg)
#     time.sleep(5)

def jiaozheng2():

    #位移一下
    msg.mode=11
    msg.gait_id=27
    msg.vel_des=[0.02,0.1,0]
    msg.duration = 1000
    msg.life_count=(msg.life_count+1)%128    
    Ctrl.Send_cmd(msg)
    time.sleep(1.3)

    #旋转一下
    angle = 40
    w = angle*3.14/180
    msg.mode=11
    msg.gait_id=27
    msg.vel_des=[0.03,0.0,-w]
    msg.duration = 1000 
    msg.life_count=(msg.life_count+1)%128
    Ctrl.Send_cmd(msg)
    time.sleep(3)

    while(True):
        #与紫色箭头后面的黄线对齐
        frame = real_rgb_camera.get_new_frame()
        print("have get new frame in purple")
        duiqi_node.get_yellow_line_y(frame)
        #调试
        # print(f"duiqi_node.angle_iny:{duiqi_node.angle_iny}")
        # print(f"duiqi_node.distance_iny:{duiqi_node.distance_iny}")
        # time.sleep(100000)

        if duiqi_node.find_line == False:
            print("no line behind purple")
            jiaozheng_bianxian(5.0*10*3.14/180)
            frame = real_rgb_camera.get_new_frame()
            duiqi_node.get_yellow_line_y(frame)
            if duiqi_node.find_line == False:
                print("no line behind purple really")
                break

        print(f"duiqi_node.angle_iny:{duiqi_node.angle_iny}")
        if duiqi_node.angle_iny >=0.1:
            jiaozheng_bianxian(6.6*duiqi_node.angle_iny*3.14/180)
        print(f"angle_over")
        time.sleep(3)
        frame = real_rgb_camera.get_new_frame()
        duiqi_node.get_yellow_line_y(frame)
        print(f"Distance_iny: {duiqi_node.distance_iny}")
        if duiqi_node.distance_iny >= 2:
            go_short_bianxian(0.17,3000) #50cm
            time.sleep(0.5)
        elif duiqi_node.distance_iny > 1.3 and duiqi_node.distance_iny < 2: 
            vel = (duiqi_node.distance_iny- 1.3)/3 
            go_short_bianxian(vel,3000)
            time.sleep(0.5) #goshortbianxian 本身的sleep太短了
            print("distance over in purple")
            break
        if duiqi_node.distance_iny <=1.3 :
            break

    #左转
    jiaozheng_bianxian(138*3.14/180)

    # # A库二维码底线调整角度
    # frame = real_rgb_camera.get_new_frame()
    # print("have get new frame after purple")
    # duiqi_node.get_yellow_line_y(frame)
    # if duiqi_node.find_line == False:
    #     print("no line after purple")
    #     jiaozheng_bianxian(-4.4*3*3.14/180)
    #     frame = real_rgb_camera.get_new_frame()
    #     duiqi_node.get_yellow_line_y(frame)
    #     if duiqi_node.find_line == False:
    #         print("no line after purple really")
    #         return
    
    # print(f"duiqi_node.angle_iny:{duiqi_node.angle_iny}")
    # jiaozheng_bianxian(5.0*duiqi_node.angle_iny*3.14/180)
    # print(f"after purple angle_over")    

    return

def xuanzhuan(direction):
    bia = 35
    msg.mode=11
    msg.gait_id=27

    msg.duration = 3000  
    yaw_speed = 1000*(90+bia)*3.140/(180*msg.duration)  
    if direction == Direction.Right:
        yaw_speed = -yaw_speed
    msg.vel_des=[0,0,yaw_speed]
    msg.life_count=(msg.life_count+1)%128    
    Ctrl.Send_cmd(msg)
    time.sleep(6)

def jiaozheng(yaw): #弧度
    msg.mode=11
    msg.gait_id=27
    yaw_speed = 20/180*3.14
    msg.duration= int(abs(yaw/yaw_speed)*1000)
    if yaw < 0:
        yaw_speed = -yaw_speed
    msg.vel_des=[0.05,0.0,yaw_speed]
    msg.life_count=(msg.life_count+1)%128    
    Ctrl.Send_cmd(msg)
    time.sleep(4)
BIA = 0.27576 - 0.164
detector = YellowLightDetector()
def yellowlight(vel=0.2, step=0.01, duration=2000):
    count = 0
    ratio = 2.2
    yaw_ratio_p = 2.8
    yaw_ratio_n = 1.8
    x_distance_ratio = 1
    x_distance_bia = 0.0
    count_max = 4
    flag_headup = False
    while(True):
        # 获取最新图像
        frame = vision_node.get_new_frame()
        print("get new frame")
        if frame is None:
            time.sleep(0.1)
            continue
        count += 1
        try:
            # 
            flag = detector.measure_distance(frame)
            if flag == False:
                flag_headup = True
                if count == 1:
                    print("no yellow light")
                    print("抬头")
                    msg.mode = 3
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -1.8, 0.0]
                    msg.pos_des = [0.0, 0.0, 0.235]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1)
                    continue
                else:
                    print("no yellow light really, go 60cm ")
                    Ctrl.go_short(vel*ratio*0.6, step, duration)
                    print("have go")
                    
            else:

                if count <=count_max:
                    if detector.yaw >=0:
                        yaw = yaw_ratio_p*detector.yaw
                    else :
                        yaw = yaw_ratio_n*detector.yaw
                    print(f"[黄灯控制] 偏移角度: {math.degrees(yaw):.2f}\u00B0")
                    jiaozheng(yaw)


            print("jiaozheng over")
            #standup()
            time.sleep(0.7)

            if flag_headup:
                    msg.mode = 3
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -1.8, 0.0]
                    msg.pos_des = [0.0, 0.0, 0.235]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1)
                    flag_headup = False

            frame = vision_node.get_new_frame()
            detector.measure_distance(frame)
            distance = detector.z_distance
            x_distance = (detector.x_distance+x_distance_bia)*x_distance_ratio
            print(f"偏移量: {x_distance:.2f}m")

            if count <=0:
                msg.mode=11
                msg.gait_id=27
                msg.duration= 3000
                change_vel = abs(x_distance*1000/msg.duration)
                if x_distance > 0:
                    change_vel = -change_vel
                print(f"[黄灯控制] x_速度: {change_vel:.2f}m/s")
                msg.vel_des=[0.0,change_vel,0.0]
                msg.life_count=(msg.life_count+1)%128    
                Ctrl.Send_cmd(msg)
                time.sleep(3.5)
                print("x have changed")
                #standup()
            # 应用偏移量
            current_distance = distance + BIA
            print(f"[黄灯控制] 当前距离: {current_distance:.2f}m")
            
            if current_distance >= 2.0+BIA:
                #print("执行前进1m")
                Ctrl.go_short(vel*ratio*0.7, step, duration)  # 前进0.7m
                #standup()
                print("have go")
                #time.sleep(2)
            elif 1.5+BIA <= current_distance < 2.0+BIA:
                #print("执行前进0.5m")
                Ctrl.go_short(vel*ratio*0.5, step, duration)  # 前进0.5m
                #standup()
                print("have go")
                #time.sleep(2)
            elif 0.8+BIA <= current_distance < 1.5+BIA:
                move_dist = (current_distance - 0.5)*ratio
                #print(f"执行前进{move_dist:.2f}m")
                adjusted_duration = int(duration * (move_dist/1.0))
                Ctrl.go_short(vel, step, adjusted_duration)
                #standup()
                print("have go")
                #time.sleep(2.5)
                #jiaozheng(-yaw_ratio*10*3.14/180) #!!!!!!!!!!!!!!!!这是给死的 不一定准！
                print("yellow over")
                break  # 结束检测循环
            elif 0+BIA<current_distance<0.8+BIA:
                print("error,当前距离过近")
                break
            else:
                #print("距离过远,前进1m")
                Ctrl.go_short(vel, step, int(duration*2))
                
            # 显示结果
            #detector.display_result(frame)
  
            
        except Exception as e:
            print(f"图像处理失败: {str(e)}")
            time.sleep(0.1)


def walk(duration = 5, dir = 0.00):  
    msg.mode = 11
    msg.gait_id = 3
    msg.vel_des = [0.1, dir, 0]
    msg.life_count = (msg.life_count + 1) % 128
    msg.duration = duration * 1000  
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 27, duration)

def down():
    msg.mode = 7
    msg.gait_id = 1
    msg.vel_des = [0.3, 0, 0]
    msg.life_count = (msg.life_count + 1) % 128
    Ctrl.Send_cmd(msg)
    time.sleep( 0.1 )
    # Ctrl.Wait_finish(7, 1, 5)

def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        rclpy.shutdown()

yaw = 0.0
arrow_direction = Direction.UNKNOWN
arrow_lock = Lock()  # 保证线程安全
arrow = None
state_id = 0
now_state = 0
QR1 = ''
QR2 = ''
QR1_cnt = 0
QR2_cnt = 0
Ctrl = PC_cmd_send()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()

detector = YellowLightDetector()

vision_node = VisionNode()
rgb_camera = SteroCameraNode("rgb_camera")
real_rgb_camera = RGBCameraNode("real_rgb_camera")
depth_node = DepthNode('slope')
speech_node = SpeechProcessor()
executor = MultiThreadedExecutor()
executor.add_node(rgb_camera)
executor.add_node(vision_node)
executor.add_node(depth_node)
executor.add_node(real_rgb_camera)
executor.add_node(speech_node)
# 添加节点到执行器

spin_thread = Thread(target=spin_executor)
spin_thread.start()

def main():
    global state_id
    global now_state
    global QR1
    global QR2
    global arrow
    global order
    global msg
    global limittime
    QR1 = 'A-1' # 二维码1
    QR2 = 'B-2' # 二维码2
    arrow = Direction.Left
    # 状态机初始化
    state_id = 1
    depth_node.set_node_running(False)
    state_entry_time = time.time()
    last_cmd_time = time.time()
    cmd_interval = 0.1  # 命令发送间隔（秒）
    standup()
    time.sleep(2.5)
    try:
        while rclpy.ok():
            current_time = time.time()
            # 0 准备右转 前进速度0.3m/s
            if state_id == 0:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 3     # 步态类型
                    msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)

                    last_cmd_time = current_time

                if vision_node._0_detected:
                    print("enter s-1")
                    state_id = 1
                    print("进入状态1")

                
            # 1右转 0.5m/s 2.25rad/s 1.2s
            elif state_id == 1:
                # 继续直走一段距离
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.step_height = [0.02, 0.02]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.8 )

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.35, 0.0, -1.4]
                msg.step_height = [0.02, 0.02]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.3 )
                state_id = 2
                print("进入状态2")
           
            # 2 前进 0.3m/s
            elif state_id == 2:
                msg.mode = 11        # 行走模式
                msg.gait_id = 3     # 步态类型
                msg.vel_des = [0.3, 0, 0]  # 前进速度0.3m/s
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(3.5)
                last_cmd_time = current_time

                msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                time.sleep(1)
                last_cmd_time = current_time

                state_id = 4
                print("进入状态4")
                        
            # 3对齐黄线
            elif state_id == 3:
                print("3")
               
            # 4识别二维码
            elif state_id == 4:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 3
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -1.8, 0.0]
                    msg.pos_des = [0.0, 0.0, 0.235]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    '''msg.mode = 12 # Recovery stand
                    msg.gait_id = 0
                    msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                    Ctrl.Send_cmd(msg)
                    Ctrl.Wait_finish(12, 0)
                    last_cmd_time = current_time'''
                    print("站立，开始识别二维码")
                    #ifsth
                    state_id = 4.5
                    print("进入状态4.5")

            elif state_id == 4.5:
                #if QR1 == 'B-1':
                speech_node.play_speech("A区库位1")    
                
                #speech_node.play_speech("A区库位2")
                    

                cmd_state = None
                standup()
                #time.sleep(2)
                #time.sleep(1.5)
                # 直走
                # global QR1
                

                # 直走
                '''msg.mode = 11
                msg.gait_id = 10  
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.1 )'''

                # 这里必须站着不动等待转圈结束，不然后面的识别会有问题
                '''msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time'''

                state_id = 5
                print("进入状态5") 
                    
            # 5 识别二维码后，到位置转弯
            elif state_id == 5:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.2 )
                print("pass 5-sleep")
                # 右转
                print(QR1)
                dir = 1 if QR1 == 'A-1' else -1
                # dur = 1.3 if QR1 == 'A-2' else 1.3
                dur = 1.3
                print(str(dur) + "wdwdad")
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, -1.4 * dir]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                print("pass")
                # time.sleep( 1.22 ) 
                #time.sleep( dur ) 原代码
                t_start = time.time()
                while time.time() - t_start < 1.22:
                    Ctrl.Send_cmd(msg) # 持续发送指令维持运动
                    time.sleep(0.02)   # 20ms 发送一次 (50Hz)
                last_cmd_time = current_time
                state_id = 6
                print(state_id)
                # 没有dur不行，一到dur就结束fff
            # 6 到位置准备转弯
            elif state_id == 6:
                #print("enter" + str(state_id))
                #print("curr_t" + str(current_time) + "last_c_t" + str(last_cmd_time))
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.12, 0.0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._6_detected:
                    print("进入状态7")
                    state_id = 6.5


            # 6 到位置准备转弯
            elif state_id == 6.5:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                last_cmd_time = current_time
                time.sleep( 0.5 )

                print("进入状态7")
                state_id = 7

            # 7转
            elif state_id == 7:
                dir = 1 if QR1 == 'A-1' else -1
                dur = 1.3 if QR1 == 'A-2' else 1.4
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.1, 0.0, -1.4 * dir]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )

                # 这里必须站着不动等待转圈结束，不然后面的识别会有问题
                '''msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time'''
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.9 )

                state_id = 9
                print("进入状态8")    

            elif state_id == 8:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -0.8               # 比例系数
                KD = 1.5                # 微分系数
                # angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._8_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态9")
                            vision_node.angle_0 = 0
                            msg.mode = 11
                            msg.gait_id = 3
                            msg.vel_des = [0.1, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.step_height = [0.03, 0.03]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 9 
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.08, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            # 9到位置趴下
            elif state_id == 9:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0.0, 0.0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._9_detected:
                    state_id = 9.5
                    print("进入状态9.5")

            elif state_id == 9.5:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.7)
                last_cmd_time = current_time
                
                down()
                while(True):
                    if now_state == 1:
                        break
                now_state = 0
                #time.sleep(3)
                standup()
                
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [-0.3, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.2)
                print("进入状态10")
                state_id = 10

            # 10 准备出库。
            elif state_id == 10:
                '''if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [-0.15, 0.0, 0.0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._10_detected:
                    print("进入状态11")
                    state_id = 11'''
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [-0.2, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(3)
                print("进入状态11")
                state_id = 11

            # 11 
            elif state_id == 11:
                dir = 1 if QR1 == 'A-1' else -1
                mul = 1.5 if QR1 == 'A-1' else 2.2
                msg.mode = 11
                msg.gait_id = 10 
                msg.vel_des = [-0.3, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.01 * mul, 0.0, -1.4 * dir]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.28 )
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.4)
                state_id = 12
                print("进入状态12")


            # 12到位置拐出
            elif state_id == 12:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 3     # 步态类型
                    msg.vel_des = [0.1, 0, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._12_detected:
                    print("进入状态13")
                    
                    '''msg.mode = 11
                    msg.gait_id = 27
                    msg.vel_des = [0.5, 0.0, 1.4 * dir]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.3 )'''
                    state_id = 12.5

            # 12.5 检测到位置后，拐出
            elif state_id == 12.5:
                '''if QR1 == 'A-2':
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0.0, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 2 )'''
                dur = 0.5 if QR1 == 'A-2' else 0.5
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.1, 0.0, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )
                dir = 1 if QR1 == 'A-1' else -1
                dur = 1.24 if QR1 == 'A-2' else 1.2
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 1.4 * dir]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )

                # 必须转弯完后停一下
                '''msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time'''

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.01, 0.01]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.5 )

                '''msg.mode = 11
                msg.gait_id = 27
                msg.vel_des = [0.3, 0.0, 1.4]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.3 )

                # 必须转弯完后停一下
                msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time'''

                state_id = 13
                print("进入状态13")

            # 13对齐方向
            elif state_id == 13:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -0.4               # 比例系数
                KD = 1.6                # 微分系数
                # angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._13_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态14")
                            vision_node.angle_0 = 0
                            state_id = 14
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            # 14到位置右拐
            elif state_id == 14:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._14_detected:
                    msg.mode = 12 # Recovery stand
                    msg.gait_id = 0
                    msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                    Ctrl.Send_cmd(msg)
                    Ctrl.Wait_finish(12, 0)
                    last_cmd_time = current_time
                    print("进入状态14.5")
                    state_id = 14.5

            elif state_id == 14.5:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.6 )
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0, -1.4]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.24 )
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.01, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 3 )
                '''dur = 0.3 if QR1 == 'A-2' else 3
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.16, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )'''
                # 必须转弯完后停一下
                msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time

                print("进入状态15")
                rgb_camera.is_node_running = True
                # print(rgb_camera.is_node_running)
                state_id = 15
                

            # 15准备进S弯 跑s弯
            elif state_id == 15:
                #print(15)
                # global arrow
                rgb_camera.is_node_running = True
                real_rgb_camera.is_node_running = False
                # os.system('python3 order1and2_test_7_15.py')
                #print(rgb_camera.is_node_running)
                time.sleep(2)
                run_cur()
                jiaozheng1()
                arrow = arrow_detector.real_direction
                print(arrow)
                if arrow == Direction.Left:
                     speech_node.play_speech("左侧路线")
                else:
                     speech_node.play_speech("右侧路线")
                standup()
                time.sleep(4)
                if arrow == Direction.Right:
                    msg.mode=11
                    msg.gait_id=27     
                    msg.duration= 5000
                    msg.vel_des=[0.0,-0.3,0.0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count=(msg.life_count+1)%128    
                    Ctrl.Send_cmd(msg)
                    time.sleep(4)

                elif arrow == Direction.Left:
                    msg.mode=11
                    msg.gait_id=27     
                    msg.duration= 3600
                    msg.vel_des=[0.0,0.3,0.0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count=(msg.life_count+1)%128    
                    Ctrl.Send_cmd(msg)
                    time.sleep(4)
                # dir = 1 if arrow == Direction.Left else -1
                # msg.mode = 11
                # msg.gait_id = 10
                # msg.vel_des = [0.3, 0, -1.4 * dir]
                # msg.rpy_des = [0, 2.5, 0]
                # msg.life_count = (msg.life_count + 1) % 128
                # Ctrl.Send_cmd(msg)
                # time.sleep( 1.3 )

                rgb_camera.is_node_running = False
                real_rgb_camera.is_node_running = False
                state_id = 18
                standup()
                print("进入状态18")
                # break

            # 16对齐方向
            elif state_id == 16:
                ALIGN_DURATION = 3    # 需要持续对齐时间（秒）
                KP = -1.2              # 比例系数
                KD = 4.0                # 微分系数
                KI = 0.01
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.00      # 搜索转速（rad/s）
                

                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._16_detected:
                        angle_error = vision_node.line_angle

                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node._aligned_duration

                        if duration >= ALIGN_DURATION:
                            print("进入状态17")
                            vision_node.angle_0 = 0
                            state_id = 17
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD , -MAX_YAW, MAX_YAW)

                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.01, yaw_speed]
                        msg.rpy_des = [0, 3, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {angle_error:+.8f}\u00B0 | 持续: {duration:.3f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.01, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 17:
                rgb_camera.is_node_running = False
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 3     # 步态类型
                    msg.vel_des = [0.06, 0.01, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._17_detected:
                    print("进入状态17.5")
                    
                    '''dir = 1 if arrow == Direction.Left else -1
                    msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.1, 0.0, -1.4 * dir]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.24 )'''
                    state_id = 17.5
                    if arrow == Direction.Left:
                        depth_node.set_detection_type('stone')
                    else:
                        depth_node.set_detection_type('slope')

            elif state_id == 17.5:
                dir = 1 if arrow == Direction.Left else -1
                standup()
                time.sleep(1)
                print(175)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.55 )
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, -1.4 * dir]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.31 )
                state_id = 18

            # 18石板路/坡道
            elif state_id == 18:
                if arrow == Direction.Left:
                    # depth_node.set_detection_type('stone')
                    #no depth
                    # stone_detected = main_detection_loop(
                    #     depth_node, Ctrl, msg, 
                    #     trigger_distance=0.55, 
                    #     detection_type="石头路"
                    # )
                    stone_detected = True

                    if stone_detected:
                        # 执行石板路动作
                        # cv2.destroyWindow("Stone Detection")

                        #no depth
                        # standup()
                        # msg.mode = 11
                        # msg.gait_id = 27
                        # msg.vel_des = [0.3, 0, 0]
                        # msg.step_height = [0.01, 0.01]
                        # msg.life_count = (msg.life_count + 1) % 128
                        # time.sleep(1)

                        standup()
                        stone_road(88)
                        # ctrl.Wait_finish(62, 81, duration)
                        state_id = 18.5
                        # vision_node.last_error = 0
                        # vision_node.integral = 0
                        state_entry_time = time.time()
                else:
                    '''msg.mode = 11
                    msg.gait_id = 27
                    msg.vel_des = [0.1, 0, 0]
                    msg.step_height = [0.04, 0.04]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1)
                    standup()
                    time.sleep(1)
                    upslope(Ctrl, msg)
                    walk(0.2, 0.2)         # 取决于中间平台走多久
                    # time.sleep(2)
                    standup()
                    downslope(Ctrl, msg)
                    # walk(5)
                    state_id = 18.5
                    state_entry_time = time.time()'''

                    #no depth

                    # slope_detected = main_detection_loop(
                    #     depth_node, Ctrl, msg, 
                    #     trigger_distance=1.4, 
                    #     detection_type="斜坡"
                    # )
                    slope_detected = True  
                    if slope_detected:
                        # 执行上斜坡动作
                        #cv2.destroyWindow("Slope Detection")

                        # no depth
                        # standup()
                        # msg.mode = 11
                        # msg.gait_id = 27
                        # msg.vel_des = [0.1, 0, 0]
                        # #msg.step_height = [0.04, 0.04]
                        # msg.life_count = (msg.life_count + 1) % 128
                        # Ctrl.Send_cmd(msg)
                        # time.sleep(0.5)

                        standup()  
                        upslope(12)  
                        print("Upslope ends.")
                        stand()
                        standup()
                        '''msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.15, 0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        msg.step_height = [0.02, 0.02]
                        #msg.duration = duration * 1000  
                        Ctrl.Send_cmd(msg)
                        time.sleep(2) '''   
                        #standup()
                        downslope()
                        msg.mode = 11
                        msg.gait_id = 27
                        msg.vel_des = [0.15, 0, -0.1]
                        msg.life_count = (msg.life_count + 1) % 128
                        msg.step_height = [0.03, 0.03]
                        #msg.duration = duration * 1000  
                        Ctrl.Send_cmd(msg)
                        time.sleep(2)
                        # walk(1)
                        # walk(5)
                        state_id = 18.5
                        state_entry_time = time.time()

            # 
            elif state_id == 18.5:
                # PID控制器参数
                KP_YAW = 0.7      # 偏航控制比例系数
                KD_YAW = 1.0      # 偏航控制微分系数
                KI_YAW = 0.05     # 偏航控制积分系数
                MAX_YAW = 0.2     # 最大偏航速度
                
                # 横向位置控制参数
                KP_LATERAL = 0.1  # 横向控制比例系数
                MAX_LATERAL = 0.1 # 最大横向速度
                
                # 初始化控制变量
                '''integral = 0.0
                last_error = 0.0'''
                
                current_time = time.time()
                time_since_entry = current_time - state_entry_time
                
                # 获取视觉误差
                yaw_error = vision_node.yaw_adjust
                lateral_error = vision_node.lateral_error
                
                # 偏航控制计算
                
                delta_error = yaw_error - vision_node.last_error
                # print(1)
                '''integral += yaw_error * cmd_interval
                integral = np.clip(vision_node.integral, -1.0, 1.0)'''
                # print(2)
                yaw_speed = KP_YAW * yaw_error + KD_YAW * delta_error
                yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                
                # 横向位置控制计算
                lateral_speed = KP_LATERAL * lateral_error
                lateral_speed = np.clip(lateral_speed, -MAX_LATERAL, MAX_LATERAL)
                
                # 构造控制指令
                # msg = MotionMsg()
                msg.mode = 11                  # 运动模式
                msg.gait_id = 3               # 步态类型
                msg.vel_des = [0.01, lateral_speed, yaw_speed]  # [前进速度，横向速度，偏航速度]
                msg.rpy_des = [0, 2.5, 0]      # 姿态角度
                msg.step_height = [0.01, 0.01] # 步高
                msg.life_count = (msg.life_count + 1) % 128
                
                # 发送控制指令
                Ctrl.Send_cmd(msg)
                
                # 调试输出
                if vision_node.debug_mode:
                    print(f"Control | YawErr: {np.degrees(yaw_error):+.1f}\u00B0 "
                        f"LatErr: {lateral_error:.2f} | "
                        f"YawSpd: {yaw_speed:.2f} LatSpd: {lateral_speed:.2f}")
                
                # 更新状态变量
                vision_node.last_error = yaw_error
                last_cmd_time = current_time
                
                # 状态退出条件
                if time_since_entry > 5 and abs(yaw_error) < 0.2:  # 15秒后退出
                    if arrow == Direction.Right:
                        depth_node.set_node_running(False)
                    else:
                        depth_node.set_detection_type('limit')
                    state_id = 19
                    vision_node.last_error = 0
                    vision_node.integral = 0
                    print("进入状态19")
                
                time.sleep(cmd_interval)  # 控制循环频率
            
            # 19限高杆/黄灯
            elif state_id == 19:
                if arrow == Direction.Left:
                    depth_node.set_detection_type('limit')
                    standup()
                    print(19)
                    limit_detected = main_detection_loop(
                        depth_node, Ctrl, msg, 
                        trigger_distance=1, 
                        detection_type="限高杆"
                    )
                    
                    if limit_detected:
                        # 执行限高杆动作
                        # cv2.destroyWindow("Limit Detection")
                        # standup(Ctrl, msg)
                        limittime = 3
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.2, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( limittime )
                        # standup(Ctrl, msg)
                        standup()
                        limit(Ctrl, msg)
                        standup()
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.05, 0.1]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 1 )
                        print('进入19.5')
                        standup()
                        '''msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.2, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 1.5 )'''

                        state_id = 19.5
                        state_entry_time = time.time()
                else:
                    standup()
                    yellowlight()
                    standup()
                    speech_node.start_countdown()
                    time.sleep(7)
                    state_id = 19.5
                    state_entry_time = time.time()

            
            elif state_id == 19.5:
                # PID控制器参数
                KP_YAW = 0.7      # 偏航控制比例系数
                KD_YAW = 1.0      # 偏航控制微分系数
                KI_YAW = 0.05     # 偏航控制积分系数
                MAX_YAW = 0.2     # 最大偏航速度
                
                # 横向位置控制参数
                KP_LATERAL = 0.1  # 横向控制比例系数
                MAX_LATERAL = 0.1 # 最大横向速度
                
                # 初始化控制变量
                '''integral = 0.0
                last_error = 0.0'''
                
                current_time = time.time()
                time_since_entry = current_time - state_entry_time
                
                # 获取视觉误差
                yaw_error = vision_node.yaw_adjust
                lateral_error = vision_node.lateral_error
                
                # 偏航控制计算
                
                delta_error = yaw_error - vision_node.last_error
                # print(1)
                '''integral += yaw_error * cmd_interval
                integral = np.clip(vision_node.integral, -1.0, 1.0)'''
                # print(2)
                yaw_speed = KP_YAW * yaw_error + KD_YAW * delta_error
                yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                
                # 横向位置控制计算
                lateral_speed = KP_LATERAL * lateral_error
                lateral_speed = np.clip(lateral_speed, -MAX_LATERAL, MAX_LATERAL)
                
                # 构造控制指令
                # msg = MotionMsg()
                msg.mode = 11                  # 运动模式
                msg.gait_id = 3               # 步态类型
                msg.vel_des = [0.01, lateral_speed, yaw_speed]  # [前进速度，横向速度，偏航速度]
                msg.rpy_des = [0, 2.5, 0]      # 姿态角度
                msg.step_height = [0.01, 0.01] # 步高
                msg.life_count = (msg.life_count + 1) % 128
                
                # 发送控制指令
                Ctrl.Send_cmd(msg)
                
                # 调试输出
                if vision_node.debug_mode:
                    print(f"Control | YawErr: {np.degrees(yaw_error):+.1f}\u00B0"
                        f"LatErr: {lateral_error:.2f} | "
                        f"YawSpd: {yaw_speed:.2f} LatSpd: {lateral_speed:.2f}")
                
                # 更新状态变量
                vision_node.last_error = yaw_error
                last_cmd_time = current_time
                
                # 状态退出条件
                if time_since_entry > 10 and abs(yaw_error) < 0.1:  # 15秒后退出
                    state_id = 19.55
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0.0, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 3 )
                    vision_node.last_error = 0
                    vision_node.integral = 0
                    print("进入状态1955")
                
                time.sleep(cmd_interval)  # 控制循环频率
            
            elif state_id == 19.55:
                dir = 1 if arrow == Direction.Left else -1
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, -0.005 * dir, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height = [0.02, 0.02]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                # 检测到并决定转向
                if vision_node._19_55_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.5)
                    # standup()
                    dir = 1 if arrow == Direction.Left else -1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.0, 0, -1.3 * dir]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(0.6)
                    standup()
                    state_id = 19.6

            elif state_id == 19.6:
                if current_time - last_cmd_time > cmd_interval:
                    
                    msg.mode = 3
                    msg.gait_id = 0
                    msg.vel_des = [0.0, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, -1.8, 0.0]
                    msg.pos_des = [0.0, 0.0, 0.235]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    # print("站立，开始识别二维码")

                if vision_node._19_6_detected:
                    if QR2 == 'B-1':
                        speech_node.play_speech("B区库位1")
                        #pass
                    else:
                        #pass
                        speech_node.play_speech("B区库位2")
                    time.sleep(1)
                    cmd_state = None
                    dir = 1 if arrow == Direction.Left else -1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.0, 0, 0.55 * dir]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.3)
                    standup()
                    state_id = 20
                    print("进入状态20")

            #20拐(left+B2 or right+B1)->21，不拐(left+B1 or right+B2)->23
            elif state_id == 20:
                # print(20)
                # 周期性发命令保持运动状态
                dir = 0.0 if arrow == Direction.Right else -0.01
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, dir, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                # 检测到并决定转向
                if vision_node._20_detected:
                    
                    if arrow == Direction.Left:
                        # state_id = 23 if QR2 == 'B-1' else 21
                        dir = 0 if QR2 == 'B-1' else 1
                    else:
                        # state_id = 23 if QR2 == 'B-2' else 21
                        dir = 0 if QR2 == 'B-2' else -1
                    dur = 1.3 if arrow == Direction.Left else 1.25
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.15 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.15, 0, -1.4 * dir]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( dur )
                    if dir != 0:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.3, 0, 0]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 3.0 )
                        state_id = 21
                        print("进入状态21")
                        continue
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.3, 0, 0] #y给正为左
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 1.3 )
                    if arrow == Direction.Left:
                        state_id = 29
                        print("进入状态29")
                    else:
                        state_id = 23
                        print("进入状态23")
                    # break
                    # state_id = 29

            
            # 21对齐方向
            elif state_id == 21:
                ALIGN_DURATION = 0.1    # 需要持续对齐时间（秒）
                KP = -0.6               # 比例系数
                KD = 1.0                # 微分系数
                # angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._21_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态22")
                            state_id = 22
                            vision_node.angle_0 = 0
                            # vision_node.aligned_duration = 0
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.04, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            # 22到位置拐弯
            elif state_id == 22:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.08, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._22_detected:
                    # 直走0.2s
                    dir = 1 if arrow == Direction.Left else -1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.1 )   
                    # 左转弯1.14s
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.15, 0.0, 1.4 * dir]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.31 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.0 )
                    if QR2 == 'B-2':
                        print("进入状态23")
                        state_id = 24
                    else:
                        print("进入状态29")
                        state_id = 30
                # break

            # 23 进B2
            elif state_id == 23:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -0.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._23_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态24")
                            vision_node.angle_0 = 0
                            # vision_node.aligned_duration = 0
                            state_id = 24
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        # msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.05, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            # 24到位置趴下
            elif state_id == 24:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.08, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._24_detected:
                    # state_id = 25
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1)
                    print('趴下')
                    down()
                    time.sleep(3)

                    #进入交互环节
                    while(True):
                        if now_state == 1:
                            break
                    now_state = 0

                    #交互成功
                    standup()
                    time.sleep(1)
                    state_id = 25
                    print("进入状态25")

            # 25起身向后转
            elif state_id == 25:
                # time.sleep(5)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, -0.8]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(4.7)

                #test 原地踏步
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.4)

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.1, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                
                '''if QR2 == 'B-2':
                    print("进入状态26")
                    state_id = 26
                else:
                    print("进入状态32")
                    state_id = 32'''
                # state_id = 26
                print("进入状态26")
                state_id = 26
                state_entry_time = time.time()

                # break

            elif state_id == 26:# 出B2
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._26_detected:
                    # standup()
                    # break
                    # # 检测到黄色区域走1.6s
                    vision_node.angle_0 = 0
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.3 )
                    dir = 0 if QR2 == 'B-1' and arrow == Direction.Left else 1
                    #右转 1s 
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0.0, -1.4 * dir]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.28 )
                    
                    if dir == 1:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.3, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 3 )
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.2, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 2 )

                    if QR2 == 'B-1' and arrow == Direction.Left:
                        print("进入状态34.5")
                    else:
                        print("进入状态27")
                    state_id = 34.5 if QR2 == 'B-1' and arrow == Direction.Left else 27
                    state_entry_time = time.time()
                    # vision_node.aligned_duration = 0
            
            
            elif state_id == 27:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -0.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._27_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态28")
                            vision_node.angle_0 = 0
                            # vision_node.aligned_duration = 0
                            state_id = 28
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.09, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
                # break

            # 28到位置右拐弯
            elif state_id == 28:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.06, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._28_detected:
                    # break
                    '''if QR2 == 'B-2':
                        print("state28.5")
                    else:
                        print("state34.5")'''
                    state_id = 28.5 # if QR2 == 'B-2' else 34.5
                    state_entry_time = time.time()
                    
                    # vision_node.aligned_duration = 0
            
            elif state_id == 28.5:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.1, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.1 )
                dir = 1 if QR2 == 'B-2' else -1
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.15, 0.0, -1.4 * dir]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.33 )
                
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.4 ) 
                
                # vision_node.aligned_duration = 0
                if QR2 == 'B-2':
                    print("state29")
                else:
                    print("state34.5")
                state_id = 29 if QR2 == 'B-2' else 34.5

            # 29进B1
            elif state_id == 29:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -0.8               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._29_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态30")
                            vision_node.angle_0 = 0

                            state_id = 30
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.01, 0.01]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            # 30到位置趴下
            elif state_id == 30:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.08, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._30_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1)
                    down()
                    time.sleep(2)

                    #进入交互
                    while(True):
                        if now_state == 1:
                            break
                    now_state = 0

                    print("进入状态31")
                    standup()
                    time.sleep(1)
                    state_id = 31
            
            # 31起身向后转
            elif state_id == 31:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, -0.8]
                msg.rpy_des = [0, 2.5, 0]
                # msg.step_height = [0.03, 0.03]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(4.7)

                #test 原地踏步
                '''msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.0, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.step_height = [0.03,0.03]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.4)'''

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.4, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.step_height =[0.03,0.03]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                
                '''if QR2 == 'B-1':
                    print("进入状态26")
                    state_id = 26
                else:'''
                print("进入状态32")
                state_id = 32
                # state_id = 26
                state_entry_time = time.time()
            
            # 32到位置拐弯 (出B1)
            elif state_id == 32:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.12, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._32_detected:
                    # standup()
                    vision_node.angle_0 = 0
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.28 )
                    dir = 0 if QR2 == 'B-2' and arrow == Direction.Right else 1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0.0, 1.4 * dir]
                    msg.step_height = [0.03,0.03]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.26 )
                    if QR2 == 'B-2' and arrow == Direction.Right:
                        print("进入状态34.5")
                    else:
                        print("进入状态33")
                    if dir == 1:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.3, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 3.3 )
                    state_id = 34.5 if QR2 == 'B-2' and arrow == Direction.Right else 33
                    state_entry_time = time.time()
                    # state_id = 33
            
            # 33对齐方向
            elif state_id == 33:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -0.4               # 比例系数
                KD = 1.6                # 微分系数
                # angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._33_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态34")
                            state_id = 34
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        # msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.06, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        # msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            # 34到位置右拐弯
            elif state_id == 34:
                # print(1)
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._34_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.1 )
                    dir = -1 if QR2 == 'B-2' and arrow == Direction.Left else 1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0.0, 1.4 * dir]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.35 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.0 ) 
                    # 原地踏步，不然state34.5会导致转弯角度不对
                    if QR2 == 'B-2' and arrow == Direction.Left:
                        print("进入状态34_5")
                    else:
                        print("进入状态23")
                    state_id = 34.5 if QR2 == 'B-2' and arrow == Direction.Left else 23
                    state_entry_time = time.time()
                    

            elif state_id == 34.5:
                # PID控制器参数
                KP_YAW = 0.7      # 偏航控制比例系数
                KD_YAW = 1.0      # 偏航控制微分系数
                KI_YAW = 0.05     # 偏航控制积分系数
                MAX_YAW = 0.2     # 最大偏航速度
                
                # 横向位置控制参数
                KP_LATERAL = 0.4  # 横向控制比例系数
                MAX_LATERAL = 0.1 # 最大横向速度
                
                # 初始化控制变量
                '''integral = 0.0
                last_error = 0.0'''
                
                current_time = time.time()
                time_since_entry = current_time - state_entry_time
                
                # 获取视觉误差
                yaw_error = vision_node.yaw_adjust
                lateral_error = vision_node.lateral_error
                
                # 偏航控制计算
                
                delta_error = yaw_error - vision_node.last_error
                # print(1)
                '''integral += yaw_error * cmd_interval
                integral = np.clip(vision_node.integral, -1.0, 1.0)'''
                # print(2)
                yaw_speed = KP_YAW * yaw_error + KD_YAW * delta_error
                yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                
                # 横向位置控制计算
                lateral_speed = KP_LATERAL * lateral_error
                lateral_speed = np.clip(lateral_speed, -MAX_LATERAL, MAX_LATERAL)
                
                # 构造控制指令
                # msg = MotionMsg()
                msg.mode = 11                  # 运动模式
                msg.gait_id = 3               # 步态类型
                msg.vel_des = [0.01, lateral_speed, yaw_speed]  # [前进速度，横向速度，偏航速度]
                msg.rpy_des = [0, 2.5, 0]      # 姿态角度
                msg.step_height = [0.01, 0.01] # 步高
                msg.life_count = (msg.life_count + 1) % 128
                
                # 发送控制指令
                Ctrl.Send_cmd(msg)
                
                # 调试输出
                print(f"Control | YawErr: {np.degrees(yaw_error):+.1f}\u00B0 "
                        f"LatErr: {lateral_error:.2f} | "
                        f"YawSpd: {yaw_speed:.2f} LatSpd: {lateral_speed:.2f}")
                
                # 更新状态变量
                vision_node.last_error = yaw_error
                last_cmd_time = current_time
                
                # 状态退出条件
                if time_since_entry > 5 and abs(np.degrees(yaw_error)) < 0.3:  # 15秒后退出
                    state_id = 35
                    vision_node.last_error = 0
                    vision_node.integral = 0
                    print("进入状态35")
                
                time.sleep(cmd_interval)  # 控制循环频率
                # break
            
            elif state_id == 35:
                if arrow == Direction.Right:
                    standup()
                    depth_node.set_detection_type('limit')
                    limit_detected = main_detection_loop(
                        depth_node, Ctrl, msg, 
                        trigger_distance=1.0, 
                        detection_type="限高杆"
                    )
                    if limit_detected:
                        # 执行限高杆动作
                        # cv2.destroyWindow("Limit Detection")
                        standup()
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep( 1.5 )
                        limit(Ctrl, msg)
                        print('进入35.5')
                        state_id = 35.5
                        current_time = time.time()
                        state_entry_time = time.time()
                else:
                    standup()
                    yellowlight()
                    standup()
                    speech_node.start_countdown()
                    time.sleep(7)
                    '''msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0.0, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.5 )'''
                    print('进入35.5')
                    state_id = 35.55
                    current_time = time.time()
                    state_entry_time = time.time()
            
            elif state_id == 35.55:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.5 )
                state_id = 35.5
                current_time = time.time()
                state_entry_time = time.time()

            elif state_id == 35.5:
                # PID控制器参数
                KP_YAW = 0.7      # 偏航控制比例系数
                KD_YAW = 1.0      # 偏航控制微分系数
                KI_YAW = 0.05     # 偏航控制积分系数
                MAX_YAW = 0.2     # 最大偏航速度
                
                # 横向位置控制参数
                KP_LATERAL = 0.4  # 横向控制比例系数
                MAX_LATERAL = 0.2 # 最大横向速度
                
                # 初始化控制变量
                '''integral = 0.0
                last_error = 0.0'''
                
                current_time = time.time()
                time_since_entry = current_time - state_entry_time
                
                # 获取视觉误差
                yaw_error = vision_node.yaw_adjust
                lateral_error = vision_node.lateral_error
                
                # 偏航控制计算
                
                delta_error = yaw_error - vision_node.last_error
                # print(1)
                '''integral += yaw_error * cmd_interval
                integral = np.clip(vision_node.integral, -1.0, 1.0)'''
                # print(2)
                yaw_speed = KP_YAW * yaw_error + KD_YAW * delta_error
                yaw_speed = np.clip(yaw_speed, -MAX_YAW, MAX_YAW)
                
                # 横向位置控制计算
                lateral_speed = KP_LATERAL * lateral_error
                lateral_speed = np.clip(lateral_speed, -MAX_LATERAL, MAX_LATERAL)
                
                # 构造控制指令
                # msg = MotionMsg()
                msg.mode = 11                  # 运动模式
                msg.gait_id = 3               # 步态类型
                msg.vel_des = [0.01, lateral_speed, yaw_speed]  # [前进速度，横向速度，偏航速度]
                msg.rpy_des = [0, 2.5, 0]      # 姿态角度
                msg.step_height = [0.02, 0.02] # 步高
                msg.life_count = (msg.life_count + 1) % 128
                
                # 发送控制指令
                Ctrl.Send_cmd(msg)
                
                # 调试输出
                if vision_node.debug_mode:
                    print(f"Control | YawErr: {np.degrees(yaw_error):+.1f}\u00B0 "
                        f"LatErr: {lateral_error:.2f} | "
                        f"YawSpd: {yaw_speed:.2f} LatSpd: {lateral_speed:.2f}")
                
                # 更新状态变量
                vision_node.last_error = yaw_error
                last_cmd_time = current_time
                
                # 状态退出条件
                if time_since_entry > 5 and abs(np.degrees(yaw_error)) < 0.1:  # 15秒后退出
                    vision_node.last_error = 0
                    vision_node.integral = 0
                    print("进入状态36")
                    state_id = 36
                
                time.sleep(cmd_interval)  # 控制循环频率
            
            elif state_id == 36:
                if arrow == Direction.Right:
                    depth_node.set_detection_type('stone')
                    stone_detected = main_detection_loop(
                        depth_node, Ctrl, msg, 
                        trigger_distance=0.6, 
                        detection_type="石头路"
                    )

                    if stone_detected:
                        standup()
                        msg.mode = 11
                        msg.gait_id = 27
                        msg.vel_des = [0.1, 0, 0.02]
                        # msg.step_height = [0.04, 0.04]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep(2.5)
                        standup()
                        # time.sleep(1)
                        stone_road(88)
                        # ctrl.Wait_finish(62, 81, duration)
                        state_id = 36.5
                        # vision_node.last_error = 0
                        # vision_node.integral = 0
                        msg.mode = 11
                        msg.gait_id = 27
                        msg.vel_des = [0.01, -0.02, -0.02]
                        # msg.step_height = [0.04, 0.04]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep(3)
                        state_entry_time = time.time()
                else:
                    depth_node.set_detection_type('slope')
                    slope_detected = main_detection_loop(
                        depth_node, Ctrl, msg, 
                        trigger_distance=1.4, 
                        detection_type="斜坡"
                    )
                    
                    if slope_detected:
                        msg.mode = 11
                        msg.gait_id = 27
                        msg.vel_des = [0.1, 0, 0]
                        msg.step_height = [0.01, 0.01]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        time.sleep(6.7)
                        standup()  
                        upslope(15)  
                        print("Upslope ends.")
                        stand()
                        standup()
                        '''msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.15, 0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        msg.step_height = [0.02, 0.02]
                        #msg.duration = duration * 1000  
                        Ctrl.Send_cmd(msg)
                        time.sleep(2) '''   
                        #standup()
                        downslope()
                        msg.mode = 11
                        msg.gait_id = 27
                        msg.vel_des = [0.01, 0, -0.2]
                        msg.life_count = (msg.life_count + 1) % 128
                        msg.step_height = [0.01, 0.01]
                        #msg.duration = duration * 1000  
                        Ctrl.Send_cmd(msg)
                        time.sleep(0.1)
                        # walk(1)
                        # walk(5)
                        state_id = 36.5
                        state_entry_time = time.time()

            elif state_id == 36.5:
                ALIGN_DURATION = 0.3    # 需要持续对齐时间（秒）
                KP = -1.0               # 比例系数
                KD = 0.5                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.02          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._36_5_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态37")
                            msg.mode = 11
                            msg.gait_id = 3
                            msg.vel_des = [0.01, 0, 0]
                            msg.rpy_des = [0, 5, 0]
                            #msg.step_height = [0.02, 0.03]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            vision_node.angle_0 = 0
                            state_id = 37
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.00, 0.0, yaw_speed]
                        msg.rpy_des = [0, 5, 0]
                        msg.step_height = [0.01, 0.01]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 37:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.01, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._37_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 0, 0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 2.0)
                    dir = 1 if arrow == Direction.Left else -1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.01, 0.0, -1.4 * dir]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.44 )
                    print("进入状态38")
                    state_id = 39
            
            elif state_id == 38:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.01      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._38_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态39")
                            vision_node.angle_0 = 0
                            state_id = 39
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.03, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 0, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 39:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.16, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._39_detected:
                    # rgb_camera.is_node_running = True
                    dir = 1 if arrow == Direction.Right else -1
                    dur = 1.2 if arrow == Direction.Right else 2
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height=[0.01,0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( dur )
                    dur = 1.4 if arrow == Direction.Right else 1.6
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0, -1.4 * dir]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height=[0.01,0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.44 )
                
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 2 )
                    print("进入状态39.5")
                    
                    state_id = 39.5
            
            elif state_id == 39.5:
                #print(15)
                order = 2
                rgb_camera.is_node_running = True
                # os.system('python3 order1and2_test_7_15.py')
                #print(rgb_camera.is_node_running)
                time.sleep(2)
                print(order)

                run_cur()

                #just for test 正式的时候需要注释 调试
                # real_rgb_camera.is_node_running = True
                # print(real_rgb_camera.is_node_running)

                jiaozheng2()
                #xuanzhuan(Direction.Left)
                time.sleep(8) #函数自带的timesleep太短

                # print(arrow)
                rgb_camera.is_node_running = False
                real_rgb_camera.is_node_running = False
                state_id = 39.55
                print("进入状态40")            
            elif state_id == 39.55:
                msg.mode = 11                  # 运动模式
                msg.gait_id = 3               # 步态类型
                msg.vel_des = [0.2, 0, 0]  # [前进速度，横向速度，偏航速度]
                msg.rpy_des = [0, 2.5, 0]      # 姿态角度
                msg.step_height = [0.03, 0.03] # 步高
                msg.life_count = (msg.life_count + 1) % 128
                time.sleep(5)
                state_id = 40

            # 对齐底线
            elif state_id == 40:
                ALIGN_DURATION = 0.15    # 需要持续对齐时间（秒）
                KP = -1.5               # 比例系数
                KD = 1.0                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = -0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._40_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态41")
                            standup()
                            vision_node.angle_0 = 0
                            state_id = 41
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.02, 0.02]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.02, 0.02]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 41:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.step_height = [0.03, 0.03]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._41_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0.0, 0.0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.2 )
                    dir = 1 if QR1 == 'A-2' else -1
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.2, 0.0, -1.4 * dir]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.38 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0.0, 0]
                    msg.rpy_des = [ 0.0, 2.5, 0.0]
                    msg.step_height = [0.01, 0.01]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.2 )
                    state_id = 43
                    print("进入状态42")
            # 对齐方向准备入库        
            elif state_id == 42:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -1.2              # 比例系数
                KD = 4.0                # 微分系数
                KI = 0.01
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._42_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态43")
                            state_id = 43
                            vision_node.angle_0 = 0
                            '''msg.mode = 11
                            msg.gait_id = 3
                            msg.vel_des = [0.1, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.step_height = [0.03, 0.03]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)'''
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        # msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 43:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.12, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._43_detected:
                    print("进入状态44")
                    state_id = 44
                    
            elif state_id == 44:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.2, 0.0, 0.0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.4 )

                dir = 1 if QR1 == 'A-2' else -1
                dur = 1.3 if QR1 == 'A-1' else 1.56
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.1, 0.0, -1.4 * dir]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.step_height = [0.02, 0.02]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0]
                msg.rpy_des = [ 0.0, 2.5, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 0.8 ) 
                state_id = 45
                print("进入状态45")    

            elif state_id == 45:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -0.8               # 比例系数
                KD = 1.5                # 微分系数
                # angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.00      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._45_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态46")
                            msg.mode = 11
                            msg.gait_id = 3
                            msg.vel_des = [0.1, 0, 0]
                            msg.rpy_des = [0, 2.5, 0]
                            msg.step_height = [0.03, 0.03]
                            msg.life_count = (msg.life_count + 1) % 128
                            Ctrl.Send_cmd(msg)
                            time.sleep(0.5)
                            state_id = 46
                            vision_node.angle_0 = 0
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.08, 0.01, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.step_height = [0.03, 0.03]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time
            
            elif state_id == 46:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._46_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(0.7)
                    last_cmd_time = current_time

                    print("进入状态47")
                    down()
                    #进入交互
                    while(True):
                        if now_state == 1:
                            break
                    now_state = 0
                    # time.sleep(1)
                    standup()
                    time.sleep(2)

                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [-0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep(1.5)
                    state_id = 47

            elif state_id == 47:
                '''if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [-0.3, 0.0, 0.0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._47_detected:
                    print("进入状态48")
                    state_id = 48'''
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [-0.3, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(1.0)
                print("进入状态48")
                state_id = 48

            elif state_id == 48:
                # dir = 1 if QR1 == 'A-1' else -1
                dir = 1 if QR1 == 'A-2' else -1
                mul = 1.5 if QR1 == 'A-2' else 2.2
                msg.mode = 11
                msg.gait_id = 10 
                msg.vel_des = [-0.3, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.01 * mul, 0.0, -1.4 * dir]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.28 )
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep(0.4)
                state_id = 49
                print("进入状态49")

            elif state_id == 49:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11        # 行走模式
                    msg.gait_id = 3     # 步态类型
                    msg.vel_des = [0.1, 0, 0]  # 前进速度0.3m/s
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time
                
                if vision_node._49_detected:
                    #print("进入状态50")
                    '''msg.mode = 11
                    msg.gait_id = 10
                    msg.vel_des = [0.5, 0.0, -1.4]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.3 )'''
                    state_id = 49.5

            elif state_id == 49.5:
                dur = 0.5 if QR1 == 'A-2' else 0.3
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 0.0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )
                dir = 1 if QR1 == 'A-2' else -1
                dur = 1.24 if QR1 == 'A-2' else 1.2
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.0, 1.4 * dir]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( dur )

                # 必须转弯完后停一下
                '''msg.mode = 12 # Recovery stand
                msg.gait_id = 0
                msg.life_count = (msg.life_count + 1) % 128  # Command will take effect when life_count update
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                last_cmd_time = current_time'''

                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [0.3, 0.01, 0.01]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 1.5 )

                state_id = 50
                print("进入状态50")

            elif state_id == 50:
                ALIGN_DURATION = 0.5    # 需要持续对齐时间（秒）
                KP = -0.4               # 比例系数
                KD = 1.6                # 微分系数
                angle_0 = 0.0           # 初始化角度
                MAX_YAW = 0.05          # 最大转向速度（rad/s）
                SEARCH_SPEED = 0.0      # 搜索转速（rad/s）
                
                if current_time - last_cmd_time > cmd_interval:
                    if vision_node._50_detected:
                        angle_error = vision_node.line_angle
                        delta = vision_node.angle_0 - angle_error
                        vision_node.angle_0 = angle_error
                        duration = vision_node.aligned_duration
                        
                        if duration >= ALIGN_DURATION:
                            print("进入状态51")
                            vision_node.angle_0 = 0
                            state_id = 51
                            continue
                        
                        yaw_speed = np.clip(angle_error * KP + delta * KD, -MAX_YAW, MAX_YAW)
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.01, 0.0, yaw_speed]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print(f"角度误差: {np.degrees(angle_error):+.1f}\u00B0 | 持续: {duration:.4f}s | yaw: {yaw_speed:.2f}")
                        last_cmd_time = current_time
                    else:
                        msg.mode = 11
                        msg.gait_id = 3
                        msg.vel_des = [0.1, 0.0, SEARCH_SPEED]
                        msg.rpy_des = [0, 2.5, 0]
                        msg.life_count = (msg.life_count + 1) % 128
                        Ctrl.Send_cmd(msg)
                        # print("搜索黄线")
                        last_cmd_time = current_time

            elif state_id == 51:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.05, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._51_detected:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.1, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.1 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.01, 0.0, -1.4]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 1.22 )
                    print("进入状态53")
                    state_id = 53
            
            elif state_id == 52:
                if current_time - last_cmd_time > cmd_interval:
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [-0.2, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    last_cmd_time = current_time

                if vision_node._52_detected:
                    
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0, 0]
                    msg.rpy_des = [0, 2.5, 0]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 0.2 )
                    msg.mode = 11
                    msg.gait_id = 3
                    msg.vel_des = [0.3, 0.0, 2.25]
                    msg.life_count = (msg.life_count + 1) % 128
                    Ctrl.Send_cmd(msg)
                    time.sleep( 2.1 )
                    print("进入状态53")
                    break
                    

            elif state_id == 53:
                msg.mode = 11
                msg.gait_id = 10
                msg.vel_des = [-0.3, 0, 0]
                msg.rpy_des = [0, 2.5, 0]
                msg.life_count = (msg.life_count + 1) % 128
                Ctrl.Send_cmd(msg)
                time.sleep( 3.5 )
                down()
                while(True):
                    continue
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
    
# coding: utf-8
