import lcm
import sys
import os
import time
from threading import Thread, Lock
import signal
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

from CtrlBase import Robot_Ctrl,State
from NodeBase import RGBCameraNode,LidarNode,Color,Direction,DepthCameraNode
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
#from testing import Testing


#Ctrl INIT
Ctrl = Robot_Ctrl()
Ctrl.run()

#rosNode INIT
rclpy.init()
rgb_camera_node = RGBCameraNode("rgb_camera_node")
lidar_node = LidarNode("laser_lidar_node")
depth_camera_node = DepthCameraNode("depth_camera_node")
# 添加节点到执行器
executor = MultiThreadedExecutor()
executor.add_node(rgb_camera_node)
executor.add_node(lidar_node)
executor.add_node(depth_camera_node)
def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        executor.shutdown()
        rclpy.shutdown()

#spin所有节点
spin_thread = Thread(target=spin_executor)
spin_thread.start()


def TaskYellowLight(vel=0.5, step=0.05, duration=2000):
    """黄灯接近控制逻辑"""
    BIA = 0.27576-(0.164)
    while Ctrl.runing and rgb_camera_node.is_node_running:
        temp = rgb_camera_node.get_distance_YellowLight()
        current_distance =  temp + BIA
        print(f"[黄灯控制] 当前距离: {current_distance:.2f}m")
        
        #just for test
        # time.sleep(6000)
        # break

        if current_distance >= 2.0+BIA:
            print("执行前进1m")
            Ctrl.go_short(vel, step, duration)  # 前进1m
        elif 1.5+BIA <= current_distance < 2.0+BIA:
            print("执行前进0.5m")
            Ctrl.go_short(vel, step, duration//2)  # 前进0.5m
        elif 0.8+BIA <= current_distance < 1.5+BIA:
            move_dist = current_distance - 0.5 #(0.5+BIA2)
            print(f"执行前进{move_dist:.2f}m")
            adjusted_duration = int(duration * (move_dist/1.0))
            Ctrl.go_short(vel, step, adjusted_duration)
            break  # 结束检测循环
        else:
            print("error")


def TaskLimitHeight(vel=0.5, step=0.05, duration=2000):
    """限高杆接近控制逻辑"""
    while Ctrl.runing and rgb_camera_node.is_node_running:
        current_distance = rgb_camera_node.get_distance_LimitHeight() 
        print(f"[限高杆控制] 当前距离: {current_distance:.2f}m")
        
        #just for test
        # time.sleep(6000)
        # break

        if current_distance >= 2.0:
            print("执行前进1m")
            Ctrl.go_short(vel, step, duration)
        elif 1.5<= current_distance < 2.0:
            print("执行前进0.5m")
            Ctrl.go_short(vel, step, duration//2)   # 前进0.5m
        elif 0.8<= current_distance < 1.5:
            move_dist = current_distance - 0.5 #(0.5)
            print(f"执行前进{move_dist:.2f}m")
            adjusted_duration = int(duration * (move_dist/1.0))
            Ctrl.go_short(vel, step, adjusted_duration)
            break  # 结束检测循环
        else:
            print("error")


def main():
    try:
        Ctrl.standup()
        #任务开始进行
        #Testing(Ctrl,msg,lidar_node)
        
        rgb_camera_node.is_node_running = True
        # Task_GreenAndRed(Ctrl,msg,rgb_camera_node,lidar_node,flag)
        #TaskYellowLight()
        TaskLimitHeight()

    except KeyboardInterrupt:
        pass
    finally:
        print("程序结束")
        Ctrl.quit()
        executor.shutdown()
        rclpy.shutdown()
        spin_thread.join()
        #sys.exit()



main()