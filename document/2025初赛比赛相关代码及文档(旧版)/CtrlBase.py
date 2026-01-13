import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

from enum import Enum, auto
class State(Enum):
    Left = 0
    Right = 1
    Middle = 2
    Unknown = 3

#控制Base,需要用import就行
class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)#开启两个线程
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = False
        #self.state = State.Left
        self.is_motion_over = True
        self.msg = robot_control_cmd_lcmt()


    def run(self):
        self.runing = True
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if(self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
            #self.gait_ok = self.rec_msg.gait_id
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.002 )

    def Wait_finish_time(self, mode, gait_id, wait_time):
        count = 0
        sum = 50 * wait_time
        while self.runing and count < sum:
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.02)
                count += 1

    
    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20: # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd",self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep( 0.005 )

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.is_motion_over = False#发送运动指令
        self.send_lock.release()

    def standup(self):  # 站立
        self.msg.mode = 12  # Recovery stand
        self.msg.gait_id = 0
        self.msg.life_count = (self.msg.life_count + 1) % 128  # Command will take effect when life_count update
        self.Send_cmd(self.msg)
        self.Wait_finish_time(self.msg.mode,self.msg.gait_id,8)

    def go_short(self, vel, step, duration):  # 前进
        self.msg.mode = 11
        self.msg.gait_id = 27
        self.msg.vel_des = [vel, 0, 0]
        self.msg.step_height = [step, step]
        self.msg.duration = duration
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.Send_cmd(self.msg)
        self.Wait_finish_time(self.msg.mode,self.msg.gait_id,5)
    
    def walk(self, vel, yaw, duration = 0,rpy = 0):  # 走
        self.msg.mode = 11
        self.msg.gait_id = 10
        self.msg.vel_des = [vel, 0, yaw]
        self.msg.rpy_des = [0, rpy, 0]
        self.msg.step_height = [0.06, 0.06]
        # self.msg.duration = duration
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.Send_cmd(self.msg)
        # self.Wait_finish_time(self.msg.mode,self.msg.gait_id,5)
        time.sleep(duration)

    def stand(self, rpy):
        self.msg.mode = 21
        self.msg.gait_id = 0
        self.msg.vel_des = [0, 0, 0]
        self.msg.rpy_des = [0, rpy, 0]
        self.msg.life_count = (self.msg.life_count + 1) % 128
        self.Send_cmd(self.msg)

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()
    

if __name__ == "__main__":
    Ctrl = Robot_Ctrl()