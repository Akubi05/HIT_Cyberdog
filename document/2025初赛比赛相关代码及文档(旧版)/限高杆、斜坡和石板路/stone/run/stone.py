# position: (0.011694, 5.980531, 0.054167, 1.568894)

import lcm
import sys
import time
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.executors import MultiThreadedExecutor

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

    def Wait_finish(self, mode, gait_id, time_limit = 10):
        count = 0
        while self.runing and count < 200 * time_limit: # 200t = 1s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1
                if( count % 200 == 0):
                    print(f"running for {count / 200} s")

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

def standup():  # 站立
    msg.mode = 12  # Recovery stand
    msg.gait_id = 0
    msg.life_count += 1  # Command will take effect when life_count update
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(12, 0, 10)

def walk(duration = 5):  # 走几步

    msg.mode = 11
    msg.gait_id = 27
    msg.vel_des = [0.2, 0, 0]
    msg.life_count += 1
    msg.duration = duration * 1000  # ms
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 27, duration)

def squat():        # 蹲下
    msg.mode = 7
    msg.gait_id = 1
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(7, 1, 5)

def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        rclpy.shutdown()

def quick_step(vel, step, duration):
    msg.mode = 11
    msg.gait_id = 3
    msg.vel_des = [vel, 0, 0.0]
    msg.rpy_des = [0.0, -0.01, 0.0]
    msg.step_height = [step, step]
    msg.duration = duration * 1000  # ms
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 3, duration)


def stone_road(duration):  # 石板路
    quick_step(0.235, 0.06, duration)

Ctrl = Robot_Ctrl()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()


executor = MultiThreadedExecutor()

# 添加节点到执行器

spin_thread = Thread(target=spin_executor)
spin_thread.start()


def main():
    try:
        standup()  # 站立
        # print("恢复站立")
        time.sleep(0.5)

        print("开始...")

        stone_road(30)  # 石板路


        print("结束...") 
        standup()  # 站立
        
    except KeyboardInterrupt:
        standup()  # 站立
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        Ctrl.quit()
        sys.exit()


if __name__ == '__main__':
    main()

