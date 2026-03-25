#!/usr/bin/env python3
"""
Cyberdog 简单控制演示
展示如何通过LCM发送控制指令来控制仿真中的机器狗
"""

import sys
import lcm
import time

# 添加LCM类型文件路径
sys.path.append('/home/daoshi/projects/sim/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


# ============== 控制标志定义 ==============
# 运动模式 (MotionMode)
MODE_OFF = 0
MODE_QP_STAND = 3          # QP站立
MODE_PURE_DAMPER = 7
MODE_LIFTED = 9
MODE_LOCOMOTION = 11       # 运动模式(踏步)
MODE_RECOVERY_STAND = 12   # 恢复站立
MODE_MOTION = 62           # 动作模式

# 步态ID (GaitId) - 部分常用
GAIT_STAND = 1             # 站立
GAIT_PRONK = 2             # 原地蹦跳
GAIT_TROT_USER = 3         # 用户小跑
GAIT_WALK = 6              # 行走
GAIT_TROT_10_4 = 5         # 飞翔小跑
GAIT_USER_00 = 80          # 用户自定义步态00
GAIT_USER_01 = 81          # 用户自定义步态01
GAIT_USER_02 = 82          # 用户自定义步态02


class CyberdogController:
    def __init__(self):
        """初始化LCM通信"""
        self.lc = lcm.LCM()
        self.channel = "exec_request"
        self.response = None

        # 订阅响应消息
        self.lc.subscribe("exec_response", self._response_handler)

    def _response_handler(self, channel, data):
        """处理机器狗的响应"""
        msg = robot_control_response_lcmt.decode(data)
        self.response = msg
        print(f"[响应] 模式: {msg.mode}, 步态: {msg.gait_id}, 状态: {msg.switch_status}")

    def send_cmd(self, mode, gait_id=0, vel_des=None, rpy_des=None, duration=1000):
        """
        发送控制指令

        Args:
            mode: 运动模式
            gait_id: 步态ID
            vel_des: 期望速度 [vx, vy, vyaw]
            rpy_des: 期望姿态 [roll, pitch, yaw]
            duration: 持续时间(ms)
        """
        msg = robot_control_cmd_lcmt()

        # 基本控制参数
        msg.mode = mode
        msg.gait_id = gait_id
        msg.contact = 0
        msg.life_count = 0

        # 速度 (x: 前后, y: 左右, z: 转向)
        if vel_des is None:
            vel_des = [0.0, 0.0, 0.0]
        msg.vel_des = vel_des

        # 姿态 (roll, pitch, yaw)
        if rpy_des is None:
            rpy_des = [0.0, 0.0, 0.0]
        msg.rpy_des = rpy_des

        # 位置
        msg.pos_des = [0.0, 0.0, 0.0]

        # 加速度
        msg.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 其他参数
        msg.ctrl_point = [0.0, 0.0, 0.0]
        msg.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.step_height = [0.0, 0.0]
        msg.value = 0
        msg.duration = duration

        # 发送消息
        self.lc.publish(self.channel, msg.encode())
        print(f"[发送] 模式: {mode}, 步态: {gait_id}, 速度: {vel_des}")

    def stand_up(self):
        """让机器狗站起来"""
        print("=== 执行: 站立 ===")
        self.send_cmd(mode=MODE_RECOVERY_STAND, gait_id=GAIT_STAND, duration=2000)
        time.sleep(2.5)

    def start_locomotion(self):
        """进入运动模式(原地踏步)"""
        print("=== 执行: 进入运动模式 ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER, duration=2000)
        time.sleep(2.5)

    def move_forward(self, speed=0.3):
        """向前移动"""
        print(f"=== 执行: 向前移动 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[speed, 0.0, 0.0], duration=500)
        time.sleep(0.6)

    def move_backward(self, speed=0.3):
        """向后移动"""
        print(f"=== 执行: 向后移动 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[-speed, 0.0, 0.0], duration=500)
        time.sleep(0.6)

    def move_left(self, speed=0.2):
        """向左移动"""
        print(f"=== 执行: 向左移动 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[0.0, speed, 0.0], duration=500)
        time.sleep(0.6)

    def move_right(self, speed=0.2):
        """向右移动"""
        print(f"=== 执行: 向右移动 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[0.0, -speed, 0.0], duration=500)
        time.sleep(0.6)

    def turn_left(self, speed=0.5):
        """左转"""
        print(f"=== 执行: 左转 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[0.0, 0.0, speed], duration=500)
        time.sleep(0.6)

    def turn_right(self, speed=0.5):
        """右转"""
        print(f"=== 执行: 右转 (速度: {speed}) ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_USER,
                     vel_des=[0.0, 0.0, -speed], duration=500)
        time.sleep(0.6)

    def stop(self):
        """停止运动"""
        print("=== 执行: 停止 ===")
        self.send_cmd(mode=MODE_LOCOMOTION, gait_id=GAIT_STAND,
                     vel_des=[0.0, 0.0, 0.0], duration=500)
        time.sleep(1.0)


def main():
    """主函数演示"""
    controller = CyberdogController()

    print("\n" + "="*50)
    print("Cyberdog 运动控制演示")
    print("="*50)
    print("\n注意: 运行此脚本前，请确保:")
    print("1. 仿真已启动 (launchsim.py)")
    print("2. use_rc 参数已设置为 0 (退出遥控模式)")
    print("3. ROS环境已激活\n")

    # 演示序列
    try:
        # 1. 先站起来
        controller.stand_up()

        # 2. 进入运动模式
        controller.start_locomotion()

        # 3. 前进几步
        for _ in range(3):
            controller.move_forward(0.3)

        # 4. 停止
        controller.stop()

        print("\n演示完成!")

    except KeyboardInterrupt:
        print("\n程序被用户中断")
        controller.stop()


if __name__ == "__main__":
    main()
