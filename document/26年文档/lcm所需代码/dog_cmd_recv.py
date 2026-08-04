import lcm
import os
import subprocess
import time
import threading
from assets import robot_control_cmd_lcmt
from robot_ctrl import Robot_Ctrl


HEARTBEAT_INTERVAL_SEC = float(os.environ.get("DOG_RELAY_HEARTBEAT_INTERVAL_SEC", "0.10"))
STATUS_PRINT_INTERVAL_SEC = 1.0
PC_LCM_URL = "udpm://239.255.76.66:7667?ttl=1"
PC_RESPONSE_LCM_URL = "udpm://239.255.76.66:7670?ttl=1"
ROBOT_RESPONSE_LCM_URL = "udpm://239.255.76.67:7670?ttl=255"
ROBOT_POSE_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
PC_CHANNEL = "robot_control_cmd"
ROBOT_CHANNEL = "robot_control_cmd"
RESPONSE_CHANNEL = "robot_control_response"
POSE_CHANNEL = "global_to_robot"
PC_MULTICAST_GROUP = "239.255.76.66"
PC_MULTICAST_IFACE = os.environ.get("PC_MULTICAST_IFACE", "wlan0")


def _zero_vec(length):
    return [0.0 for _ in range(length)]


class DogRelay:
    def __init__(self):
        check_pc_multicast_route()

        # 1. 启动控制线程
        self.Ctrl = Robot_Ctrl()
        self.Ctrl.rec_thread.daemon = True
        self.Ctrl.send_thread.daemon = True
        self.Ctrl.run() 
        print(">>> 机器人控制线程已启动")

        # 2. 初始化本地缓存的指令 (默认为趴下，防止失控)
        self.msg_lock = threading.Lock()
        self.current_msg = robot_control_cmd_lcmt()
        self.current_msg.mode = 0 
        self.current_msg.gait_id = 0
        self.current_msg.life_count = 0
        self.sent_count = 0
        self.response_forward_count = 0
        self.last_status_print_time = 0.0
        
        # 3. 启动一个【本地心跳线程】来专门喂狗
        # 这就是区别！我们不依赖 PC 的频率，我们在本地通过 USB 网卡稳定喂狗
        self.keep_alive_thread = threading.Thread(target=self.heartbeat_loop)
        self.keep_alive_thread.daemon = True
        self.keep_alive_thread.start()
        print(
            ">>> 狗端底层命令心跳已启动: "
            f"interval={HEARTBEAT_INTERVAL_SEC:.3f}s, "
            f"freq={1.0 / HEARTBEAT_INTERVAL_SEC:.1f}Hz"
        )

        # 4. 初始化接收 PC 命令，以及把狗侧反馈转回 PC 侧
        self.lc = lcm.LCM(PC_LCM_URL) # 注意确认端口
        self.lc.subscribe(PC_CHANNEL, self.pc_callback)
        self.robot_response_lc = lcm.LCM(ROBOT_RESPONSE_LCM_URL)
        self.robot_response_lc.subscribe(RESPONSE_CHANNEL, self.robot_response_callback)
        self.pc_response_lc = lcm.LCM(PC_RESPONSE_LCM_URL)
        self.response_forward_thread = threading.Thread(target=self.response_forward_loop)
        self.response_forward_thread.daemon = True
        self.response_forward_thread.start()
        self.robot_pose_lc = lcm.LCM(ROBOT_POSE_LCM_URL)
        self.robot_pose_lc.subscribe(POSE_CHANNEL, self.robot_pose_callback)
        self.pose_forward_thread = threading.Thread(target=self.pose_forward_loop)
        self.pose_forward_thread.daemon = True
        self.pose_forward_thread.start()
        print(">>> 正在监听 PC 命令 (转发模式)...")

    def normalize_command(self, msg):
        """
        PC 只发 mode 或发了不完整参数时，给底盘补一组安全默认值。
        完整速度/姿态命令仍然会保留 PC 发来的字段。
        """
        mode = int(msg.mode)
        msg.mode = mode
        msg.gait_id = int(msg.gait_id)
        msg.contact = int(msg.contact)
        msg.value = int(msg.value)
        msg.duration = int(msg.duration)
        msg.vel_des = list(msg.vel_des)
        msg.rpy_des = list(msg.rpy_des)
        msg.pos_des = list(msg.pos_des)
        msg.acc_des = list(msg.acc_des)
        msg.ctrl_point = list(msg.ctrl_point)
        msg.foot_pose = list(msg.foot_pose)
        msg.step_height = list(msg.step_height)

        if mode == 0:
            msg.gait_id = 0
            msg.duration = 0
            msg.vel_des = _zero_vec(3)
        elif mode == 7:
            # PUREDAMPER: duration=0 或 >3000ms。短 duration 可能吞掉后续动作。
            if msg.gait_id not in (0, 1):
                msg.gait_id = 0
            if 0 < msg.duration <= 3000:
                msg.duration = 0
            msg.vel_des = _zero_vec(3)
        elif mode == 12:
            # RECOVERY_STAND: duration=0 或 >6000ms。
            msg.gait_id = 0
            if 0 < msg.duration <= 6000:
                msg.duration = 0
            msg.vel_des = _zero_vec(3)
            if len(msg.rpy_des) >= 3:
                msg.rpy_des = [0.0, 0.0, 0.0]
        elif mode == 11:
            # LOCOMOTION 需要 gait_id。PC 没给时默认零速站立 gait=1。
            if msg.gait_id == 0:
                msg.gait_id = 1
            if len(msg.step_height) >= 2 and msg.step_height[0] == 0.0 and msg.step_height[1] == 0.0:
                # 中文注释：零速保持时允许 PC 显式发送 step_height=0，用于停止原地踏步。
                # 只有仍有速度/yaw 命令时，才把“未设置步高”的默认值补成 6cm。
                vel_nonzero = any(abs(float(v)) > 1e-4 for v in msg.vel_des[:3])
                if vel_nonzero:
                    msg.step_height = [0.06, 0.06]
            if msg.duration < 0:
                msg.duration = 0

        return msg

    @staticmethod
    def copy_command(msg):
        snapshot = robot_control_cmd_lcmt()
        snapshot.mode = int(msg.mode)
        snapshot.gait_id = int(msg.gait_id)
        snapshot.contact = int(msg.contact)
        snapshot.life_count = int(msg.life_count)
        snapshot.vel_des = list(msg.vel_des)
        snapshot.rpy_des = list(msg.rpy_des)
        snapshot.pos_des = list(msg.pos_des)
        snapshot.acc_des = list(msg.acc_des)
        snapshot.ctrl_point = list(msg.ctrl_point)
        snapshot.foot_pose = list(msg.foot_pose)
        snapshot.step_height = list(msg.step_height)
        snapshot.value = int(msg.value)
        snapshot.duration = int(msg.duration)
        return snapshot

    def pc_callback(self, channel, data):
        try:
            # A. 收到 PC 指令，只更新参数，不直接转发
            # 这样即使 PC 发得慢，或者只发一次，本地的心跳线程也会一直维持这个状态
            incoming_msg = self.normalize_command(robot_control_cmd_lcmt.decode(data))
            
            # 更新本地缓存的意图 (Mode, Velocity, etc.)
            with self.msg_lock:
                self.current_msg.mode = incoming_msg.mode
                self.current_msg.gait_id = incoming_msg.gait_id
                self.current_msg.contact = incoming_msg.contact
                self.current_msg.vel_des = list(incoming_msg.vel_des)
                self.current_msg.rpy_des = list(incoming_msg.rpy_des)
                self.current_msg.pos_des = list(incoming_msg.pos_des)
                self.current_msg.acc_des = list(incoming_msg.acc_des)
                self.current_msg.ctrl_point = list(incoming_msg.ctrl_point)
                self.current_msg.foot_pose = list(incoming_msg.foot_pose)
                self.current_msg.step_height = list(incoming_msg.step_height)
                self.current_msg.value = incoming_msg.value
                self.current_msg.duration = incoming_msg.duration
                # 注意：不要用 PC 的 life_count，用本地的

            print(
                "收到 PC 指令 -> "
                f"mode={incoming_msg.mode}, gait={incoming_msg.gait_id}, "
                f"vel={list(incoming_msg.vel_des)}, duration={incoming_msg.duration}"
            )

        except Exception as e:
            print(f"解码错误: {e}")

    def publish_msg(self, msg):
        with self.Ctrl.send_lock:
            self.Ctrl.cmd_msg = msg
            self.Ctrl.delay_cnt = 50
            self.Ctrl.lc_s.publish(ROBOT_CHANNEL, msg.encode())
            self.sent_count += 1

    def robot_response_callback(self, channel, data):
        self.pc_response_lc.publish(RESPONSE_CHANNEL, data)
        self.response_forward_count += 1

    def robot_pose_callback(self, channel, data):
        self.pc_response_lc.publish(POSE_CHANNEL, data)

    def response_forward_loop(self):
        while self.Ctrl.runing:
            self.robot_response_lc.handle_timeout(100)

    def pose_forward_loop(self):
        while self.Ctrl.runing:
            self.robot_pose_lc.handle_timeout(100)

    def print_status_if_due(self):
        now = time.time()
        if now - self.last_status_print_time < STATUS_PRINT_INTERVAL_SEC:
            return
        self.last_status_print_time = now
        status = self.Ctrl.get_status_snapshot()
        with self.msg_lock:
            target = (
                int(self.current_msg.mode),
                int(self.current_msg.gait_id),
                list(self.current_msg.vel_des),
                int(self.current_msg.duration),
                int(self.current_msg.life_count),
            )
        print(
            "转发状态 -> "
            f"sent={self.sent_count}, target mode/gait={target[0]}/{target[1]}, "
            f"resp_fwd={self.response_forward_count}, "
            f"vel={target[2]}, duration={target[3]}, life={target[4]}, "
            f"feedback mode/gait={status['mode']}/{status['gait_id']}, "
            f"bar={status['order_process_bar']}, switch={status['switch_status']}, "
            f"ori_err={status['ori_error']}, foot_err={status['footpos_error']}"
        )

    def heartbeat_loop(self):
        """
        这个线程负责按固定频率向狗发送指令，并自动处理 life_count。
        默认 10Hz，约为原 50Hz 的 1/5；可用 DOG_RELAY_HEARTBEAT_INTERVAL_SEC 覆盖。
        """
        while True:
            # 1. 自动自增 life_count (核心关键！)
            with self.msg_lock:
                self.current_msg.life_count = (self.current_msg.life_count + 1) % 128
                msg_snapshot = self.copy_command(self.current_msg)

            # 2. 发送给底层控制库，并立刻 publish 一包，方便调试链路
            self.publish_msg(msg_snapshot)
            
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    def run(self):
        try:
            while True:
                self.lc.handle_timeout(100)
                self.print_status_if_due()
        except KeyboardInterrupt:
            pass
        finally:
            self.Ctrl.runing = 0


def check_pc_multicast_route():
    try:
        result = subprocess.run(
            ["ip", "route", "get", PC_MULTICAST_GROUP],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except Exception as exc:
        print(f"!!! 无法检查 PC 侧组播路由: {exc}")
        return

    route_text = result.stdout.strip()
    expected = f"dev {PC_MULTICAST_IFACE}"
    if result.returncode == 0 and expected in route_text:
        print(f">>> PC 侧组播路由正常: {PC_MULTICAST_GROUP} -> {PC_MULTICAST_IFACE}")
        return

    print(
        "!!! PC 侧组播路由可能走错网卡，PC 可能收不到位姿/控制反馈。\n"
        f"!!! 当前: {route_text or 'unknown'}\n"
        f"!!! 修复: echo 123 | sudo -S ip route replace {PC_MULTICAST_GROUP}/32 dev {PC_MULTICAST_IFACE}"
    )


if __name__ == "__main__":
    relay = DogRelay()
    relay.run()
