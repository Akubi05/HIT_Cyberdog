import lcm
import os
import time
import threading

from assets import robot_control_cmd_lcmt

DEFAULT_SEND_INTERVAL_SEC = float(os.environ.get("PC_CMD_SEND_INTERVAL_SEC", "0.30"))


class PC_cmd_send:
    def __init__(self, lcm_url=None, cmd_channel=None, send_interval=None):
        # 中文注释：默认走真机 PC 侧固定组播地址。
        self.lcm_url = lcm_url or 'udpm://239.255.76.66:7667?ttl=1'
        self.cmd_channel = cmd_channel or 'robot_control_cmd'
        self.send_interval = float(
            DEFAULT_SEND_INTERVAL_SEC if send_interval is None else send_interval
        )
        if self.send_interval <= 0.0:
            raise ValueError("send_interval 必须大于 0")
        self.lc = None
        self.running = False
        self.lock = threading.Lock()
        self._thread = None

        try:
            self.lc = lcm.LCM(self.lcm_url)
            print(f">>> [PC] LCM 初始化成功: {self.lcm_url}")
        except RuntimeError as e:
            print(f"!!! LCM 初始化失败: {e}")
            raise

        # 初始化消息对象
        self.current_msg = robot_control_cmd_lcmt()
        self._init_msg()

    def _init_msg(self):
        """初始化所有字段，防止空值导致 encode 崩溃"""
        self.current_msg.mode = 0
        self.current_msg.gait_id = 0
        self.current_msg.contact = 0
        self.current_msg.life_count = 0
        self.current_msg.value = 0
        self.current_msg.duration = 0
        self.current_msg.vel_des = [0.0] * 3
        self.current_msg.rpy_des = [0.0] * 3
        self.current_msg.pos_des = [0.0] * 3
        self.current_msg.acc_des = [0.0] * 6
        self.current_msg.ctrl_point = [0.0] * 3
        self.current_msg.foot_pose = [0.0] * 6
        self.current_msg.step_height = [0.0] * 2

    def run(self):
        """启动后台发送线程"""
        if not self.running:
            self.running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            print(f">>> [PC] 后台发送线程已启动 ({1.0 / self.send_interval:.1f}Hz)")

    def _loop(self):
        while self.running:
            with self.lock:
                self._publish_locked()
            
            time.sleep(self.send_interval)

    def _publish_locked(self):
        self.current_msg.life_count = (self.current_msg.life_count + 1) % 128
        try:
            self.lc.publish(self.cmd_channel, self.current_msg.encode())
        except Exception as e:
            print(f"发送报错: {e}")

    def Send_cmd(self, msg, immediate=False):
        """更新待发送指令；停走等紧急指令可要求立即发布。"""
        with self.lock:
            self.current_msg.mode = msg.mode
            self.current_msg.gait_id = msg.gait_id
            self.current_msg.contact = msg.contact
            self.current_msg.value = msg.value
            self.current_msg.duration = msg.duration
            self.current_msg.vel_des = list(msg.vel_des)
            self.current_msg.rpy_des = list(msg.rpy_des)
            self.current_msg.pos_des = list(msg.pos_des)
            self.current_msg.step_height = list(msg.step_height)
            self.current_msg.acc_des = list(msg.acc_des)
            self.current_msg.ctrl_point = list(msg.ctrl_point)
            self.current_msg.foot_pose = list(msg.foot_pose)
            if immediate:
                self._publish_locked()

    def stop(self):
        self.running = False

    def quit(self):
        self.stop()
