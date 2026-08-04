import time
from threading import Lock, Thread

import lcm

from assets import robot_control_cmd_lcmt, robot_control_response_lcmt


# 中文注释：底层 LCM 控制通道封装，维护命令发送、反馈接收和安全退出。
class Robot_Ctrl(object):
    # 中文注释：初始化 LCM 发布/订阅、共享命令对象和控制反馈状态。
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

    # 中文注释：启动控制发送线程和反馈接收线程。
    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    # 中文注释：处理底层控制反馈消息并更新状态快照。
    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode
            self.gait_ok = self.rec_msg.gait_id
        else:
            self.mode_ok = 0
            self.gait_ok = 0

    # 中文注释：循环接收 LCM 反馈，直到退出事件触发。
    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    # 中文注释：等待指定 mode/gait 的动作反馈完成或超时。
    def Wait_finish(self, mode, gait_id, time_count=None):
        start_time = time.time()
        timeout_sec = None if time_count is None else float(time_count)
        while self.runing and (self.app_state is None or not self.app_state.exit_requested):
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            now = time.time()
            if timeout_sec is not None and now - start_time >= timeout_sec:
                return False
            time.sleep(0.005)
        return False

    # 中文注释：线程安全地复制当前底层控制状态。
    def get_status_snapshot(self):
        msg = self.rec_msg
        return {
            'mode': int(getattr(msg, 'mode', 0)),
            'gait_id': int(getattr(msg, 'gait_id', 0)),
            'order_process_bar': int(getattr(msg, 'order_process_bar', 0)),
            'switch_status': int(getattr(msg, 'switch_status', 0)),
            'ori_error': int(getattr(msg, 'ori_error', 0)),
            'footpos_error': int(getattr(msg, 'footpos_error', 0)),
        }

    # 中文注释：按 delay_cnt 持续重发最近一条命令，保障底层接收。
    def send_publish(self):
        while self.runing:
            with self.send_lock:
                if self.delay_cnt > 20:
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
            time.sleep(0.005)

    # 中文注释：提交新的控制命令并触发一段时间的重复发布。
    def Send_cmd(self, msg):
        with self.send_lock:
            self.delay_cnt = 50
            self.cmd_msg = msg

    # 中文注释：请求退出并清空持续发布计数。
    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()
