import lcm
import time
import threading
from robot_control_cmd_lcmt import robot_control_cmd_lcmt

class PC_cmd_send:
    def __init__(self):
        # ========================================================
        # 【核心修复】完全照搬 mintest.py 的配置
        # 1. 显式指定 udpm:// (组播协议)
        # 2. 显式指定 IP 和 端口 (239.255.76.67:7667)
        # 3. 显式指定 ttl=1 (允许跨设备传输)
        # ========================================================
        lcm_url = "udpm://239.255.76.66:7667?ttl=1"
        
        try:
            self.lc = lcm.LCM(lcm_url)
            print(f">>> [PC] LCM 初始化成功: {lcm_url}")
        except RuntimeError as e:
            print(f"!!! LCM 初始化失败: {e}")
            return

        self.cmd_channel = "robot_control_cmd"
        
        # 初始化消息对象
        self.current_msg = robot_control_cmd_lcmt()
        self._init_msg()
        
        self.running = False
        self.lock = threading.Lock()

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
            t = threading.Thread(target=self._loop, daemon=True)
            t.start()
            print(">>> [PC] 后台发送线程已启动 (50Hz)")

    def _loop(self):
        while self.running:
            with self.lock:
                # 1. 心跳自增
                self.current_msg.life_count = (self.current_msg.life_count + 1) % 128
                
                try:
                    # 2. 发送数据
                    self.lc.publish(self.cmd_channel, self.current_msg.encode())
                except Exception as e:
                    print(f"发送报错: {e}")
            
            # 保持 50Hz
            time.sleep(0.02)

    def Send_cmd(self, msg):
        """更新要发送的指令"""
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

    def stop(self):
        self.running = False