import lcm
import time
import threading
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from CtrlBase import Robot_Ctrl 

class DogRelay:
    def __init__(self):
        # 1. 启动控制线程
        self.Ctrl = Robot_Ctrl()
        self.Ctrl.run() 
        print(">>> 机器人控制线程已启动")

        # 2. 初始化本地缓存的指令 (默认为趴下，防止失控)
        self.current_msg = robot_control_cmd_lcmt()
        self.current_msg.mode = 0 
        self.current_msg.gait_id = 0
        self.current_msg.life_count = 0
        
        # 3. 启动一个【本地心跳线程】来专门喂狗
        # 这就是区别！我们不依赖 PC 的频率，我们在本地通过 USB 网卡稳定喂狗
        self.keep_alive_thread = threading.Thread(target=self.heartbeat_loop)
        self.keep_alive_thread.daemon = True
        self.keep_alive_thread.start()

        # 4. 初始化接收 PC 命令
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1") # 注意确认端口
        self.lc.subscribe("robot_control_cmd", self.pc_callback)
        print(">>> 正在监听 PC 命令 (转发模式)...")

    def pc_callback(self, channel, data):
        try:
            # A. 收到 PC 指令，只更新参数，不直接转发
            # 这样即使 PC 发得慢，或者只发一次，本地的心跳线程也会一直维持这个状态
            incoming_msg = robot_control_cmd_lcmt.decode(data)
            
            # 更新本地缓存的意图 (Mode, Velocity, etc.)
            self.current_msg.mode = incoming_msg.mode
            self.current_msg.gait_id = incoming_msg.gait_id
            self.current_msg.vel_des = incoming_msg.vel_des
            self.current_msg.rpy_des = incoming_msg.rpy_des
            self.current_msg.pos_des = incoming_msg.pos_des
            # 注意：不要用 PC 的 life_count，用本地的

            print(f"收到 PC 指令 -> 更新目标 Mode: {self.current_msg.mode}")

        except Exception as e:
            print(f"解码错误: {e}")

    def heartbeat_loop(self):
        """
        这个线程负责以 50Hz 的频率向狗发送指令，并自动处理 life_count
        """
        while True:
            # 1. 自动自增 life_count (核心关键！)
            self.current_msg.life_count = (self.current_msg.life_count + 1) % 128
            
            # 2. 发送给底层控制库
            self.Ctrl.Send_cmd(self.current_msg)
            
            # 3. 保持 20-50Hz 的频率
            time.sleep(0.02)

    def run(self):
        while True:
            try:
                self.lc.handle()
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    relay = DogRelay()
    relay.run()