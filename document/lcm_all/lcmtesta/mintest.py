import lcm
import time
import sys
import argparse

# 确保生成的 ex.example_t 类在路径中
try:
    from ex.example_t import example_t
except ImportError:
    print("[ERROR] 找不到生成的 ex.example_t 模块。请先运行: lcm-gen -p example_t.lcm")
    sys.exit(1)

# 简化配置：只使用局域网组播
MULTICAST_ADDR = "239.255.76.67"  # 局域网组播地址
DEFAULT_PORT = 7667
DEFAULT_CHANNEL = "EXAMPLE_CHANNEL"
TTL = 1  # TTL=1 限制在局域网内

def run_node(mode, port, channel):
    """
    运行 LCM 节点 - 局域网组播模式
    
    Args:
        mode: 'send' 或 'recv'
        port: 端口号
        channel: LCM 频道名称
    """
    # 构建组播 URL
    lcm_url = f"udpm://{MULTICAST_ADDR}:{port}?ttl={TTL}"
    
    try:
        lc = lcm.LCM(lcm_url)
        print(f"[INFO] LCM 组播初始化成功")
        print(f"[INFO] 模式: {mode} | 组播地址: {MULTICAST_ADDR}:{port} | 频道: {channel}")
    except RuntimeError as e:
        print(f"[ERROR] LCM 初始化失败: {e}")
        print(f"[TIP] 请检查防火墙是否开放端口 {port}")
        sys.exit(1)

    if mode == "send":
        # 发送端
        msg = example_t()
        msg.name = "Multicast_Sender"
        msg.enabled = True
        msg.position = [1.0, 2.0, 3.0]
        
        print(f"[INFO] 开始组播发送...")
        try:
            while True:
                msg.timestamp = int(time.time() * 1000)
                lc.publish(channel, msg.encode())
                print(f"[SEND] Timestamp: {msg.timestamp}")
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[INFO] 发送端已停止")

    else:
        # 接收端
        def message_handler(channel, data):
            try:
                msg_rx = example_t.decode(data)
                print(f"[RECV] 频道: {channel} | 名字: {msg_rx.name} | 时间戳: {msg_rx.timestamp}")
            except Exception as e:
                print(f"[ERROR] 解码失败: {e}")

        lc.subscribe(channel, message_handler)
        print(f"[INFO] 正在监听组播消息...")
        try:
            while True:
                lc.handle_timeout(100)
        except KeyboardInterrupt:
            print("\n[INFO] 接收端已停止")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="LCM 局域网组播通信脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 发送端
  python3 mintest.py send
  
  # 接收端
  python3 mintest.py recv
  
  # 自定义端口和频道
  python3 mintest.py send --port 7668 --chan MY_CHANNEL
        """
    )
    parser.add_argument('mode', choices=['send', 'recv'], 
                        help='运行模式: send 或 recv')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, 
                        help=f'端口号 (默认: {DEFAULT_PORT})')
    parser.add_argument('--chan', type=str, default=DEFAULT_CHANNEL, 
                        help=f'通信频道 (默认: {DEFAULT_CHANNEL})')

    args = parser.parse_args()
    run_node(args.mode, args.port, args.chan)