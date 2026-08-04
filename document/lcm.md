# 测试lcm通信

mintest

```
# 发送端（默认配置）
python3 mintest.py send

# 接收端（默认配置）
python3 mintest.py recv

# 自定义端口和频道(可选)
python3 mintest.py send --port 7668 --chan MY_CHANNEL

# 如果没有ex文件夹,运行
lcm-gen -p example.lcm
```

# final环境(电脑)

```
pip install lcm --break-system-packages
pip install toml --break-system-packages
pip install pyzbar --break-system-packages
sudo apt update
sudo apt install -y libzbar0
```

# lcm所有
.
./lcmtesta
./lcmtesta/example.lcm
./lcmtesta/ex
./lcmtesta/ex/__pycache__
./lcmtesta/ex/__pycache__/__init__.cpython-313.pyc
./lcmtesta/ex/__pycache__/example_t.cpython-313.pyc
./lcmtesta/ex/__init__.py
./lcmtesta/ex/example_t.py
./lcmtesta/mintest.py
./computer_dog
./computer_dog/dog_cmd_recv.py
./computer_dog/pc_cmd_send.py
./computer_dog/final_198_merge1.py

# 让计算机发送命令让机器狗接受并执行
final需要的文件:
color_config.py,
CtrlBase.py,
NodeBase.py,
robot_control_cmd_lcmt.py,
robot_control_response_lcmt.py
pc_cmd_send.py
狗运行dog_cmd_recv.py

window运行代码:
```
netsh interface ipv4 show interfaces
route add 239.0.0.0 mask 255.0.0.0 0.0.0.0 if 19
```
19是网卡序号

linux代码

```
# 假设你的局域网网卡名称是 wlp0s20f3
sudo ip route replace 239.255.76.66/32 dev wlp0s20f3
```

机器狗上运行
```
# 假设你的局域网网卡名称是 wlan0,有线网卡eth0
echo 123 | sudo -S ip route replace 239.255.76.66/32 dev wlan0
echo 123 | sudo -S ip route replace 239.255.76.67/32 dev eth0
```

然后检查：

```
ip route get 239.255.76.66
ip route get 239.255.76.67
```

期望结果：
```
239.255.76.66 dev wlan0
239.255.76.67 dev eth0
```

另外注意：route add/ip route replace 这种改法重启后会丢，需要重启后重新执行，或者后面再做成开机脚本。


网络示意

现在有两段网络，.66 和 .67 在这个 LCM 转发方案里分别代表两边的组播域：
```
  PC <---- wlan0 / 172.20.10.x ----> 机器狗 NX <---- eth0 / 192.168.44.x ----> 运控板
          用 239.255.76.66                         用 239.255.76.67
```
所以：239.255.76.66 -> wlan0

用于 PC 和机器狗 NX 之间通信：

- PC 发 robot_control_cmd 到 239.255.76.66:7667
- NX 的 dog_cmd_recv.py 监听 .66
- NX 把 global_to_robot、robot_control_response 转发回 .66:7670
- PC 的测试脚本从 .66:7670 收位姿和反馈

而：239.255.76.67 -> eth0

用于 NX 和内部运控板之间通信：

- NX 把 PC 的控制命令转发到 239.255.76.67:7671
- NX 从 239.255.76.67:7670 收运控反馈
- NX 从 239.255.76.67:7667 收 global_to_robot

不能让 .66 走 eth0，否则 PC 收不到反馈；也不能让 .67 走 wlan0，否则 NX 收不到/发不到内部运控板。

一句话：.66 是外网 PC 侧，走 wlan0；.67 是狗内部运控侧，走 eth0。

