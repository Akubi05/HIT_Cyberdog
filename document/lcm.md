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

# 让计算机发送命令让机器狗接受并执行
final需要的文件:
color_config.py,
CtrlBase.py,
NodeBase.py,
robot_control_cmd_lcmt.py,
robot_control_response_lcmt.py
pc_cmd_send.py
狗运行dog_cmd_recv.py

(网卡相关只运行一次就行,不需要写到bashrc)

window运行代码:
```
netsh interface ipv4 show interfaces
route add 239.0.0.0 mask 255.0.0.0 0.0.0.0 if 19
```
19是网卡序号

linux代码

```
# 假设你的局域网网卡名称是 wlp0s20f3
sudo route add -net 239.0.0.0 netmask 255.0.0.0 dev wlp0s20f3
```

机器狗上运行
```
# 假设你的局域网网卡名称是 wlan0,有线网卡eth0
sudo route add -net 239.0.0.0 netmask 255.0.0.0 dev wlan0
sudo route add -host 239.255.76.67 dev eth0
```