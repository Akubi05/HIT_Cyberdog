# 狗配网不要用app直接用typec线

app 不好使,你直接用type线 连接狗的download口

```
ssh mi@192.168.55.1

sudo nmcli dev wifi rescan

sudo nmcli dev wifi list

sudo nmcli dev wifi connect "WiFi名称" password "WiFi密码"
e.g. 
sudo nmcli dev wifi connect "zxp1" password "12345678"
```
成功会显示：
```
Device 'wlan0' successfully activated with 'xxxx-xxxx-xxxx'

```
```
ip route 
```
查看狗的ip

```
mi@mi-desktop:~$ ip route
default via 192.168.94.224 dev wlan0 proto dhcp metric 600 
default via 192.168.55.100 dev l4tbr0 metric 32766 
169.254.0.0/16 dev docker0 scope link metric 1000 linkdown 
172.17.0.0/16 dev docker0 proto kernel scope link src 172.17.0.1 linkdown 
192.168.44.0/24 dev eth0 proto kernel scope link src 192.168.44.1 
192.168.55.0/24 dev l4tbr0 proto kernel scope link src 192.168.55.1 
224.0.0.0/4 dev eth0 scope link 
```

狗ip 192.168.94.56

