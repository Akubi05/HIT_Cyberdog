# 仿真使用cyberdog_race说明文档

- Source: `document\官方文档\仿真使用cyberdog_race说明文档.pdf`
- Pages: 15

## Page 1

cyberdog_race说明文档
1.环境搭建
1.1推荐环境配置
•Ubuntu20.04
•Docker20.10.21（安装教程：https://docs.docker.com/engine/install/ubuntu/）
1.2docker镜像导入&运行
1.下载docker包cyberdog_race.tar
2.&本地导入Docker镜像
PlainText
sudodockerload-icyberdog_race.tar
3.授权XServer
PlainText
xhost+
4.运行Docker镜像
Bash
sudodockerrun-it--shm-size="1g"--privileged=true-e
DISPLAY=$DISPLAY-v/tmp/.X11-unix:/tmp/.X11-unixcyberdog_sim:v1
2.仿真环境使用
仿真环境基于gazebo仿真器，令仿真器程序直接与控制程序cyberdog_control进行
通信，并将机器人的各关节数据与传感器数据转发为ros2topic，并通过rviz进行机器
人状态的可视化。
2.1运行仿真环境

## Page 2

2.1.1脚本运行整个仿真环境
在打开docker后，在终端运行
Shell
cd/home/cyberdog_sim
python3src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
运行后的仿真器共有2个界面：gazebo界面为仿真器界面，通过该界面可以确认机器
人与环境的交互；rviz2为可视化界面，主要显示机器人自身的状态与传感器返回的数
据。
rviz2可视化界面（左）&gazebo仿真器界面（右）
运行仿真器后弹出三个窗口：cyerdog_gazebo为仿真器程序的界面；
cyberdog_control为控制程序的界面；cyberdog_visual为rviz2可视化程序的界面。
各界面会返回对应程序的状态以及发生的错误。

## Page 3

程序界面
2.2.2导入镜像到打开仿真器的全流程视频
[tutorial.mp4]
2.1.3分别运行各个程序
除脚本启动外，可通过以下步骤分别运行各程序
Gazebo仿真程序
首先启动gazebo程序，于cyberdog_sim文件夹下进行如下操作：
Plaintext
$source/opt/ros/galactic/setup.bash
$sourceinstall/setup.bash
$ros2launchcyberdog_gazeborace_gazebo.launch.py
也可通过如下命令打开gazebo仿真中的激光雷达传感器
Plaintext
$source/opt/ros/galactic/setup.bash
$sourceinstall/setup.bash
$ros2launchcyberdog_gazeborace_gazebo.launch.py
use_lidar:=true
注意：若启动Gazebo仿真程序时出现如下错误，是由于gazebo程序没有彻底杀

## Page 4

死导致。可通过在docker终端运行killall-9gazebo&killall-9gzserver&killall-9
gzclient指令彻底杀死进程后，重新运行仿真程序。
对于gazebo进程无法通过ctrl+C彻底杀死的问题，可以使用如下脚本运行gazebo，
运行时在检测到ctrl+C后该脚本会自动杀死所有gazebo的进程。在cyberdog_sim文
件夹下运行：
Plaintext
$source/opt/ros/galactic/setup.bash
$sourceinstall/setup.bash
$chmod+x
src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py
$python3
src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py
ros2launchcyberdog_gazebogazebo.launch.pyCopyto
clipboardErrorCopied
cyberdog控制程序
然后启动cyberdog_locomotion的控制程序。打开一个新的终端，在cyberdog_sim文
件夹下运行：
Plaintext
$source/opt/ros/galactic/setup.bash
$sourceinstall/setup.bash
$ros2launchcyberdog_gazebocyberdog_control_launch.py
rviz可视化界面
最后打开可视化界面，打开一个新的终端，在cyberdog_sim文件夹下运行：
Plaintext
$source/opt/ros/galactic/setup.bash
$sourceinstall/setup.bash
$ros2launchcyberdog_visualcyberdog_visual.launch.py
2.2仿真环境通信结构
与仿真环境中的机器人建立通信需要使用到lcm与ROS2topic两种通信方式。

## Page 5

他们的相关文档见：
lcm通信：cyberdogblog运动控制模块1.2节
lcmgithub库
ROS2topic：ros2topic官方文档
2.2.1通信结构介绍
整个仿真环境的通信结构如下图所示
•cyberdog控制程序和gazebo之间通过sharedmemory进行通信，gazebo程序
创建host的共享内存，控制程序通过attach到该内存上进行通信。其通信的内容为
robotToSim/simToRobot。
•ros2仿真界面接受从控制程序通过lcm发送的电机和里程计信号等信号并通过
topic转发为/joint_states与/tf。
•gazebo仿真程序会将仿真中的imu与激光雷达的数据以ros2topic的形式进行发
送，其topic名称为/imu和/scan
2.2.2机器人控制接口
lcm通信
在仿真中，控制程序的高层接口会也能够使用，可以通过lcm通信向控制程序发送指
令，控制仿真中的机器人。具体的功能与实现方法可参照运控文档的运动控制模块。
ROS2topic通信
在仿真中可以运行motionmanager，该程序能够提供一个ROS2topic的接口，能够
将ROS2topic的控制指令转换为lcm指令发送给控制程序，该部分的具体使用方法在
后续会进行说明。

## Page 6

2.2.3机器人状态及传感器接口
仿真中机器人数据会以lcm和ROS2topic的方式对外进行发送，可根据自身的需求选
择抓取数据的方式。
lcm数据的抓取
lcm数据，可以在控制程序/home/cyberdog_sim/src/cyberdog_locomotion/script目录
下使用lcm自带的lcm-logger抓取数据，然后使用第三方库log2smat将抓取的数据转
换为matlab的mat文件。具体使用方法如下：
Plaintext
$cd/home/cyberdog_sim/src/cyberdog_locomotion/script
$./make_types.sh#安装后首次使用需运行该脚本以生成lcm的type文件
$lcm-logger#可通过Crtl+C退出数据抓取，会自动生成lcm的log文件。
$./log_convert<生成的log文件名><要生成的mat文件名.mat>
也可在script文件夹下运行launch_lcm_spy.sh脚本实时显示lcm数据。
Lcm数据实时监测界面
仿真程序发送的lcm数据simulator_state结构如下simulator_lcmt.lcm

## Page 7

Plaintext
structsimulator_lcmt{
doublevb[3]; //机身坐标系下xyz速度
doublerpy[3]; //翻滚角俯仰角偏航角
int64_ttimesteps;//实际时间戳
doubletime; //
doublequat[4]; //机身朝向四元数
doubleR[9]; //机身朝向旋转矩阵
doubleomegab[3]; //机身坐标系下角速度
doubleomega[3]; //机身角速度
doublep[3]; //机身位置
doublev[3]; //机身速度
doublevbd[3]; //机身加速度
doubleq[12]; //关节位置
doubleqd[12]; //关节速度
doubleqdd[12]; //关节加速度
doubletau[12]; //关节输出转矩
doubletauAct[12];//关节
doublef_foot[12];//足端接触力xyz方向分量
doublep_foot[12];//足端位置xyz坐标
}
其中未提及坐标的数据皆为世界坐标系下的数据。
关节相关数据的顺序为FR-侧摆髋关节FR-前摆髋关节FR-膝关节FL-侧摆髋关节FL-
前摆髋关节FL-膝关节RR-侧摆髋关节RR-前摆髋关节RR-膝关节RL-侧摆髋关节RL-
前摆髋关节RL-膝关节。
足端相关数据的顺序为FRFLRRRL。
ROS2topic数据
仿真平台会以标准的ROS2topic/tf2(里程计&各link坐标转换)/joint_states(各关节的
状态)/imu(imu加速度数据)进行发送具体可参照：
sensor_msgs文档
tf2官方文档
相应的在gazebo中通过官方插件加入的传感器也能够通过ROStopic进行发送，该
部分将在下一节进行具体说明。
2.3传感器加入与使用

## Page 8

仿真模型中，对应实际机器的传感器位置设置了对应的link，可通过直接在对应的link
上添加对应的传感器插件来在仿真中使用不同的传感器。
仿真模型中对应传感器的link&
目前link上附上的传感器插件
各传感器link位置与实机相对应
目前使用传感器插件有两个途径：Gazebo官方支持传感插件；或通过github获取一
些gazebo的第三方插件
该节将以机器人颈部的激光雷达为例，介绍在仿真中如何加入和使用传感器插件。
2.3.1在机器人模型中加入传感器插件
首先，需要在
/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/x
acro文件夹下的机器人描述文件中的对应link上增加传感器。
该xacro文件与urdf（机器人描述文件）的资料见：
xacrofilewiki
Urdfwiki
可以通过robot.xacro确认模型中的传感器link，并在gazebo.xacro中进行传感器的创
建。
•找到robot.xacro需要增加的传感器对应的link

## Page 9

•在增加激光雷达传感器，在在gazebo.xacro的<robot>下增加如下代码：
XML
<gazeboreference="lidar_link"> <!--传感器所在
link名称-->
<sensorname="realsense"type="ray"> <!--传感器的参数
设置-->
<always_on>true</always_on>
<visualize>true</visualize>
<pose>0.000.0000</pose>
<update_rate>5</update_rate>
<ray>
<scan>
<horizontal>
<samples>180</samples>
<resolution>1.000000</resolution>
<min_angle>-1.5700</min_angle>
<max_angle>1.5700</max_angle>
</horizontal>
</scan>

## Page 10

<range>
<min>0.01</min>
<max>12.00</max>
<resolution>0.015000</resolution>
</range>
<noise>
<type>gaussian</type>
<mean>0.0</mean>
<stddev>0.01</stddev>
</noise>
</ray>
<pluginname="cyberdog_laserscan"
filename="libgazebo_ros_ray_sensor.so"><!--传感器插件的库，可通过官
网资料查询-->
<ros>
<remapping>~/out:=scan</remapping> <!--
传感器返回数据topic的名称-->
</ros>
<output_type>sensor_msgs/LaserScan</output_type><!--
传感器返回数据topic的类型，可查阅官方资料-->
<frame_name>lidar_link</frame_name> <!--
传感器所在link名称-->
</plugin>
</sensor>
</gazebo>
•保存后启动仿真器确认传感器是否生效

## Page 11

此处设置的激光雷达与实机激光雷达位置一致，且会进行正前方180°，12m范围的扫
描。
2.3.2确认传感器的数据&可视化
确认传感器topic正确发送
官方传感器插件会根据在gazebo.xacro中设置的<remapping>发送相同名称的topic，
且各位为<output_type>中的内容。
因此，可通过在docker终端中输入ros2topiclist指令查看当前的ROS2topic是否存
在/scantopic，并通过ros2topicecho/scan打印出topic的内容。

## Page 12

ros2topiclist
激光雷达传感器的topic数据打印
topic中各数据含义可通过查询wiki进行查询。
激光雷达的topic类型sensor_msgs/LaserScan格式如下：
至此，完成传感器在仿真环境中的导入，并能够通过建立ROS2Node接受和处理传
感器返回的数据，实现对传感器的利用。
传感器数据的可视化

## Page 13

rviz2可视化界面能够支持大部分gazebo传感器数据的显示。
以激光雷达为例，在确认激光雷达topic正常发送后，能够通过rviz2可视化激光雷达
返回的数据，步骤如下：
•运行仿真程序，并打开可视化界面，点击左侧display下方的Add按钮，打开
add界面，点击Bytopic选项卡。
•此时，可确认所有正在通信的topic，若传感器对应topic正常发送且rviz2能够进
行可视化，则会出现彩色的选项，双击打开对应选项后，在display界面出现对应的下
拉菜单。

## Page 14

•打开下拉菜单，将ReliabilityPolicy改为BestEffort后，rviz就能够可视化激光
雷达扫描得到的点云。
2.4世界模型的创建
通过设置仿真器世界文件，可以实现仿真环境改动，以方便调试。
仿真场景的文件存放在docker的
/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/world目录下，场景

## Page 15

文件后缀为.world，使用sdf(SimulationDescriptionFormat)格式。
目前仿真器默认打开race.world场景，可通过修改或创建新的场景文件来设计需要的
仿真场景。
当设计完成后，可通过更改
/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/launch目录下的
race_gazebo.launch.py文件。只需将wname部分的名称改成对应场景.world文件的
名称并保存。此时运行仿真程序即可加载对应场景。
更改wnamedefault_value为对应名称
单独加载某段赛道
