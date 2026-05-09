# Cyberdog视觉功能

- Source: `document\官方文档\Cyberdog视觉功能.pdf`
- Pages: 8

## Page 1

Cyberdog视觉功能
计算机视觉(ComputerVision)是一种利用计算机科学和数学方法处理、分析图像和
视频等视觉数据的技术。在本次大赛中，计算机视觉可以为机器人提供感知能力，
帮助其规划最优路径，避免障碍物并迅速穿越赛道。
Cyberdog2配备多个摄像头，涵盖了不同方向和视角，以全方位地捕获周围环境的
图像。参赛队需要设计和实现系列算法，使机器人能够在图像中检测到不同类型的
障碍物，识别后做出相应的动作调整。
一、相机模块介绍
相机驱动模块使用ArgusAPI提供的接口来操控MIPI相机硬件并实时捕获图像，
使用ROS2提供的接口来管理相机节点，为外部模块提供交互接口。
模块整体架构图如下：

## Page 2

模块内部描述图如下：
1.安装依赖项
编译本模块需要依赖若干外部软件包，编译前需按照下列命令安装：
1.nvidia-l4t-jetson-multimedia-api
sudoapt-getinstallnvidia-l4t-jetson-multimedia-api
2.cuda-toolkit
sudoapt-getinstallcuda-toolkit-10-2
3.libavformat-dev
sudoapt-getinstalllibavformat-dev

## Page 3

2.运行测试程序
测试程序地址：
https://github.com/MiRoboticsLab/cyberdog_ros2/tree/main/cyberdog_interaction/cyberd
og_camera/cyberdog_camera/src/camera_test
基于相机api的相机测试程序，可以用来测试相机是否正常，亦可以作为camera
api使用方式参考。
编译：
colconbuild--merge-install--packages-up-tocamera_test
运行：
./build/camera_test/camera_testcam_idwidthheightrgb/bgr
例如，测试camera0，640x480分辨率RGB出图的话，使用如下命令：
./build/camera_test/camera_test0640480rgb
二、RGB相机Demo
1.修改gazebo.xacro
在gazebo.xacro中添加并保存
<gazeboreference="RGB_camera_link">
<sensortype="camera"name="rgbcamera">
<always_on>true</always_on>
<update_rate>15.0</update_rate>
<cameraname="rgb_camera">
<horizontal_fov>1.46608</horizontal_fov>
<image>
<width>320</width>
<height>180</height>
<format>R8G8B8</format>
</image>
<distortion>
<k1>0.0</k1>
<k2>0.0</k2>
<k3>0.0</k3>
<p1>0.0</p1>

## Page 4

<p2>0.0</p2>
<center>0.50.5</center>
</distortion>
</camera>
<pluginname="rgb_camera_plugin"filename="libgazebo_ros_camera.so">
<ros>
<!--<namespace>stereo</namespace>-->
<remapping>~/image_raw:=image_raw</remapping>
<remapping>~/camera_info:=camera_info</remapping>
</ros>
<!--Setcameraname.Ifempty,defaultstosensorname(i.e."sensor_name")-->
<camera_name>rgb_camera</camera_name>
<!--SetTFframename.Ifempty,defaultstolinkname(i.e."link_name")-->
<frame_name>RGB_camera_link</frame_name>
<hack_baseline>0.2</hack_baseline>
</plugin>
</sensor>
</gazebo>
2.运行仿真程序
运行仿真程序后可通过window选项下的TopicVisualization中找到对应topic并打
开,可确认RGB相机正常运行
通过ros2echotopic可确认topic正常发送

## Page 5

3.rviz可视化
在rivz2中通过以下设置可将topic可视化
三、相机服务接口
CameraService.srv配置
uint8SET_PARAMETERS=0
​
uint8TAKE_PICTURE=1
​
uint8START_RECORDING=2
​
uint8STOP_RECORDING=3
​
uint8GET_STATE=4
​
uint8DELETE_FILE=5
​
uint8GET_ALL_FILES=6
​
uint8START_LIVE_STREAM=7

## Page 6

​
uint8STOP_LIVE_STREAM=8
​
uint8START_IMAGE_PUBLISH=9
​
uint8STOP_IMAGE_PUBLISH=10
​
​
​
uint8command
​
#commandarguments
​
stringargs
​
uint16width
​
uint16height
​
uint16fps
​
---
​
uint8RESULT_SUCCESS=0
​
uint8RESULT_INVALID_ARGS=1
​
uint8RESULT_UNSUPPORTED=2
​
uint8RESULT_TIMEOUT=3
​
uint8RESULT_BUSY=4
​
uint8RESULT_INVALID_STATE=5
​
uint8RESULT_INNER_ERROR=6
​
uint8RESULT_UNDEFINED_ERROR=255
​
​
​
uint8result
​
stringmsg
​
int32code

## Page 7

RGB鱼眼相机
#打开、关闭camera
ros2launchcamera_teststereo_camera.py
#查看namespace
ros2nodelist
###如果是开机自启，注意topic前加上命名空间
ros2lifecycleset/stereo_cameraconfigure
ros2lifecycleset/stereo_cameraactivate
ros2lifecycleset/stereo_cameradeactivate
ros2lifecycleset/stereo_cameracleanupCopytoclipboardErrorCopied
获取图像
接口形式：rostopic
接口名称：camera_server
topic名称：
#使用topic订阅rgb相机、左鱼眼相机，右鱼眼相机
#左鱼眼
/image_left
#右鱼眼

## Page 8

/image_right
#rgb相机
/image_rgbCopytoclipboardErrorCopied
图像数据topic内容:sensor_msgs::msg::Image
AI相机
#打开、关闭camera
ros2runcamera_testcamera_server
##开启AI相机指令
ros2servicecall/camera_serviceprotocol/srv/CameraService"{command:9,width:640,
height:480,fps:0}"
##关闭相机指令
ros2servicecall/camera_serviceprotocol/srv/CameraService"{command:10,args:
''}"CopytoclipboardErrorCopied
获取图像
接口形式：rostopic
接口名称：camera_server
topic名称：
#使用topic订阅AI相机
#AI相机topic
topic:/imageCopytoclipboardErrorCopied
图像数据topic内容:sensor_msgs::msg::Image
更多教程可参看官网开发者手册。
