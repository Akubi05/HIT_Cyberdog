# rgb相机demo

- Source: `document\官方文档\rgb相机demo.pdf`
- Pages: 3

## Page 1

rgb相机demo
1.修改gazebo.xacro
在gazebo.xacro中添加并保存
PowerShell
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
<p2>0.0</p2>
<center>0.50.5</center>
</distortion>
</camera>
<pluginname="rgb_camera_plugin"
filename="libgazebo_ros_camera.so">
<ros>
<!--<namespace>stereo</namespace>-->
<remapping>~/image_raw:=image_raw</remapping>
<remapping>~/camera_info:=camera_info</remapping>
</ros>
<!--Setcameraname.Ifempty,defaultstosensor
name(i.e."sensor_name")-->
<camera_name>rgb_camera</camera_name>
<!--SetTFframename.Ifempty,defaultstolink
name(i.e."link_name")-->
<frame_name>RGB_camera_link</frame_name>

## Page 2

<hack_baseline>0.2</hack_baseline>
</plugin>
</sensor>
</gazebo>
2.运行仿真程序
运行仿真程序后可通过window->TopicVisualization中找到对应topic并打开,可确认
rgb相机正常运行
通过ros2echotopic可确认topic正常发送
3.rviz可视化
在rivz2中通过以下设置可将topic可视化

## Page 3

[No extractable text]
