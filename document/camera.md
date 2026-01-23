94号狗

如果相机命令执行失败,可以在每个命令前面执行 ros2 daemon stop

ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/camera/camera configure

ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/camera/camera activate

ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/stereo_camera configure

ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/stereo_camera activate


ros2 launch realsense2_camera on_dog.py
ros2 lifecycle set /camera/camera configure
ros2 lifecycle set /camera/camera activate
ros2 lifecycle set /camera/camera deactivate
ros2 lifecycle set /camera/camera cleanup

ros2 run camera_test stereo_camera
ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate
ros2 lifecycle set /stereo_camera deactivate
ros2 lifecycle set /camera/camera cleanup

ros2 run camera_test camera_server
ros2 service call /camera_service protocol/srv/CameraService "{command: 9, width: 640, height: 480, fps: 0}"

179
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_01_60/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_01_60/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_01_60/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_01_60/stereo_camera activate

198
ros2 lifecycle set /mi_desktop_48_b0_2d_7a_ff_a8/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7a_ff_a8/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_7a_ff_a8/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7a_ff_a8/stereo_camera activate

13
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_05_3e/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_05_3e/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_05_3e/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_7b_05_3e/stereo_camera activate

19
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_be_5c/stereo_camera activate

205
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_bb_e3/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_bb_e3/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_bb_e3/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_5f_bb_e3/stereo_camera activate

23
ros2 lifecycle set /mi_desktop_48_b0_2d_60_12_4a/camera/camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_60_12_4a/camera/camera activate
ros2 lifecycle set /mi_desktop_48_b0_2d_60_12_4a/stereo_camera configure
ros2 lifecycle set /mi_desktop_48_b0_2d_60_12_4a/stereo_camera activate

