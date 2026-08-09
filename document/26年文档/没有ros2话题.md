
  1. ros2 topic list 默认用了 ROS2 daemon，当前 daemon 缓存/发现不正常，所以只显示：

  /parameter_events
  /rosout

  2. 用 --no-daemon 可以看到完整 ROS graph：

  source /etc/mi/ros2_env.conf
  ros2 topic list --no-daemon

  我刚查到狗上这样能看到大量节点和 topic。

  但相机 topic 当前状态是：

  /mi_desktop_48_b0_2d_7b_05_3e/image

  这个 topic 存在，但：

  Publisher count: 0
  Subscription count: 1

  也就是 AI 图像 topic 现在没有发布者。并且这些当前不存在：

  /mi_desktop_48_b0_2d_7b_05_3e/image_rgb
  /mi_desktop_48_b0_2d_7b_05_3e/image_left
  /mi_desktop_48_b0_2d_7b_05_3e/image_right

  你可以先用这个命令确认：

  source /etc/mi/ros2_env.conf
  ros2 topic list --no-daemon | grep image

  如果想让普通 ros2 topic list 也刷新，试：

  source /etc/mi/ros2_env.conf
  ros2 daemon stop
  ros2 daemon start
  ros2 topic list

  不过相机 ROS topic 没发布者的问题，重启 daemon 不会解决；那需要重新启动/激活相机节点。之前我能取到四路图，是因为用了 nvarguscamerasrc
  sensor-id=0..3 直接抓 Argus，不依赖 ROS topic。
