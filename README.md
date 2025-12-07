# fnos_ubuntu2004
1. 这个仓库主要同步fnos_ubuntu2004虚拟机中的代码，因为虚拟机存在崩溃的可能性。
2. 这个虚拟主要是为了wheeltec机器人开发而创建。




3. 机器人基础信息：
   1. 机器人已经自动连接上了家里局域网WiFi因此可以通过SSH远程登录机器人上位机主控：ip:192.168.1.231 用户名：wheeltec 密码：dongguan
   2. 机器人上位机ubuntu版本：18.04，ROS版本：melodic
   3. 机器人上位机主控主要文件路径：
      1. /home/wheeltec/wyb: 
         wyb   //存放自己单独测试的ros包：
         └── work_space
            └── src
                  ├── 
                  └── ros_astra_camera //奥比中光相机ros包
      2. /home/wheeltec/wheeltec_robot //机器人导航包含的功能包
         wheeltec_robot
         ├── build
         ├── devel
         ├── src
         │   ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
         │   ├── depthimage_to_laserscan-melodic-devel
         │   ├── fdilink_ahrs
         │   ├── imu_tf_broadcaster
         │   ├── ipa_exploration
         │   ├── kcf_track
         │   ├── navigation-melodic
         │   ├── robot_pose_ekf
         │   ├── ros_astra_camera
         │   ├── ROS常用功能命令6.0.txt
         │   ├── rrt_exploration
         │   ├── sh_manager
         │   ├── simple_follower
         │   ├── slam_karto
         │   ├── teb_local_planner-melodic-devel
         │   ├── tts_make
         │   ├── turn_on_wheeltec_robot
         │   ├── usb_cam
         │   ├── web_video_server
         │   ├── wheeltec_joy_control
         │   ├── wheeltec_lidar
         │   ├── wheeltec_multi
         │   ├── wheeltec_robot_rc
         │   ├── world_canvas_msgs
         │   ├── xf_mic_asr_offline_circle
         │   └── yesense_imu
         └── src.zip


