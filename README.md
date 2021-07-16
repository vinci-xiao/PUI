# PUI on ROS2

Distributuon
---
https://docs.ros.org/en/foxy/Releases.html

Install 
---
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/) 
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)

Reference 
---
- [ROS2 目錄](https://www.guyuehome.com/25194)
- [ROS2 cheats sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)
- [ROS Kinetic和Ubuntu 16.04 LTS的支持即将结束：如何减轻影响？](https://cn.ubuntu.com/blog/ros-kinetic-and-ubuntu-16-04-eol)
- [在Ubuntu 20.04中安装ROS2最新版本Foxy Fitzroy](https://blog.csdn.net/feimeng116/article/details/106602562)
- [ROS2-Foxy安装教程
](https://blog.csdn.net/qq825255961/article/details/106949261)
- [古月居-Why ROS2?](https://mp.weixin.qq.com/s?__biz=MzU1NjEwMTY0Mw==&mid=2247485880&idx=1&sn=9c7293998e169c033b1346ff21746759&chksm=fbcb70dcccbcf9ca03f7b98d6eec76192cda1f554a87b5dc089613ca5b26d4796b7e4692789d&scene=126&sessionid=1583412112&key=31fb57371192e55612920a04dba61df0b419722fe0dcef4b187ac24ef262b0532b0f267ffaab159608cc5566fd048abd9c45c67963e6930b4d5487ac5132a5a50fa10309ffa7fae71065258f47e9d908&ascene=1&uin=MTk5ODYzNDU%3D&devicetype=Windows+10&version=62080079&lang=zh_CN&exportkey=AWwDnb%2F7qd7feFXtZ7eMWOs%3D&pass_ticket=EkZ7OwOfdKdlb16kpJzipqs%2BvnXpyjJ%2BIZaegUUgbWo%3D古月居)
- [古月居-ROS2探索总结](https://www.guyuehome.com/blog/index/category/12)
- [古月居-ROS2 DDOS explaination](https://www.guyuehome.com/805)
- [Udemy-ROS2 For Beginners (ROS Foxy-2021)](https://www.udemy.com/course/ros2-for-beginners/)
- [Turtlebot3 Introducing ROS2 Tutorials](https://discourse.ros.org/t/tb3-introducing-ros2-tutorials/5959)
- [ROS2机器人高效编程](https://class.guyuehome.com/detail/p_5e5f85df06a80_6Ga4qV9h/6)
- [ROBOTICS- turtlebot3 e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [ROS2 Foxy tutorial](https://docs.ros.org/en/foxy/Tutorials.html)
- [Migrating launch files from ROS 1 to ROS 2](https://docs.ros.org/en/foxy/Guides/Launch-files-migration-guide.html)
- [How to Setup VSCode for ROS](https://www.youtube.com/watch?v=RXyFSnjMd7M)
- [Migration guide from ROS 1](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html?highlight=map#launch-files)
- [採智科技機器人產品中文線上手冊翻譯整理](https://hackmd.io/@idminer/usermanual-tw/https%3A%2F%2Fhackmd.io%2FyhleSV1CQhmTJCMkFci80w)
- [gitbook](https://zlargon.gitbooks.io/git-tutorial/content/startup/commit_a_patch.html)

Imigration Journal from ROS1 to ROS2
---
#### 1. pui_teleop pkg requirement
- joystick_drivers `sudo apt install ros-foxy-joy`
- [teleop_twist_joy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/) `sudo apt install ros-foxy-teleop-twist-joy`
- TODO: 
    可以新增鍵盤遙控 teleop_twist_keyboard `sudo apt install ros-foxy-teleop-twist-keyboard`    
> Note:
> [在索引中新增arguement:launch file](https://answers.ros.org/question/339484/how-to-solve-file-rviz2launchpy-was-not-found-in-the-share-directory-of-package/)
#### 2. pui_msg pkg 
- [Official tutorial: Custom-ROS2-Interfaces](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
- [ROS2 embedding a msg as a field in a custom msg](https://answers.ros.org/question/314724/ros2-embedding-a-msg-as-a-field-in-a-custom-msg/)
> Tips: 
> 確認自定義msg建立 `ros2 interface show pui_msgs/msg/MultiRange `
#### 3. pui_uwb
- Install Eigen library `sudo apt install libeigen3-dev`
- [Role of including customed msg](https://www.programmersought.com/article/83463964985/)
> Example of publishing topic thro cmd 
> `ros2 topic pub /topic std_msgs/msg/String "data: nono" `
> `ros2 topic pub /uwb_range geometry_msgs/msg/Pose "{position: {x: 1,y: 2, z: 3}}"`
> `ros2 topic pub /uwb_range pui_msgs/msg/MultiRange "{min_range: 6,max_range: 87}"`

> TODO
> - [x] subscribe customed msg
> - [x] ADD Eigen lib
> - [x] Minimum Square Estimation
> - [x] Output result
> - [ ] bug: position error
> - [ ] initial position
> - [ ] Get parameters node

#### 4. pui_markers
- [ros2 marker](https://github.com/ros2/common_interfaces/tree/master/visualization_msgs)
- [ros2 time-node](https://answers.ros.org/question/287946/ros-2-time-handling/)
> TODO
> - [x] tag_marker.cpp (merge into uwb_xinyi.cpp)
> - [x] tag_path.cpp
> - [x] ros::duration migrate to ros2
> - [x] clean code

#### 5. pui_bringup
- Dynamixel (pui_drive)
    - [Dynamixel Wizard 2.0 (not support Ubuntu 20.04 yet)](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
    - [Dynamixel Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository)
    - [DYNAMIXEL Quick Start Guide for ROS 2](https://www.youtube.com/watch?v=E8XPqDjof4U)
    - [DynamixelSDK 3.7.40 (Support ROS 2 Only)](https://github.com/ROBOTIS-GIT/DynamixelSDK/releases/tag/3.7.40)
    - [dynamixel-workbench-msgs](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs/tree/ros2-devel)
    - [XL430-W250-T](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
    - `ros2 topic pub -1 /set_velocity dynamixel_sdk_custom_interfaces/msg/SetVelocity "{id: 1, velocity: 0}"`
    - ![](https://i.imgur.com/tw1U9Yr.png)
        `ros2 launch pui_teleop pui_teleop_launch.py`
        `ros2 run dynamixel_sdk_examples xl430_controll_node`
> Dynamixel (pui_drive) TODO
> - [x] Speed controller.cpp
> - [x] mapping cmd_vel to speed controll
- rviz2 (pui_overlapping)
    - [參考nav2_bringup rviz_launch.py](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/bringup/launch/rviz_launch.py)
    - [Nav2源码阅读（一）nav2_bringup tb3_simulation_launch.py](https://blog.csdn.net/weixin_41680653/article/details/117449074)
- map_server (pui_overlapping)
    - [NAV2 install](https://navigation.ros.org/getting_started/index.html)
    - [NAV2- map server](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
    - [managed life cycle](https://design.ros2.org/articles/node_lifecycle.html)
    - [Map not showing in RVIZ2 when running with autostart](https://github.com/ros-planning/navigation2/issues/867)
    - [no map received](https://get-help.robotigniteacademy.com/t/rviz-no-map-received/4721)
    - [NAV2 -Map Server/Saver](https://navigation.ros.org/configuration/packages/configuring-map-server.html?highlight=map_server)
    - `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: ~/pui_ws/src/pui_bringup/maps/hector_fixed.yaml}"`
    - `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: ~/pui_ws/src/pui_bringup/params/pui_params.yaml}"`
    - [turtlebot3_navigation2](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel/turtlebot3_navigation2)

- transform (pui_overlapping)
    - `ros2 run tf2_ros static_transform_publisher 1.52 1.68 0 0 0 -1.66 world map`
    - [ROS2入门教程-发布joint states和TF](https://www.ncnynl.com/archives/201801/2257.html)
    - [Using tf2 with ROS 2
](https://docs.ros.org/en/foxy/Tutorials/tf2.html)

- velodyne
    - `sudo apt install ros-foxy-velodyne-*`
    - [ros-drive velodyne](https://github.com/ros-drivers/velodyne/tree/ros2)
    - http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
    - Network setting
        ![](https://i.imgur.com/aj3OITA.png)
    - Check conection
        ![](https://i.imgur.com/JNXYD5I.png)
    - View on rviz2 (Fixed fram= velodyne)
        ![](https://i.imgur.com/tIPKSW1.png)

- ros2 bag (pui_overlapping)
    - [Ros2bag](https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)
    - `sudo apt install ros-foxy-rosbag2`
    - `ros2 bag record --all -o name`
- ros-serial arduino node
    - [micro-ROS arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy)
    - [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
    - [micro_ros_setup-foxy](https://github.com/micro-ROS/micro_ros_setup/tree/foxy)
    - [ROS2与arduino入门教程-安装Teensyduino](https://www.ncnynl.com/archives/202012/3989.html)
        `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6`
    - [開啟arduino_agent後,ros_node找不到時](https://github.com/micro-ROS/micro_ros_arduino/issues/223)
        > 註解掉ros_domain_id=30
    - [Teensy Hardware serial](https://www.pjrc.com/teensy/td_uart.html)
    - ![](https://i.imgur.com/F08KNdI.png)
    - [Adding a new package to the build system](https://github.com/micro-ROS/micro_ros_arduino/issues/14#issuecomment-722242175)
    - [Array messages on micro ros](https://github.com/micro-ROS/rmw-microxrcedds/issues/53)

---
https://blog.csdn.net/RNG_uzi_/article/details/87340932
> TODO
> - [ ] pui_overlapping
> - [ ] pui_drive (codes in dynamixel_sdk... needs to be clean)
> - [x] velodyne
> - [x] point_to_laser



---
https://www.stereolabs.com/docs/code-samples/
https://www.stereolabs.com/docs/ros2/positional-tracking/


https://la.mathworks.com/help/ros/ug/exchange-data-with-ros-2-publishers-and-subscribers.html

https://www.twblogs.net/a/5e81f804bd9eee211686735e

https://gist.github.com/fntsrlike/323c171f88246c9357bb

https://www.youtube.com/watch?v=Nbc7fzxFlYo

[ros2 gazebo](https://cloud.tencent.com/developer/article/1508804)


## cheat-sheet
#### collect data
```
ros2 launch pui_bringup velodyne_all_launch.py
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
ros2 bag record -a -o bag_name
```

#### replay data
```
ros2 bag play bag_name --loop
ros2 launch pui_uwb trilateration_mse.launch.py
ros2 run pui_markers uwb_marker
ros2 run pui_markers tag_path 
rviz2
```


![](https://i.imgur.com/Au3jMkI.png)

