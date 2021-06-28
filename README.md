# PUI on ROS2

[TOC]

---

Distributuon
---
https://docs.ros.org/en/foxy/Releases.html

Install 
---
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/) 
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)

Reference 
---
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
- [採智科技機器人產品中文線上手冊翻譯整理](https://hackmd.io/@idminer/usermanual-tw/https%3A%2F%2Fhackmd.io%2FyhleSV1CQhmTJCMkFci80w)

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
> - [ ] ros::duration migrate to ros2
> - [ ] clean code

#### 5. pui_bringup
- Dynamixel (pui_drive)
    - [Dynamixel Wizard 2.0 (not support Ubuntu 20.04 yet)](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
    - [Dynamixel Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository)
    - [DYNAMIXEL Quick Start Guide for ROS 2](https://www.youtube.com/watch?v=E8XPqDjof4U)
    - [DynamixelSDK 3.7.40 (Support ROS 2 Only)](https://github.com/ROBOTIS-GIT/DynamixelSDK/releases/tag/3.7.40)
    - TODO
        -  [ ] Speed controller.cpp
        -  [ ] mapping cmd_vel to speed controll
- map_server (pui_overlapping)
    - d
    - d
- rviz2 (pui_overlapping)
    - f
    - f
- transform (pui_overlapping)
    - [ROS2入门教程-发布joint states和TF](https://www.ncnynl.com/archives/201801/2257.html)
    - f
- ros2 bag (pui_overlapping)
    - e
    - f
> TODO
> - [ ] pui_overlapping
> - [ ] pui_drive
> - [ ] velodyne
> - [ ] point_to_laser



---
https://www.stereolabs.com/docs/code-samples/
https://www.stereolabs.com/docs/ros2/positional-tracking/


https://la.mathworks.com/help/ros/ug/exchange-data-with-ros-2-publishers-and-subscribers.html

https://www.twblogs.net/a/5e81f804bd9eee211686735e

https://gist.github.com/fntsrlike/323c171f88246c9357bb

https://www.youtube.com/watch?v=Nbc7fzxFlYo