# PUI on ROS2

## cheat-sheet
#### remote control
```
ros2 launch pui_node drive_launch.py
ros2 launch pui_teleop pui_teleop_launch.py
```

#### test uwb for init_pose
prerequist:
    ```ros2 launch pui_bringup robot.launch.py```
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/pui_uwb -b 115200 -v6
ros2 launch pui_uwb uwb_hybrid.launch.py
```

#### Navigation 2 for pui_t2
 - on pui_t2
```
ros2 launch pui_node drive_launch.py namespace:=t2

ros2 launch bluespace_ai_xsens_mti_driver name_imu.launch.py namespace:=t2

ros2 launch pui_bringup robot_t2.launch.py use_namespace:=True namespace:=t2
```

 - on pui_master
```
ros2 launch nav2_bringup bringup_launch.py autostart:=False map:=$(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/maps/map_10153.yaml params_file:=$(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/params/t2_params.yaml use_sim_time:=False use_namespace:=true namespace:=t2

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/rviz/t2_default_view.rviz
```

#### test Rviz2 panel for multi-robot
```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/rviz/nav2_custom_view.rviz
```

#### build map
```
ros2 launch pui_bringup velodyne_all_launch.py
ros2 launch pui_bringup robot.launch.py
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
ros2 launch pui_navigation2 online_async_launch.py
```

#### save a map
```
ros2 launch nav2_map_server map_saver_server.launch.py 
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: map_1015_2, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
ros2 run nav2_map_server map_saver_cli -f ~/map_name
```

#### Navigation 2 for pui_bot (master)
```
ros2 launch pui_bringup robot.launch.py use_namespace:=False

ros2 launch pui_bringup velodyne_all_launch.py

ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py

ros2 launch pui_node drive_launch.py

ros2 launch nav2_bringup bringup_launch.py autostart:=False map:=$(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/maps/map_10153.yaml params_file:=$(ros2 pkg prefix pui_navigation2)/share/pui_navigation2/params/nav2_params.yaml use_sim_time:=False

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

-> Click on the 2D Pose Estimate button and point the location of the robot on the map.
-> Send a Goal Pose by using Nav2 Goal

##### publish initial_pose topic 
```
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 3.5826, y: 1.3826, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 0.4439}}, } }'
```

#### micro-ros
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
```

#### collect data
```
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


### Compile specific pkg
```
ros2 pkg create --build-type ament_cmake <-package name->
```

### Test uwb calculation
```
ros2 launch pui_description display.launch.py
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 115200 -v 6
ros2 launch pui_uwb trilateration_mse.launch.py
ros2 run pui_markers tag_path
```

## PUI hardware configuration
*[/pui_bringup/21-pui.rules] - hardware rules permission for joy and dynamixel controll

### Find USB devics info 
```
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'
udevadm info -a -n /dev/ttyUSB0 | grep '{devpath}'
```
### Implant hardware rules
```
sudo cp *.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
