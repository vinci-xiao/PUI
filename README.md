# PUI on ROS2

## cheat-sheet
#### remote control
```
ros2 run pui_node dynamixel_controller
ros2 launch pui_teleop pui_teleop_launch.py
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
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: map_name, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
ros2 run nav2_map_server map_saver_cli -f ~/map_name
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

