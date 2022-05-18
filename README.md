# PUI on ROS2

## [Create PUI workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
```sh
mkdir -p ~/pui_ws/src && cd ~/pui_ws 
git clone https://github.com/vinci-xiao/PUI.git -b foxy src
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
source /install/bash.rc
```

## [Change Namespace for Multi-Robot](https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756)

1. Modify pui_launcher/launch/pui_brinup.launch.py
```sh
Node(
    package='turtlebot3_node',
    executable='turtlebot3_ros',
    node_namespace='pui_1',  # <---------------- CHANGE THIS!
    parameters=[tb3_param_dir],
    arguments=['-i', usb_port],
    output='screen'),
])
```

2. Modify pui_launcher/launch/turtlebot3_state_publisher.launch.py
```sh
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    node_namespace='pui_1',  # <------------------- CHANGE THIS!
    output='screen',
    parameters=[rsp_params, {'use_sim_time': use_sim_time}])
    ])
```

3. Modify pui_launcher/param/burger.yaml
```sh
pui_1:    # <------------------- CHANGE THIS!
  turtlebot3_node:
    ros__parameters:

pui_1:    # <------------------- CHANGE THIS!
  diff_drive_controller:
    ros__parameters:
```

Common question:
#### Could not get lock /var/lib/dpkg/lock
```sh
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
```

## cheat-sheet

#### Compile and Run!
```
. install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
ros2 launch pui_launcher pui_brinup.launch.py
```

#### Compile specific pkg
```
ros2 pkg create --build-type ament_cmake <-package name->
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

## PUI hardware configuration
*[/pui_bringup/21-pui.rules] - hardware rules permission for joy and dynamixel controll

#### Find USB devics info 
```
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'
udevadm info -a -n /dev/ttyUSB0 | grep '{devpath}'
```
#### Apply hardware rules
```
sudo cp *.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```


https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation

https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756

https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel