# PUI on ROS2

## [Create PUI workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
```sh
mkdir -p ~/mih_ws/src && cd ~/mih_ws 
git clone https://github.com/vinci-xiao/PUI.git -b foxy src
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

## cheat-sheet

### Compile specific pkg
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


https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation

https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756

https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel