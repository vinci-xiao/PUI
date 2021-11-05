# pui_bringup
-

### pre-install dynamixel_sdk 
'cd ~/pui_ws/src'
'git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK'
'cd ~/pui_ws && colcon build --symlink-install'
'sudo chmod a+rw /dev/ttyUSB0'

### Find USB devices info 
```
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'
udevadm info -a -n /dev/ttyUSB0 | grep '{devpath}'
```