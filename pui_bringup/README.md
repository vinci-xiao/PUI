# pui_bringup
-

### pre-install dynamixel_sdk 
'cd ~/pui_ws/src'
'git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK'
'cd ~/pui_ws && colcon build --symlink-install'
'sudo chmod a+rw /dev/ttyUSB0'

### USB device commands
```
lsusb
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}'
```

### Load hardware rules
```
cp rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```