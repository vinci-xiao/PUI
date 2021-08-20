# pui_bringup
-

### USB device commands
```
lsusb
udevadm info /dev/ttyUSB0
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}'
```

### Load hardware rules
```
cp rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```