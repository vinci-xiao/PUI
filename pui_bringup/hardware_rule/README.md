# PEV hardware configuration files
* [vesc](vesc/) - Include motor controller config files
* [pui-devices.rules](10-pev-devices.rules) - Define USB setting
* [setRule.sh](setRule.sh) - A script files to put "10-pev-devies.rules" and ".conf" to system

## Find USB device serial devpath
```
udevadm info -a -n /dev/ttyUSB1 | grep '{devpath}'
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}'
```
