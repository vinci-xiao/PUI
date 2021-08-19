# PUI on ROS2

## Setup
1. Preinstall packages
    ```
    sudo apt install ros-foxy-joy ros-foxy-teleop-twist-joy libeigen3-dev
    sudo apt install ros-foxy-rosbag2 ros-foxy-rviz2 ros-foxy-rviz2-dbgsym ros-foxy-rviz-common*
    sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
    sudo apt install ros-foxy-dynamixel-sdk
    ```
2. Git Clone pui packages
    ```
    cd ~/pui_ws
    git clone -b ros2-foxy https://github.com/vinci-xiao/PUI.git src
    colcon build
    ```
3. Load hardware rules
    ```
    cp src/pui_bringup/rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

4. Build xsense third-party driver (bluespace_ai_xsens_mti_driver)
    - Copy bluespace_ai_xsens_mti_driver folder into your ROS 2.0 workspace 'src' folder.
        Make sure the permissions are set to **o+rw** on your files and directories.
    - Build xspublic from your ament workspace:
        ```
        pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
        ```
    - Build Xsens MTi driver package:
        ```
        colcon build
        ```
    - Source workspace:
        ```
        source install/setup.bash
        ```

## 
- Git workflow see [here](https://medium.com/i-think-so-i-live/git%E4%B8%8A%E7%9A%84%E4%B8%89%E7%A8%AE%E5%B7%A5%E4%BD%9C%E6%B5%81%E7%A8%8B-10f4f915167e)


