import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    joy_pkg_dir = LaunchConfiguration(
        'joy_pkg_dir',
        default=os.path.join(get_package_share_directory('pui_teleop'),'launch'))

    imu_pkg_dir = LaunchConfiguration(
        'imu_pkg_dir',
        default=os.path.join(get_package_share_directory('bluespace_ai_xsens_mti_driver'),'launch'))

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joy_pkg_dir,'/pui_teleop_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_pkg_dir,'/xsens_mti_node.launch.py'])
        ),

        # this node is not working
        Node(
            package='pui_drive',
            executable='xl430_controll_node',
            name='xl430_controll_node',
            output='screen'
        ),
    ])