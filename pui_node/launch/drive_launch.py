import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    # Create the launch configuration variables
    # usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace'),

        # DeclareLaunchArgument(
        #     'usb_port',
        #     default_value=usb_port,
        #     description='Connected USB port with U2D2'),

        Node(
            namespace=[namespace],
            package='pui_node',
            executable='dynamixel_controller',
            # arguments=['-i', usb_port],
            output='screen'),
    ])
