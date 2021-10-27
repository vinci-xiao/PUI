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
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # Create the launch configuration variables

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    pui_param_dir = LaunchConfiguration(
        'pui_param_dir',
        default=os.path.join(
            get_package_share_directory('pui_bringup'),
            'param','burger' + '.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with U2D2'),

        DeclareLaunchArgument(
            'pui_param_dir',
            default_value=pui_param_dir,
            description='Full path to pui parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/pui_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            namespace=[namespace],
            package='pui_node',
            executable='pui_ros',
            parameters=[pui_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
