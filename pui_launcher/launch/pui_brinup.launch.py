import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('pui_launcher'),  
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    # if LDS_MODEL == 'LDS-01':
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    # elif LDS_MODEL == 'LDS-02':
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
    #     LDS_LAUNCH_FILE = '/ld08.launch.py'
    # else:
    #     lidar_pkg_dir = LaunchConfiguration(
    #         'lidar_pkg_dir',
    #         default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hlds_laser.launch.py']),  # <----- CHANGE THIS
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            node_namespace='pui_1',  # <---------------- CHANGE THIS!
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])