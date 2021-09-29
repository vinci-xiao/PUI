import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'pui_bot' + '.urdf'
    pkg_share = FindPackageShare(package='pui_description').find('pui_description')
    default_model_path = os.path.join(pkg_share, 'urdf/pui_bot.urdf')

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('pui_description'),
        'urdf',
        urdf_file_name)

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        DeclareLaunchArgument
        (
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument
        (
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),

        Node
        (
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time,'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        )
    ])