import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace'),

        Node(
            namespace=[namespace],
            package = 'pui_uwb',
            name = 'hybrid_cosine',
            executable = 'hybrid_cosine',
            # parameters= [
            #             params,
            #             {"tag_id": 2}
            #             ]
            ),
    ])
