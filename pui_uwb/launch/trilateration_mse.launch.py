import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    
    remappings = [('/uwb_range', '/uwb_teensy_node_publisher')]
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='pui_uwb', node_executable='trilateration_mse', name='trilateration_mse',
            remappings=remappings)
    ])