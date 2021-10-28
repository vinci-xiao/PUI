import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # params_dir = os.path.join(get_package_share_directory('pui_uwb'), 'params')
    # param_config = os.path.join(params_dir, "uwb.yaml")
    # with open(param_config, 'r') as f:
        # params = yaml.safe_load(f)["following_node"]["ros__parameters"]

    node1= Node(
        # namespace='tag1',
        package = 'pui_uwb',
        name = 'hybrid_cosine',
        executable = 'hybrid_cosine',
        # parameters= [
        #             params,
        #             {"tag_id": 2}
        #             ]
    )

    # node2= Node(
    #     namespace='tag2',
    #     package = 'pui_uwb',
    #     name = 'following_node',
    #     executable = 'following_node',
    #     parameters= [
    #                 params,
    #                 {"tag_id": 1}
    #                 ]
    # )

    ld.add_action(node1)
    # ld.add_action(node2)
    return ld