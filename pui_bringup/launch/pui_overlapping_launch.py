import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pui_bringup')
 
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map_file')
    rviz_config_file = LaunchConfiguration('rviz_file')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server']
    use_sim_time = LaunchConfiguration('use_sim_time')    

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', '/tf'),
                  ('/tf_static', '/tf_static')]

   # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'pui_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(bringup_dir, 'maps', 'hector_fixed.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'rviz_file',
            default_value=os.path.join(bringup_dir, 'rviz', 'map_overlapping.rviz'),
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the pui stack'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Trilateration
        Node(
            package='pui_uwb',
            executable='trilateration_mse',
            name='trilateration_mse',
            output='screen'),   

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        # Lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),    

        # UWB markers
        Node(
            package='pui_markers',
            executable='uwb_marker',
            name='uwb_marker',
            output='screen'),    

        # Visualize tag_path 
        Node(
            package='pui_markers',
            executable='tag_path',
            name='tag_path',
            output='screen'),    

        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file])    

    ])