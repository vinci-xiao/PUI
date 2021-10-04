"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_dir= os.path.join(
        ament_index_python.packages.get_package_share_directory('pui_bringup'),'config')
    # driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(config_dir, 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    # convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(config_dir, 'VLP16-velodyne_convert_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(config_dir, 'VLP16db.yaml')
    velodyne_convert_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                    executable='velodyne_convert_node',
                                                    output='both',
                                                    parameters=[convert_params])

    # laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(config_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])


    return launch.LaunchDescription([velodyne_driver_node,
                                     velodyne_convert_node,
                                     velodyne_laserscan_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])