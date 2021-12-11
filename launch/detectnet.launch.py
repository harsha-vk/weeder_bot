#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('weed_locator'),'config','params.yaml')
    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)['detectnet']['ros__parameters']
    image_rect = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('weed_locator'),'launch','image_rect.launch.py')))
    detectnet_node = Node(package = 'ros_deep_learning',
                          node_executable = 'detectnet',
                          parameters = [config_params],
                          remappings = [('/detectnet/image_in','/pi_cam/image_rect')])
    ld.add_action(image_rect)
    ld.add_action(detectnet_node)
    return ld