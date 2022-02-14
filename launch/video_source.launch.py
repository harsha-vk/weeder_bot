#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('weed_locator'),'config','params.yaml')
    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)['video_source']['ros__parameters']
    image_node = Node(package = 'ros_deep_learning',
                      executable = 'video_source',
                      parameters = [config_params])
    ld.add_action(image_node)
    return ld