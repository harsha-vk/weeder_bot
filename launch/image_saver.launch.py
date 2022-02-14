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
    image_rect = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('weed_locator'),'launch','image_rect.launch.py')))
    saver_node = Node(package = 'image_view',
                      executable = 'extract_images',
                      parameters = [config_path],
                      remappings = [('/image','/pi_cam/image_rect')])
    ld.add_action(image_rect)
    ld.add_action(saver_node)
    return ld