#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('weed_locator'),'config','params.yaml')
    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)['crop_decimate_node']['ros__parameters']
    video_source = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('weed_locator'),'launch','video_source.launch.py')))
    rect_comp = ComposableNode(package = 'image_proc',
                               plugin = 'image_proc::RectifyNode',
                               name = 'rectify_color_node',
                               remappings = [('/image','/video_source/raw'),
                                             ('/camera_info','/video_source/camera_info'),
                                             ('/image_rect','/video_source/rect')])
    crop_comp = ComposableNode(package = 'image_proc',
                               plugin = 'image_proc::CropDecimateNode',
                               name = 'crop_decimate_node',
                               remappings = [('/in/image_raw','/video_source/rect'),
                                             ('/in/camera_info','/video_source/camera_info'),
                                             ('/out/image_raw','/pi_cam/image_rect'),
                                             ('/out/camera_info','/pi_cam/camera_info')],
                               parameters = [config_params])
    image_processing = ComposableNodeContainer(namespace='',
                                               name='image_proc_container',
                                               package='rclcpp_components',
                                               executable='component_container',
                                               composable_node_descriptions=[rect_comp,crop_comp])
    ld.add_action(video_source)
    ld.add_action(image_processing)
    return ld