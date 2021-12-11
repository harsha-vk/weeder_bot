#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('weed_locator'),'config','params.yaml')
    gps_node = Node(package = 'ublox_gps',
                    node_executable = 'ublox_gps_node',
                    parameters = [config_path],
                    remappings = [('/fix','/gps/fix'),
                                  ('/fix_velocity','/gps/fix_velocity')])
    ld.add_action(gps_node)
    return ld