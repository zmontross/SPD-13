"""
# Description: Launch file for a single instance of the 'differential_cmd_vel' node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    laser_beep = Node(
        name = 'laser_beep',
        package = 'spd_13',
        executable = 'laser_beep',
        output = 'screen'
    )
    ld.add_action(laser_beep)

    return ld