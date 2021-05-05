"""
# Description: Launch file for a single instance of the 'differential_cmd_vel' node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Parameters are stored in 'pkg_dir/param/'
    params = os.path.join(
        get_package_share_directory('spd_13'),
        'param',
        'differential_cmd_vel.yaml'
    )

    differential_cmd_vel = Node(
        name = 'differential_cmd_vel',
        package = 'spd_13',
        executable = 'differential_cmd_vel',
        output = 'screen',
        parameters = [params],
    )
    ld.add_action(differential_cmd_vel)

    return ld