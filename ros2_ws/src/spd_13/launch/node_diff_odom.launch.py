"""
# Description: Launch file for a single instance of the 'differential_odometry' node
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
        'differential_odometry.yaml'
    )

    differential_odometry = Node(
        name = 'differential_odometry',
        package = 'spd_13',
        executable = 'differential_odometry',
        output = 'screen',
        parameters = [params],
    )
    ld.add_action(differential_odometry)

    return ld