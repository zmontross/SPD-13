"""
# Description: Launch file for a single instance of the 'serial_arbiter' node
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
        'serial_arbiter.yaml'
    )

    serial_arbiter = Node(
        name = 'serial_arbiter',
        package = 'spd_13',
        executable = 'serial_arbiter',
        output = 'screen',
        parameters = [params],
    )
    ld.add_action(serial_arbiter)

    return ld