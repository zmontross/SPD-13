"""
# Description: Launch file for a single instance of the 'pid_velocity'.
#               This node has overrides for published and subscribed topics.
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
        'pid_velocity.yaml'
    )

    pid_velocity_left = Node(
        name = 'pid_velocity_left',
        package = 'spd_13',
        executable = 'pid_velocity',
        output = 'screen',
        parameters = [params],
        remappings = [
            ('/velocity_setpoint', '/velocity_left'),
            ('/encoder', '/encoder_left'),
            ('/motor_power', '/motor_power_left'),
            ('/set_motor_power', '/set_motor_power_left'),
        ],
    )
    ld.add_action(pid_velocity_left)

    return ld