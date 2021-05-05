import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('spd_13'),
        'param',
        'serial_arbiter.yaml'
    )

    serial_arbiter = Node(
        name = 'serial_arbiter',
        package = 'spd_13',
        executable = 'serial_arbiter',
        output = 'screen',
        parameters = [config],
    )
    ld.add_action(serial_arbiter)

    return ld