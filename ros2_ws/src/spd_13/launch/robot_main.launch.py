"""
# Description: Primary launch file for SPD-13.
#   Only the absolutely-necessary, bare-minimum nodes are launched.
#   Each of those nodes has its own launch file,
#       and this "master" launch file merely references them.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    ld = LaunchDescription()

    launchFiles = [
        "node_serial_arbiter.launch.py",
        "node_diff_odom.launch.py",
        "node_diff_cmd_vel.launch.py",
        "node_pid_vel_right.launch.py",
        "node_pid_vel_left.launch.py"
    ]

    for lf in launchFiles:
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/", lf],
                )
            )
        )
    
    return ld