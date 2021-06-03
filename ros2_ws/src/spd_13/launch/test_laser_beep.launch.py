"""
# Description: Test launch file for SPD-13.
#   Used to troubleshoot PID Velocity functionality using the right motor.
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
        "node_laser_beep.launch.py"
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