

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    serial_arbiter = Node(
        name = 'Serial Arbiter',
        package = 'spd_13',
        executable = 'serial_arbiter',
        output = 'screen',
        parameters = [{
            'serial_port_path': '/dev/ttyACM0',
            'serial_baud_rate': '115200',
            'serial_terminator_char': '\r',
            'data_publish_period', '0.01'
        }],
    )
    ld.add_action(serial_arbiter)

    differential_odom = Node(
        name = 'Differential Odometry',
        package = 'spd_13',
        executable = 'differential_odometry',
        output = 'screen',
        parameters = [{
            
        }]
    )
    ld.add_action(diff_odom)

    noodle = Node(
        name = '',
        package = 'spd_13',
        executable = '',
        output = 'screen',
        parameters = [{
            
        }]
    )
    ld.add_action(noodle)

    noodle = Node(
        name = '',
        package = 'spd_13',
        executable = '',
        output = 'screen',
        parameters = [{
            
        }]
    )
    ld.add_action(noodle)
    
    noodle = Node(
        name = '',
        package = 'spd_13',
        executable = '',
        output = 'screen',
        parameters = [{
            
        }]
    )
    ld.add_action(noodle)
    
    noodle = Node(
        name = '',
        package = 'spd_13',
        executable = '',
        output = 'screen',
        parameters = [{
            
        }]
    )
    ld.add_action(noodle)


    return ld