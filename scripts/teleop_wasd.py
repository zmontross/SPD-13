#!/usr/bin/env python3

# Hold wasd
# Accelerate at some fixed rate.
# When key released decelerate back to zero.
# If opposing keys held, e.g. w/d or a/s, then they should cancel-out
# Need a loop that checks for keys held, but transmits new msgs at fixed period (100ms?)
# Accel rate for ctrl, separate decel rate for no-input slowing. accel > decel.


import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist, Vector3
from rclpy.qos import QoSProfile


import termios
import tty

from datetime import datetime

LINEAR_MIN = -0.09
LINEAR_MAX = 0.09
LINEAR_STEP = 0.01

ANGULAR_MIN = -2.00
ANGULAR_MAX = 2.00
ANGULAR_STEP = 0.25


banner = """
Control Your SPD-13!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity (0.09)
a/d : increase/decrease angular velocity (2.00)

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_velocities(target_linear_velocity, target_angular_velocity, flag):

    print('\r{}'.format(' '*80), end='') # Blank-out previous characters

    flag_result = ''
    if flag == True:
        flag_result = '!'

    print('\rlinear velocity {}\t angular velocity {}   {}{}'.format(
        target_linear_velocity,
        target_angular_velocity,
        flag_result,
        ' '*10),
        end=''
    )


def clamp(x, low=0, high=100):

    return max(low, min(x, high))


def make_cmd_vel_allstop():
    v = Vector3(
        x = 0.00,
        y = 0.00,
        z = 0.00
    )

    w = Vector3(
        x = 0.00,
        y = 0.00,
        z = 0.00
    )

    cmd_vel = Twist(
        linear = v,
        angular = w
    )

    return cmd_vel


def main():

    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_wasd')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    target_linear_velocity = 0.00
    target_angular_velocity = 0.00

    time_previous = datetime.now()
    time_since_last = 0.00 # seconds

    try:
        print(banner)
        print_velocities(target_linear_velocity, target_angular_velocity, False)
        while(1):
            
            key = get_key(settings)
            
            if key == 'w':
                target_linear_velocity = clamp(
                    target_linear_velocity + LINEAR_STEP,
                    LINEAR_MIN,
                    LINEAR_MAX)
            elif key == 's':
                target_linear_velocity = clamp(
                    target_linear_velocity - LINEAR_STEP,
                    LINEAR_MIN,
                    LINEAR_MAX)
            elif key == 'a':
                target_angular_velocity = clamp(
                    target_angular_velocity + ANGULAR_STEP,
                    ANGULAR_MIN,
                    ANGULAR_MAX)
            elif key == 'd':
                target_angular_velocity = clamp(
                    target_angular_velocity - ANGULAR_STEP,
                    ANGULAR_MIN,
                    ANGULAR_MAX)
            # elif key == ' ' or key == 'x':
            #     target_linear_velocity = 0.00
            #     target_angular_velocity = 0.00
            else:
                # Stop on Ctrl+C or Escape-key
                if key == '\x03' or key == '\x1B':
                    print('\r\n')
                    break


            if abs(target_linear_velocity) < LINEAR_STEP:
                target_linear_velocity = 0.00
            else:
                round(target_linear_velocity, 3)

            if abs(target_angular_velocity) < ANGULAR_STEP:
                target_angular_velocity = 0.00
            else:
                round(target_angular_velocity, 3)

            flag = False

            time_current = datetime.now()
            time_since_last = time_current - time_previous

            if time_since_last.total_seconds() >= 0.5:
                flag = True
                time_previous = time_current
            


                # TODO Allow multiple keys to be held/detected.
                # if key == 'w':
                #     velocity++
                # elif key == 's':
                #     velocity--
                # else:
                #     if velocity > 0.00:
                #         velocity -= LINEAR_INTERTIAL_STEP
                #     elif velocity < 0.00:
                #         velocity += LINEAR_INTERTIAL_STEP


                # if key == 'a':
                #     ang++
                # elif key == 'd':
                #     ang--
                # else:
                #     if ang > 0.00:
                #         ang -= ANGULAR_INTERTIAL_STEP
                #     elif ang < 0.00:
                #         ang += ANGULAR_INTERTIAL_STEP
                    


            print_velocities(target_linear_velocity, target_angular_velocity, flag)


            # TODO Jacket in "did this change?" logic
            v = Vector3(
                x = target_linear_velocity,
                y = 0.0,
                z = 0.0
            )

            w = Vector3(
                x = 0.0,
                y = 0.0,
                z = target_angular_velocity
            )

            cmd_vel = Twist(
                linear = v,
                angular = w
            )

            pub.publish(cmd_vel)

    except Exception as e:
        print(e)

    finally:

        pub.publish(make_cmd_vel_allstop())

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
