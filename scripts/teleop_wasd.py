#!/usr/bin/env python3

# Hold wasd
# Accelerate at some fixed rate.
# When key released decelerate back to zero.
# If opposing keys held, e.g. w/d or a/s, then they should cancel-out
# Need a loop that checks for keys held, but transmits new msgs at fixed period (100ms?)
# Accel rate for ctrl, separate decel rate for no-input slowing. accel > decel.

# TODO Maybe change velocities based on gap from min/max? e.g. v = v + (max-v)/2

import sys

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


import pygame
from pygame import color
from pygame.locals import *

LINEAR_MIN = -0.09
LINEAR_MAX = 0.09
LINEAR_STEP = 0.01
LINEAR_DRAG_STEP = 0.0075

ANGULAR_MIN = -2.00
ANGULAR_MAX = 2.00
ANGULAR_STEP = 0.2
ANGULAR_DRAG_STEP = 0.1

global_scan_data_array = []


def clamp(x, low=0, high=100):

    return max(low, min(x, high))


def make_cmd_vel(v, w):
    v = Vector3(
        x = v,
        y = 0.00,
        z = 0.00
    )

    w = Vector3(
        x = 0.00,
        y = 0.00,
        z = w
    )

    cmd_vel = Twist(
        linear = v,
        angular = w
    )

    return cmd_vel


class ScanPip():
    def __init__(self, width=10, height=10, posx=10, posy=10, color=(255, 255, 255)):
        super().__init__()
        self.width = width
        self.height = height
        self.posx = posx
        self.posy = posy
        self.color = color

    def update(self, height=10, color=(128, 128, 128)):
        self.height = height
        self.color = color


class ScanLine():
    def __init__(self, width=500, height=100, posx=100, posy=100, numpips=10):
        super().__init__()
        self.width = width
        self.height = height
        self.posx = posx
        self.posy = posy

        self.pip_width = self.width / numpips

        self.pips = []
        for p in range(numpips):
            pip = ScanPip(
                width = self.pip_width,
                height = self.height,
                posx = self.posx - self.width/2 + self.pip_width/2 + self.pip_width*p,
                posy = self.posy,
                color = (clamp(255-p, 0, 255), clamp(192-p, 0, 255), clamp(96+p, 0, 255))
            )
            self.pips.append(pip)


class TeleopWasd(Node):

    def __init__(self):

        super().__init__('teleop_wasd')

        self.qos = QoSProfile(depth=10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_cb, self.qos)

        self.get_logger().info("Initialized.")

    def scan_cb(self, message):

        global_scan_data_array.copy(message.ranges)

def main():

    rclpy.init()
    node_teleop_wasd = TeleopWasd()

    target_linear_velocity = 0.00
    target_linear_velocity_last = 0.00
    target_angular_velocity = 0.00
    target_angular_velocity_last = 0.00


    linear_drag = True
    angular_drag = True



    pygame.init()
    # pygame.font.init()
    pygame_width = 1024
    pygame_height = 768
    pygame_fps = 10

    FramePerSec = pygame.time.Clock()

    pygame_fill_white = (255, 255, 255)
    pygame_fill_gray = (128, 128, 128)

    pygame_window = pygame.display.set_mode((pygame_width, pygame_height))

    pygame_font = pygame.font.Font(None, 36)

    
    ## TODO What if the pips grew in size as the distance became closer?
    ## TODO Pips both grow in size, and transition to red, as distances draw near. shrink and go blue with distance.

    sl = ScanLine(width=1025, height=300, posx=pygame_width//2, posy=pygame_height//2, numpips=109)
    

    try:
        while(1):

            # Need to spin
            rclpy.spin_once(node_teleop_wasd, timeout_sec=0.001)

            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()

            pygame_window.fill(pygame_fill_gray)

            linear_drag = True
            angular_drag = True

            keys_pressed = pygame.key.get_pressed()

            # Stop on Escape-key
            if keys_pressed[K_ESCAPE]:
                pygame.quit()
                sys.exit()

            if keys_pressed[K_w]:
                target_linear_velocity = target_linear_velocity + LINEAR_STEP
                linear_drag = False

            if keys_pressed[K_s]:
                target_linear_velocity = target_linear_velocity - LINEAR_STEP
                linear_drag = False

            if keys_pressed[K_a]:
                target_angular_velocity = target_angular_velocity + ANGULAR_STEP
                angular_drag = False

            if keys_pressed[K_d]:
                target_angular_velocity = target_angular_velocity - ANGULAR_STEP
                angular_drag = False

            if keys_pressed[K_SPACE] or keys_pressed[K_x]:
                target_linear_velocity = 0.00
                target_angular_velocity = 0.00
                linear_drag = False
                angular_drag = False
            

            if linear_drag:
                if target_linear_velocity >= LINEAR_DRAG_STEP:
                    target_linear_velocity = target_linear_velocity - LINEAR_DRAG_STEP

                elif target_linear_velocity <= -LINEAR_DRAG_STEP:
                    target_linear_velocity = target_linear_velocity + LINEAR_DRAG_STEP

                else:
                    target_linear_velocity = 0.00
                

            if angular_drag:
                if target_angular_velocity >= ANGULAR_DRAG_STEP:
                    target_angular_velocity = target_angular_velocity - ANGULAR_DRAG_STEP

                elif target_angular_velocity <= -ANGULAR_DRAG_STEP:
                    target_angular_velocity = target_angular_velocity + ANGULAR_DRAG_STEP

                else:
                    target_angular_velocity = 0.00

            
            target_linear_velocity = clamp(
                    target_linear_velocity,
                    LINEAR_MIN,
                    LINEAR_MAX)

            target_angular_velocity = clamp(
                    target_angular_velocity,
                    ANGULAR_MIN,
                    ANGULAR_MAX)

            if abs(target_linear_velocity) < LINEAR_STEP:
                target_linear_velocity = 0.00
            else:
                round(target_linear_velocity, 3)

            if abs(target_angular_velocity) < ANGULAR_STEP:
                target_angular_velocity = 0.00
            else:
                round(target_angular_velocity, 3)



            ## GUI Stuff Below

            velocity_string = "Linear: {0:.3f}    Angular: {1:.3f}".format(
                                target_linear_velocity, target_angular_velocity)

            pygame_velocity_text = pygame_font.render(velocity_string, 1, pygame_fill_white)
            vposx, vposy = pygame_font.size(velocity_string)
            vtext_pos = pygame_velocity_text.get_rect(centerx=vposx/2, centery=vposy/2)
            pygame_window.blit(pygame_velocity_text, vtext_pos)

            # 109 scan ranges, centered on number 55, or 54 if index-base-zero

            # for i in range(-16, -1):
            #     sl.pips[54 + i].update(
            #     height=( global_scan_data_array[i]*210 ),
            #     color=(192, 64, 64))

            sl.pips[54].update(
                height=( global_scan_data_array.ranges[0]*210 ),
                color=(192, 64, 64))

            # for i in range(1, 16):
            #     sl.pips[54 + i].update(
            #     height=( global_scan_data_array[i]*210 ),
            #     color=(192, 64, 64))





            if target_linear_velocity > 0.05:
                sl.pips[5].update(height=400, color=(192,64,64))
            else:
                sl.pips[5].update(height=300, color=(64,192,64))


            for pip in sl.pips:

                pygame.draw.rect(
                    pygame_window,
                    pip.color,
                    pygame.Rect(
                        pip.posx - pip.width/2,
                        pip.posy - pip.height/2,
                        pip.width,
                        pip.height
                        )
                    )

            pygame.display.update()


            if (target_linear_velocity != target_linear_velocity_last) or (target_angular_velocity != target_angular_velocity_last):
                cmd_vel = make_cmd_vel(target_linear_velocity, target_angular_velocity)
                node_teleop_wasd.pub_cmd_vel.publish(cmd_vel)
                
                target_linear_velocity_last = target_linear_velocity
                target_angular_velocity_last = target_angular_velocity

            FramePerSec.tick(pygame_fps)

            

            

    except Exception as e:

        print(e)

    finally:

        cmd_vel = make_cmd_vel(0.00, 0.00)

        node_teleop_wasd.pub_cmd_vel.publish(cmd_vel)


if __name__ == '__main__':
    main()
