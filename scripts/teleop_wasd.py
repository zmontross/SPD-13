#!/usr/bin/env python3

#TODO BUG cmd_vel msg OK, but prints of lin/ang V to screen breach min/max limits briefly before clamping.
#TODO Slider (ideal) or selection buttons for FOV
#TODO Selection buttons for Resolution
#TODO On-screen explanation of WASD controls, labels for FOV/Res. selections
#TODO Info about scans (header data, time since last rx)
#TODO Hold Shift to for manual-step mode; don't auto-decrement velocities to zero
#TODO Mouse-hover over ScanPip to see range
#TODO Average the ranges, filter outliers (e.g. range==0 ? use_prev_range : use_new_range)
#TODO Auto-save selections to a file
#TODO Offload classes and helper functions to other python modules, import them here.

import sys

import pygame
from pygame import color
from pygame.locals import *


import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

LINEAR_MIN = -0.09
LINEAR_MAX = 0.09
LINEAR_STEP = 0.01
LINEAR_DRAG_STEP = 0.0075

ANGULAR_MIN = -2.00
ANGULAR_MAX = 2.00
ANGULAR_STEP = 0.2
ANGULAR_DRAG_STEP = 0.1



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
        
        self._range_height_scale_const = 200
        self._range_color_scale_const = 60
        
        self.pip_width = self.width / numpips

        self.pips_near = []

        for p in range(numpips):
            pip = ScanPip(
                width = self.pip_width,
                height = self.height,
                posx = self.posx - self.width/2 + self.pip_width/2 + self.pip_width*p,
                posy = self.posy,
                color = (clamp(255-p, 0, 255), clamp(192-p, 0, 255), clamp(96+p, 0, 255))
            )
            self.pips_near.append(pip)

        self.pips_far = []
        for p in range(numpips):
            pip = ScanPip(
                width = self.pip_width,
                height = self.height,
                posx = self.posx - self.width/2 + self.pip_width/2 + self.pip_width*p,
                posy = self.posy,
                color = (clamp(255-p, 0, 255), clamp(192-p, 0, 255), clamp(96+p, 0, 255))
            )
            self.pips_far.append(pip)

    def update(self, ranges):

        # 109 scan ranges, centered on number 55, or 54 if index-base-zero

        for n in range(0, 109):

            if n < 54:
                range_val = ranges[54-n]
            elif n == 54:
                range_val = ranges[0]
            else:
                range_val = ranges[359 - (n-54)]

            range_val_scaled = clamp( range_val * self._range_height_scale_const, 64, self.height)
            height_val = self.height - range_val_scaled
            color_val = (clamp(255 - range_val * self._range_color_scale_const, 0, 255), 0, 0)


            self.pips_far[n].update(
                height=self.pips_near[n].height,
                color=(0, 0, clamp(255 - range_val * self._range_color_scale_const, 0, 255))
            )

            self.pips_near[n].update(
                height=height_val,
                color=color_val
            )

    def draw(self, window):

        pygame.Surface.lock(window)
        for pip in self.pips_far:
            pygame.draw.rect(
                window,
                pip.color,
                pygame.Rect(
                    pip.posx - pip.width/2,
                    pip.posy - pip.height/2,
                    pip.width,
                    pip.height
                    )
                )

        for pip in self.pips_near:
            pygame.draw.rect(
                window,
                pip.color,
                pygame.Rect(
                    pip.posx - pip.width/2,
                    pip.posy - pip.height/2,
                    pip.width,
                    pip.height
                    )
                )
        pygame.Surface.unlock(window)


class TeleopWasd(Node):

    def __init__(self):

        super().__init__('teleop_wasd')

        self.last_ranges = [12.00] * 360 # The array is already known to have a max of 360 elements.
        self.scan_received = False

        self.target_linear_velocity = 0.00
        self.target_linear_velocity_last = 0.00
        self.target_angular_velocity = 0.00
        self.target_angular_velocity_last = 0.00

        self.linear_drag = True
        self.angular_drag = True

        self.qos = QoSProfile(depth=10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_cb, self.qos)

        self.timer_pub_cmd_vel = self.create_timer(0.10, self.cmd_vel_timer_cb)

        self.get_logger().info("Initialized.")

    def scan_cb(self, message):

        if not self.scan_received:
            self.scan_received = True

        for i in range(0, 360):
            # Grab datapoint
            self.last_ranges[i] = message.ranges[i]

    def cmd_vel_timer_cb(self):

        if self.linear_drag:
            if self.target_linear_velocity >= LINEAR_DRAG_STEP:
                self.target_linear_velocity = self.target_linear_velocity - LINEAR_DRAG_STEP
            elif self.target_linear_velocity <= -LINEAR_DRAG_STEP:
                self.target_linear_velocity = self.target_linear_velocity + LINEAR_DRAG_STEP
            else:
                self.target_linear_velocity = 0.00

        if self.angular_drag:
            if self.target_angular_velocity >= ANGULAR_DRAG_STEP:
                self.target_angular_velocity = self.target_angular_velocity - ANGULAR_DRAG_STEP

            elif self.target_angular_velocity <= -ANGULAR_DRAG_STEP:
                self.target_angular_velocity = self.target_angular_velocity + ANGULAR_DRAG_STEP

            else:
                self.target_angular_velocity = 0.00
        
        self.target_linear_velocity = clamp(
                self.target_linear_velocity,
                LINEAR_MIN,
                LINEAR_MAX)

        self.target_angular_velocity = clamp(
                self.target_angular_velocity,
                ANGULAR_MIN,
                ANGULAR_MAX)

        if abs(self.target_linear_velocity) < LINEAR_STEP:
            self.target_linear_velocity = 0.00
        else:
            round(self.target_linear_velocity, 3)

        if abs(self.target_angular_velocity) < ANGULAR_STEP:
            self.target_angular_velocity = 0.00
        else:
            round(self.target_angular_velocity, 3)

        if (self.target_linear_velocity != self.target_linear_velocity_last) or (self.target_angular_velocity != self.target_angular_velocity_last):
            cmd_vel = make_cmd_vel(self.target_linear_velocity, self.target_angular_velocity)
            self.pub_cmd_vel.publish(cmd_vel)
            
            self.target_linear_velocity_last = self.target_linear_velocity
            self.target_angular_velocity_last = self.target_angular_velocity

def pygame_userevent(bool):
    if bool:
        return False
    else:
        return True

def main():

    rclpy.init()
    teleop_wasd = TeleopWasd()

    pygame.init()
    # pygame_width = 1024
    # pygame_height = 768
    pygame_width = 1920
    pygame_height = 1080
    pygame_fps = 60

    pygame_fill_white = (255, 255, 255)
    pygame_fill_gray = (96, 96, 96)
    pygame_fill_black = (0, 0, 0)

    clock = pygame.time.Clock()

    handle_input = False
    EVENT_PROCESS_KEYS = pygame.USEREVENT
    pygame.time.set_timer(EVENT_PROCESS_KEYS, 100)

    pygame_window = pygame.display.set_mode((pygame_width, pygame_height))

    pygame_font = pygame.font.Font(None, 36)


    sl = ScanLine(
        width=pygame_width+1,
        height=pygame_height,
        posx=pygame_width//2,
        posy=pygame_height//2,
        numpips=109
        )

    try:
        while(1):

            # Process ROS2 Node events
            rclpy.spin_once(teleop_wasd, timeout_sec=0.001)


            # Process Pygame events
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == EVENT_PROCESS_KEYS:
                    handle_input = pygame_userevent(handle_input)


            # Process Pygame Key Presses


            if handle_input:

                handle_input = False

                keys_pressed = pygame.key.get_pressed()

                if keys_pressed[K_ESCAPE]:
                    pygame.quit()
                    sys.exit()

                teleop_wasd.linear_drag = True
                teleop_wasd.angular_drag = True

                # if clock.

                if keys_pressed[K_w]:
                    teleop_wasd.target_linear_velocity = teleop_wasd.target_linear_velocity + LINEAR_STEP
                    teleop_wasd.linear_drag = False

                if keys_pressed[K_s]:
                    teleop_wasd.target_linear_velocity = teleop_wasd.target_linear_velocity - LINEAR_STEP
                    teleop_wasd.linear_drag = False

                if keys_pressed[K_a]:
                    teleop_wasd.target_angular_velocity = teleop_wasd.target_angular_velocity + ANGULAR_STEP
                    teleop_wasd.angular_drag = False

                if keys_pressed[K_d]:
                    teleop_wasd.target_angular_velocity = teleop_wasd.target_angular_velocity - ANGULAR_STEP
                    teleop_wasd.angular_drag = False

                if keys_pressed[K_SPACE] or keys_pressed[K_x]:
                    teleop_wasd.target_linear_velocity = 0.00
                    teleop_wasd.target_angular_velocity = 0.00
                    teleop_wasd.linear_drag = False
                    teleop_wasd.angular_drag = False


            # Draw Pygame GUI

            pygame_window.fill(pygame_fill_black)

            velocity_string = "Linear: {0:.3f}    Angular: {1:.3f}".format(
                                teleop_wasd.target_linear_velocity, teleop_wasd.target_angular_velocity)

            pygame_velocity_text = pygame_font.render(velocity_string, 1, pygame_fill_white)
            vposx, vposy = pygame_font.size(velocity_string)
            vtext_pos = pygame_velocity_text.get_rect(centerx=vposx/2, centery=vposy/2)
            pygame_window.blit(pygame_velocity_text, vtext_pos)


            if teleop_wasd.scan_received:
                sl.update(teleop_wasd.last_ranges)
                sl.draw(pygame_window)

            else:
                no_scan_string = "Waiting for scan..."
                no_scan_text = pygame_font.render(no_scan_string, 1, pygame_fill_white)
                no_scan_posx, no_scan_posy = pygame_font.size(no_scan_string)
                no_scan_pos = no_scan_text.get_rect(centerx=(pygame_width/2 - no_scan_posx/2), centery=(pygame_height/2 - no_scan_posy/2))
                pygame_window.blit(no_scan_text, no_scan_pos)

            pygame.display.update()

            clock.tick(pygame_fps)


    except Exception as e:

        print(e)

    finally:

        cmd_vel = make_cmd_vel(0.00, 0.00)

        teleop_wasd.pub_cmd_vel.publish(cmd_vel)


if __name__ == '__main__':
    main()
