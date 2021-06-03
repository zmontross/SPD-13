
from math import pi

from numpy import array, interp

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from tf2_ros import *

class LaserBeep(Node):

    def __init__(self):
        super().__init__('laser_beep')

        self.qos = QoSProfile(
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # self.pub_cmdvel = self.create_publisher(Twist, "cmd_vel", self.cmdvel_cb, self.qos)

        self.pub_beep = self.create_publisher(Int16, "beep", self.qos)

        self.sub_laser_scan = self.create_subscription(LaserScan, "scan", self.laser_scan_cb, self.qos)

        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_cb, self.qos)


    def laser_scan_cb(self, message):

        # self.get_logger().info("I like pie and I don't know why but it lets me fly until the day is neigh.")

        front = self.average_distance(0, 10, message)
        front_right = self.average_distance(45, 10, message)
        right = self.average_distance(90, 10, message)
        rear_right = self.average_distance(135, 10, message)
        rear = self.average_distance(180, 10, message)
        rear_left = self.average_distance(225, 10, message)
        left = self.average_distance(270, 10, message)
        front_left = self.average_distance(315, 10, message)

        b = Int16()

        if (range_val < 0.25) or (range_val == float('inf')):
            b.data = 4
        elif range_val < 0.50:
            b.data = 3
        elif range_val < 0.75:
            b.data = 2
        elif range_val < 1.00:
            b.data = 1
        else:
            b.data = 0

        if b != 0:
            self.pub_beep.publish(b)


    def odom_cb(self, message):
        # calculate distance traveled since last odom rx.
        self.get_logger().info("I like pie and I don't know why but it lets me fly until the day is neigh.")




    def average_distance(self, degree: int, window_width: int, scan: LaserScan):
        """
        Returns the average distance measured around a pivot-value in a LaserScan message.
        degree: "Pivot value" around which the average distance will be calculated.
        window_width: Whole-number angle-width centered around 'degree'. Rounded-up if even (e.g. '2' rounded to '3').
        scan: LaserScan message. It is assumed that the message will contain a ranges[] array
            with per-degree measurements (total of 360)
            whose first value is zero degrees (forward)
            and whose last value is 359 degrees
        """

        average = 0.0

        degree = degree % 360

        if (window_width % 2) == 0:
            window_width = window_width + 1

        window_min = (degree - window_width // 2)
        window_max = (degree + window_width // 2)

        summation = 0.0
        for i in range(window_min, window_max+1):
            summation = summation + scan.ranges[i]

        average = summation / window_width

        return average
        




def main(args=None):

    rclpy.init(args=args)

    laser_beep = LaserBeep()

    try:
        while rclpy.ok():
            rclpy.spin_once(laser_beep, timeout_sec=0.01)
            # do things

    except KeyboardInterrupt:
        laser_beep.get_logger().info("Keyboard Interrupt")
        pass
                
    except Exception as e:
        print(e)
        pass

    laser_beep.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
