

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from std_msgs.msg import Float32




class DifferentialCmdVel(Node):

    def __init__(self):
        super().__init__('Cmd_Vel')

        self.declare_parameter('wheel_diameter_meters', 0.04186)

        self.declare_parameter('wheel_separation_meters', 0.1875)


        self.wheel_diameter_meters = self.get_parameter('wheel_diameter_meters').value

        self.wheel_separation_meters = self.get_parameter('wheel_separation_meters').value


        self.get_logger().info("Wheel diameter (m): {}".format(self.wheel_diameter_meters))
        self.get_logger().info("Wheel separation (m): {}".format(self.wheel_separation_meters))

        
        self.subscriber_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.publisher_velocity_right = self.create_publisher(Float32, 'velocity_right', 10)
        self.publisher_velocity_left = self.create_publisher(Float32, 'velocity_left', 10)


    def cmd_vel_callback(self, message):

        # Convert received cmd_vel into right/left motor velocities

        # REP 103, forward velocity is the received 'X' vector component.
        # Rotations in the xy-plane are around the z-axis.

        # https://github.com/jfstepha/differential-drive/blob/master/scripts/twist_to_motors.py
        # https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
        # Forward velocity is the average of the wheels, (Vleft + Vright) / 2
        # Influence of angular twist, (Vright - Vleft) / wheel_separation
        # Vleft = Vx - (Omega-z * wheel_separation / 2)
        # Vright = Vx + (Omega-z * wheel_separation / 2)
        
        target_velocity_right = Float32()
        target_velocity_left = Float32()

        rotation_factor = message.angular.z * self.wheel_separation_meters / 2

        target_velocity_right.data = message.linear.x + rotation_factor
        target_velocity_left.data = message.linear.x - rotation_factor

        self.publisher_velocity_right.publish(target_velocity_right)
        self.publisher_velocity_left.publish(target_velocity_left)

        self.get_logger().info('/cmd_vel [Twist], linear.x={0}\tangular.z={1} --> R={2}\tL={3}'.format(\
            message.linear.x,\
            message.angular.z,\
            target_velocity_right.data,\
            target_velocity_left.data))


    

def main(args=None):

    rclpy.init(args=args)

    diff_cmdvel = DifferentialCmdVel()

    try:

        while rclpy.ok():
            rclpy.spin_once(diff_cmdvel, timeout_sec=0.01)
            # do things
        
    except KeyboardInterrupt:
        self.get_logger().info('Keyboard interrupt exception was caught. Shutting down.')
        pass

    diff_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
