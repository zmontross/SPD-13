## TODO gross eyesore license agreement


## TODO Credit https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py

from math import sin, cos, pi


import rclpy

from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, SetParametersResult

from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from std_msgs.msg import Float32, Float64

from rclpy.time import Time


class DifferentialOdometry(Node):
    
    def __init__(self):

        super().__init__('Odometry')

        # TODO Relocate robot physical characteristics to an URDF file

        # Declare Parameters
        # Default 'encoder_count_minimum/maximum' derived from signed int32 range.
        self.declare_parameter('encoder_count_minimum', -2147483648)
        self.declare_parameter('encoder_count_maximum', 2147483647)

        # Default 'encoder_counts_per_rev' provided by https://www.pololu.com/product/4825
        self.declare_parameter('encoder_counts_per_rev', 2248.86)

        # self.declare_parameter('encoder_wrap_low', -1717986919) # ((count_max - count_min) * 0.3) + count_min
        # self.declare_parameter('encoder_wrap_high', 1717986918) # ((count_max - count_min) * 0.7) + count_min

        # Default measured via calipers, https://www.dfrobot.com/product-1477.html
        self.declare_parameter('wheel_diameter_meters', 0.04186)

        # Default measured via CAD, https://grabcad.com/library/dfrobot-devastator-1
        # Defined as the separation between the wheel centers, https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
        self.declare_parameter('wheel_separation_meters', 0.1875)

        self.declare_parameter('base_frame_id', 'base_frame')
        self.declare_parameter('odom_frame_id', 'odom')

        ##self.declare_parameter('', '')


        # Retrieve/store initial param values
        self.encoder_count_minimum = self.get_parameter('encoder_count_minimum').value
        self.encoder_count_maximum = self.get_parameter('encoder_count_maximum').value
        self.encoder_counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        # self.encoder_wrap_low = self.get_parameter('encoder_wrap_low').get_parameter_value()
        # self.encoder_wrap_high = self.get_parameter('encoder_wrap_high').get_parameter_value()
        self.wheel_diameter_meters = self.get_parameter('wheel_diameter_meters').value
        self.wheel_separation_meters = self.get_parameter('wheel_separation_meters').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        self.get_logger().info("Encoder count minimum: {}".format(self.encoder_count_minimum))
        self.get_logger().info("Encoder count maximum: {}".format(self.encoder_count_maximum))
        self.get_logger().info("Encoder counts per rev: {}".format(self.encoder_counts_per_rev))
        self.get_logger().info("Wheel diameter (m): {}".format(self.wheel_diameter_meters))
        self.get_logger().info("Wheel separation (m): {}".format(self.wheel_separation_meters))
        self.get_logger().info("Base frame ID: {}".format(self.base_frame_id))
        self.get_logger().info("Odom frame ID: {}".format(self.odom_frame_id))
        

        # Compute miscellanous information
        self.encoder_counts_per_meter = self.encoder_counts_per_rev / (self.wheel_diameter_meters * pi)
        
        # Instance variables
        self.update_frequency = 100 # Hz
        self.update_dt = 1 / self.update_frequency
        self.update_tlast = self.get_clock().now()


        self.encoder_left = 0
        self.encoder_left_last = 0
        # self.encoder_wrap_multiplier_left = 0 # Default encoders shouldn't wrap for 250km

        self.encoder_right = 0
        self.encoder_right_last = 0
        # self.encoder_wrap_multiplier_right = 0 # Default encoders shouldn't wrap for 250km

        self.position_x = 0.0 # X-coord in XY plane
        self.position_y = 0.0 # Y-coord in XY plane
        self.position_theta = 0.0 # Theta angle in XY plane

        self.velocity_linear = 0.0 # meters per second
        self.velocity_angular = 0.0 # radians per second


        # Create publishers, subscribers, timers
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        
        self.transform_broadcaster = TransformBroadcaster(self)
        
        # self.subscriber_joint_state = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) ## TODO Learning Q: Why consider "joint_states" when we can just push an Odom message directly?
        ## Robotis' TB3 odometry.cpp/hpp subs to a JointState msg, updates recorded joint positions (prev/curr) + deltas, recalcs odom using a Duration, and publishes odom/tf
        self.subscriber_encoder_right = self.create_subscription(Int32, 'encoder_right', self.encoder_right_callback, 10)
        self.subscriber_encoder_left = self.create_subscription(Int32, 'encoder_left', self.encoder_left_callback, 10)
        
        self.update_timer = self.create_timer(self.update_dt, self.update_callback)

        self.get_logger().info('Differential Odometry node initialized!')


    # def joint_state_callback(self):
    #     print("joint_state_callback!")

    def encoder_left_callback(self, message):

        self.encoder_left_last = self.encoder_left
        self.encoder_left = message.data


    def encoder_right_callback(self, message):

        self.encoder_right_last = self.encoder_right
        self.encoder_right = message.data

    
    def update_callback(self):

        current_time = self.get_clock().now()

        dt = current_time - self.update_tlast
        self.update_tlast = current_time

        # Calculate Odometry
        if self.encoder_left == 0:
            distance_left_meters = 0.0
        else:
            distance_left_meters = (self.encoder_left - self.encoder_left_last) / self.encoder_counts_per_meter

        if self.encoder_right == 0:
            distance_right_meters = 0.0
        else:
            distance_right_meters = (self.encoder_right - self.encoder_right_last) / self.encoder_counts_per_meter

        self.encoder_left_last = self.encoder_left
        self.encoder_right_last = self.encoder_right


        distance_average = (distance_left_meters + distance_right_meters) / 2

        theta = (distance_right_meters - distance_left_meters) / self.wheel_separation_meters # "this approximation works (in radians) for small angles" TODO double-check this estimation

        if dt.to_msg().sec == 0:
            self.velocity_linear = 0.0
            self.velocity_angular = 0.0
        else:
            self.velocity_linear = distance_average / dt.to_msg().sec
            self.velocity_angular = theta / dt.to_msg().sec


        if distance_average != 0:
            # Calculate distance traveled in X and Y
            distance_x = distance_average * cos(theta)
            distance_y = distance_average * -sin(theta)

            # Calculate final position of robot
            self.position_x = self.position_x + (distance_x * cos(self.position_theta)) + (distance_y * -sin(self.position_theta))  # Note the negative Sin()
            self.position_y = self.position_y + (distance_y * sin(self.position_theta)) + (distance_y * cos(self.position_theta))

        if theta != 0:
            self.position_theta = self.position_theta + theta


        # Publish odometry information
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sin(self.position_theta / 2) # TODO Learning: Review why these angles are halved; Quaternion phenomena, not ROS-specific
        q.w = cos(self.position_theta / 2)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.transform_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.velocity_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.velocity_angular

        self.publisher_odom.publish(odom)

def main(args=None):

    rclpy.init(args=args)

    diff_odom = DifferentialOdometry()

    try:

        while rclpy.ok():
            rclpy.spin_once(diff_odom, timeout_sec=0.01)
            # do things
        
    except KeyboardInterrupt:
        self.get_logger().info('Keyboard interrupt exception was caught. Shutting down.')
        pass

    diff_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
