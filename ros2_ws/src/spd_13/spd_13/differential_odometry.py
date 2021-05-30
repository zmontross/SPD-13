## TODO gross eyesore license agreement

## TODO Credit https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py

## TODO Credit https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

## TODO Credit http://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from rclpy.constants import S_TO_NS

from std_msgs.msg import Int32
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DifferentialOdometry(Node):
    
    def __init__(self):

        super().__init__('differential_odometry')

        # TODO Relocate robot physical characteristics to an URDF file

        # Declare Parameters
        self.declare_node_parameters()

        # Retrieve/store initial param values
        self.encoder_count_minimum = self.get_parameter('encoder_count_minimum').value
        self.encoder_count_maximum = self.get_parameter('encoder_count_maximum').value
        self.encoder_counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        self.wheel_diameter_meters = self.get_parameter('wheel_diameter_meters').value
        self.wheel_separation_meters = self.get_parameter('wheel_separation_meters').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_period_sec = self.get_parameter('publish_period_sec').value

        self.get_logger().info("Encoder count minimum: {}".format(self.encoder_count_minimum))
        self.get_logger().info("Encoder count maximum: {}".format(self.encoder_count_maximum))
        self.get_logger().info("Encoder counts per rev: {}".format(self.encoder_counts_per_rev))
        self.get_logger().info("Wheel diameter (m): {}".format(self.wheel_diameter_meters))
        self.get_logger().info("Wheel separation (m): {}".format(self.wheel_separation_meters))
        self.get_logger().info("Base frame ID: {}".format(self.base_frame_id))
        self.get_logger().info("Odom frame ID: {}".format(self.odom_frame_id))
        self.get_logger().info("Publish period (s): {}".format(self.publish_period_sec))
        
        self.encoder_counts_per_meter = self.encoder_counts_per_rev / (self.wheel_diameter_meters * pi)
        
        self.encoder_left = 0
        self.encoder_left_last = 0
        self.encoder_right = 0
        self.encoder_right_last = 0

        self.time_previous = self.get_clock().now()

        self.position_x = 0.0 # X-coord in XY plane
        self.position_y = 0.0 # Y-coord in XY plane
        self.position_theta = 0.0 # Theta angle in XY plane

        self.velocity_linear = 0.0 # meters per second
        self.velocity_angular = 0.0 # radians per second

        self.qos = QoSProfile(
            # Based on ROBOTIS Turtlebot3 usage,
            # qos.py initialization values,
            # and ROS2 Foxy documentation, https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )
        
        self.publisher_odom = self.create_publisher(Odometry, self.odom_frame_id, self.qos)        
        self.transform_broadcaster = TransformBroadcaster(self)

        self.subscriber_encoder_right = self.create_subscription(Int32, 'encoder_right', self.encoder_right_cb, self.qos)
        self.subscriber_encoder_left = self.create_subscription(Int32, 'encoder_left', self.encoder_left_cb, self.qos)
        
        self.update_timer = self.create_timer(self.publish_period_sec, self.update_cb)

        self.get_logger().info('Differential Odometry node initialized!')


    def declare_all_parameters(self):
        """
        Declare all of the parameters for this node. Default parameter values are set here.
        """

        self.declare_parameter(
            name = 'encoder_count_minimum',
            descriptor = 'Absolute minimum encoder value (default is signed-int32 minimum).',
            value = -2147483648
        )

        self.declare_parameter(
            name = 'encoder_count_maximum',
            descriptor = 'Absolute maximum encoder value (default is signed-int32 maximum).',
            value = 2147483647
        )

        self.declare_parameter(
            name = 'encoder_counts_per_rev',
            descriptor = 'Number of encoder counts per motor shaft revolution. Default is provided by Pololu DC motor 4825: https://www.pololu.com/product/4825',
            value = 2248.86
        )

        self.declare_parameter(
            name = 'wheel_diameter_meters',
            descriptor = 'Diameter, in meters, of the wheel attached to the drive motor. Default measured via calipers: https://www.dfrobot.com/product-1477.html',
            value = 0.04186
        )

        self.declare_parameter(
            name = 'wheel_separation_meters',
            descriptor = 'Distance, in meters, between between differential wheel centers. Default measured via CAD: https://grabcad.com/library/dfrobot-devastator-1',
            value = 0.1875
        )

        self.declare_parameter(
            name = 'base_frame_id',
            descriptor = 'Name of the robot TF base_frame/base_link/etcetera. Used for TF2 and Odometry messages.',
            value = 'base_frame'
        )

        self.declare_parameter(
            name = 'odom_frame_id',
            descriptor = 'Name of the robot TF odometry. Used for TF2 and Odometry messages.',
            value = 'odom'
        )

        self.declare_parameter(
            name = 'publish_period_sec',
            descriptor = 'Period, in seconds, between TF / Odometry message publishes.',
            value = 0.01
        )

    # def joint_state_cb(self):
    #     print("joint_state_callback!")


    def encoder_left_cb(self, message):

        self.encoder_left_last = self.encoder_left
        self.encoder_left = message.data


    def encoder_right_cb(self, message):

        self.encoder_right_last = self.encoder_right
        self.encoder_right = message.data

    
    def update_cb(self):

        time_current = self.get_clock().now()
        dt = time_current - self.time_previous
        self.time_previous = time_current

        if self.encoder_left == 0:  # Calculate distance covered by LEFT motor
            distance_left = 0.0
        else:
            distance_left = (self.encoder_left - self.encoder_left_last) / self.encoder_counts_per_meter
        
        if self.encoder_right == 0: # Calculate distance covered by RIGHT motor
            distance_right = 0.0
        else:
            distance_right = (self.encoder_right - self.encoder_right_last) / self.encoder_counts_per_meter

        self.encoder_left_last = self.encoder_left
        self.encoder_right_last = self.encoder_right


        distance_average = (distance_left + distance_right) / 2   # Calculation dependent only on latest two encoder counts
        theta = (distance_right - distance_left) / self.wheel_separation_meters # "this approximation works (in radians) for small angles" TODO double-check this estimation

        
        if distance_average != 0:
            # Calculate distance traveled in X and Y
            distance_x = distance_average * cos(theta)
            distance_y = distance_average * -sin(theta)

            # Calculate final position of robot
            self.position_x = self.position_x + (distance_x * cos(self.position_theta)) + (distance_y * -sin(self.position_theta))  # Note the negative Sin()
            self.position_y = self.position_y + (distance_y * sin(self.position_theta)) + (distance_y * cos(self.position_theta))
        
        if theta != 0:
            self.position_theta = self.position_theta + theta

        if dt.nanoseconds == 0: # Purely for initial-conditions / reset protection
            self.velocity_linear = 0.0
            self.velocity_angular = 0.0
        else:
            self.velocity_linear = distance_average / (dt.nanoseconds / S_TO_NS)
            self.velocity_angular = theta / (dt.nanoseconds / S_TO_NS)

        # Publish transform, odometry information
        point = Point(
            x = self.position_x,
            y = self.position_y,
            z = 0.0
        )

        quaternion = Quaternion(
            x = 0.0,
            y = 0.0,
            z = sin(self.position_theta / 2), # TODO Learning: Review why these angles are halved; x -> qxq* for vector 'x', quat. 'q'
            w = cos(self.position_theta / 2)
        )

        header = Header(
            stamp = time_current.to_msg(),
            frame_id = self.odom_frame_id
        )

        pose = Pose(
            position = point,
            orientation = quaternion
        )

        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = self.base_frame_id
        transform.transform = pose

        odom = Odometry()
        odom.header = header
        odom.child_frame_id = self.base_frame_id
        odom.pose = pose
        odom.twist.twist.linear.x = self.velocity_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.velocity_angular

        self.transform_broadcaster.sendTransform(transform)
        self.publisher_odom.publish(odom)


def main(args=None):

    rclpy.init(args=args)

    diff_odom = DifferentialOdometry()

    try:
        while rclpy.ok():
            rclpy.spin_once(diff_odom, timeout_sec=0.01)
            # do things

    except KeyboardInterrupt:
        diff_odom.get_logger().info("Keyboard Interrupt")
        pass
                
    except Exception as e:
        print(e)
        pass

    diff_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
