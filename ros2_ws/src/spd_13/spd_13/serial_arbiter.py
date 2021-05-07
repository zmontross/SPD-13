## TODO gross eyesore license agreement

# TODO Bug: Occasionally motor messages fail to parse. Observed to happen with accel/gyro too.

import serial

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, SetParametersResult


## TODO Resolve custom message problems.
## At time of writing there are runtime problems related to C for using custom messages in ament_python projects.
## Ideally we would use a custom message type to enforce proper use of published data.
## See:
## - https://answers.ros.org/question/373256/ros2-could-not-import-rosidl_typesupport_c-for-package/
## - https://answers.ros.org/question/350084/define-custom-messages-in-python-package-ros2/
## - https://www.gitmemory.com/issue/ros2/ros2/1101/795386484
## - https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html
## - https://docs.ros.org/en/foxy/Tutorials/Single-Package-Define-And-Use-Interface.html
##
from std_msgs.msg import String, Int16, Int32


## TODO function and class documentation (heredoc stuffs)
class SerialCommands():
    # Pololu Astar Arduino Commands
    # Queries
    query_accelerometer = 'qa'
    query_gyroscope = 'qg'
    query_encoders = 'qe'
    query_motors = 'qm'
    # Resets
    reset_imu = 'xi'     # IMU == Accelerometer and Gyroscope
    reset_enc1 = 'xe1'   # Encoder 1 count
    reset_enc2 = 'xe2'   # Encoder 2 count
    ## TODO add reference to motor-speed-set commands (sm1 xxxx)
    # Miscellaneous - Command Character Echo Toggle
    echo_toggle = '@'
    # Miscellaneous - Beeps
    do_beep1 = 'b1'
    do_beep2 = 'b2'
    do_beep3 = 'b3'
    do_beep4 = 'b4'
    ## TODO Expland beeps for length, duration, multiple notes, etc. (requires updating Arduino code)

    MAX_BYTES = 20


class SerialArbiter(Node):

    def __init__(self):
        super().__init__('serial_arbiter')

        # Declare Parameters and default values
        self.declare_parameter('serial_port_path', '/dev/ttyACM0')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('data_publish_period', 0.01)


        # Store parameter values and log them to file
        self.serial_port_path = self.get_parameter('serial_port_path').value
        self.serial_baud_rate = self.get_parameter('serial_baud_rate').value
        self.data_publish_period = self.get_parameter('data_publish_period').value

        self.get_logger().info('Serial Port path: {}'.format(self.serial_port_path))
        self.get_logger().info('Serial Port baud rate: {}'.format(self.serial_baud_rate))
        self.get_logger().info('Encoder publish period (sec): {}'.format(self.data_publish_period))


        # Variables for storing latest responses from serial.
        self.serial_rx_qa = 'ACCEL:0:0:0'
        self.serial_rx_qg = 'GYRO:0:0:0'
        self.serial_rx_qe = 'ENC:0:0'
        self.serial_rx_qm = 'MOTOR:0:0'


        # Flags and data set by subscriptions.
        # Set Motor Power
        self.update_motor_power_right = False
        self.update_motor_power_left = False
        self.requested_motor_power_right = 0
        self.requested_motor_power_left = 0


        # Setup publishers, subscribers
        self.publisher_accel = self.create_publisher(String, 'accelerometer', 10)
        self.publisher_gyro = self.create_publisher(String, 'gyro', 10)
        
        self.publisher_enc_right = self.create_publisher(Int32, 'encoder_right', 10)
        self.publisher_enc_left = self.create_publisher(Int32, 'encoder_left', 10)

        self.publisher_motor_right = self.create_publisher(Int16, 'motor_power_right', 10)
        self.publisher_motor_left = self.create_publisher(Int16, 'motor_power_left', 10)

        self.subscriber_motor_right = self.create_subscription(Int16, 'set_motor_power_right', self.set_motor_power_right_callback, 10)
        self.subscriber_motor_left = self.create_subscription(Int16, 'set_motor_power_left', self.set_motor_power_left_callback, 10)


        # Node-specific setup
        ## TODO wrap with try/catch, log initial serial connection error
        self.serial_port = serial.Serial(self.serial_port_path, self.serial_baud_rate, timeout=2)
        
        self.timer_publish_all = self.create_timer(self.data_publish_period, self.publish_all_callback)     # analgous to rclcpp::Node::create_wall_timer()

        self.add_on_set_parameters_callback(self.parameters_callback)


    
    def publish_all_callback(self):

        # Poll data from serial, store responses.
        self.serial_rx_qa =  self.serial_send(SerialCommands.query_accelerometer)
        self.serial_rx_qg =  self.serial_send(SerialCommands.query_gyroscope)
        self.serial_rx_qe =  self.serial_send(SerialCommands.query_encoders)
        self.serial_rx_qm =  self.serial_send(SerialCommands.query_motors)

        # Update motor speeds if requested by subscriptions.
        try:
            if self.update_motor_power_right:
                self.update_motor_power_right = False
                self.serial_send("sm1 %d" % self.requested_motor_power_right)
        except:
            self.get_logger().warning("Failed to update Motor 1 power.")
            pass
        
        try:
            if self.update_motor_power_left:
                self.update_motor_power_left = False
                self.serial_send("sm2 %d" % self.requested_motor_power_left)
        except:
            self.get_logger().warning("Failed to update Motor 2 power.")
            pass

        # Accelerometer
        try:
            accel_msg = String()
            _, ax, ay, az = self.serial_rx_qa.split(':')
            accel_msg.data = "x={}|y={}|z={}".format(ax, ay, az)
            self.publisher_accel.publish(accel_msg)
            self.get_logger().debug('Publishing Acceleration, %s' % accel_msg.data)
        except:
            self.get_logger().warning("Failed to parse Accelerometer message.")
            pass

        # Gyroscope
        try:
            gyro_msg = String()
            _, gx, gy, gz = self.serial_rx_qg.split(':')
            gyro_msg.data = "x={}|y={}|z={}".format(gx, gy, gz)
            self.publisher_accel.publish(gyro_msg)
            self.get_logger().debug('Publishing Gyroscope, %s' % gyro_msg.data)
        except:
            self.get_logger().warning("Failed to parse Gyroscope message.")
            pass

        # Encoders
        try:
            enc_right_msg = Int32()
            enc_left_msg = Int32()
            _, enc_right_msg_temp, enc_left_msg_temp = self.serial_rx_qe.split(':')
            enc_right_msg.data = int(enc_right_msg_temp)
            enc_left_msg.data = int(enc_left_msg_temp)
            self.publisher_enc_right.publish(enc_right_msg)
            self.publisher_enc_left.publish(enc_left_msg)
            self.get_logger().debug('Publishing Encoders, R[{}] L[{}]'.format(enc_right_msg.data, enc_left_msg.data))
        except:
            self.get_logger().warning("Failed to parse Encoder message.")
            pass

        # Motors
        try:
            motor_right_msg = Int16()
            motor_left_msg = Int16()
            _, motor_right_msg_temp, motor_left_msg_temp = self.serial_rx_qm.split(':')
            motor_right_msg.data = int(motor_right_msg_temp)
            motor_left_msg.data = int(motor_left_msg_temp)
            self.publisher_motor_right.publish(motor_right_msg)
            self.publisher_motor_left.publish(motor_left_msg)
            self.get_logger().debug('Publishing Motor Power, R[{}] L[{}]'.format(motor_right_msg.data, motor_left_msg.data))
        except:
            self.get_logger().warning("Failed to parse Motor message.")
            pass


    def parameters_callback(self, params):
        # Reference, https://roboticsbackend.com/ros2-rclpy-parameter-callback/
        success = False
        for param in params:
            if param.name == 'data_publish_period':
                self.data_publish_period = param.value
                self.timer_publish_all.timer_period_ns = self.data_publish_period * 1000000000.0   # Seconds-to-Nanoseconds
                success = True
        return SetParametersResult(successful=success)

    def set_motor_power_right_callback(self, message):
        self.requested_motor_power_right = int(message.data)
        self.get_logger().debug('Subscription rx, Right Motor Power, {}'.format(self.requested_motor_power_right))
        self.update_motor_power_right = True
    
    def set_motor_power_left_callback(self, message):
        self.requested_motor_power_left = int(message.data)
        self.get_logger().debug('Subscription rx, Left Motor Power, {}'.format(self.requested_motor_power_left))
        self.update_motor_power_left = True


    def serial_send(self, tx):
        # Clear buffers
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # Write given input as raw bytes; Arduino code is expected to handle any possible input.
        self.serial_port.write(bytes(tx + '\r', 'utf-8'))

        # Grab raw bytes up to the pre-agreed terminator.
        rx = self.serial_port.read_until(bytes('\r', 'utf-8'), SerialCommands.MAX_BYTES)

        # Convert bytes to string, remove expected terminator.
        rx = rx.decode('utf-8').strip('\r')

        return rx


def main(args=None):

    rclpy.init(args=args)

    arbiter = SerialArbiter()

    arbiter.serial_send('\r\r') # Send some terminating chars to properly synchronize with other side; "spam the Enter-key"
    arbiter.serial_send(SerialCommands.reset_enc1)
    arbiter.serial_send(SerialCommands.reset_enc2)
    arbiter.serial_send('sm1 0') # Stop motor 1
    arbiter.serial_send('sm2 0') # Stop motor 2
    arbiter.serial_send(SerialCommands.do_beep4) # Beep to alert, notify of running state

    try:
        while rclpy.ok():
            # Process callbacks/etc.
            rclpy.spin_once(arbiter, timeout_sec=0.01)  # Timeout req. to decouple loop from publish freq., nonzero to help subs.

    except Exception as e:
        arbiter.get_logger().error(e)
        pass

    try:
        arbiter.serial_send('sm1 0') # Stop motor 1
        arbiter.serial_send('sm2 0') # Stop motor 2
    except:
        pass

    # Explicitly destroy node
    arbiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
