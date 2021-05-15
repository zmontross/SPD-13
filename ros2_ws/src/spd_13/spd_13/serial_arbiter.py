## TODO gross eyesore license agreement

# TODO Bug: Occasionally motor messages fail to parse. Observed to happen with accel/gyro too.

import serial

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
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
from std_msgs.msg import Int16, Int32
from geometry_msgs.msg import Vector3


## TODO function and class documentation (heredoc stuffs)

class SerialArbiter(Node):

    def __init__(self):
        super().__init__('serial_arbiter')

        # Declare Parameters and default values
        self.declare_parameter('serial_port_path', '/dev/ttyACM0')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('accelerometer_publish_period', 0.25)
        self.declare_parameter('gyroscope_publish_period', 0.25)
        self.declare_parameter('encoder_publish_period', 0.05)
        self.declare_parameter('motor_publish_period', 0.05)


        # Store parameter values and log them to file
        self.serial_port_path = self.get_parameter('serial_port_path').value
        self.serial_baud_rate = self.get_parameter('serial_baud_rate').value
        self.accelerometer_publish_period = self.get_parameter('accelerometer_publish_period').value
        self.gyroscope_publish_period = self.get_parameter('gyroscope_publish_period').value
        self.encoder_publish_period = self.get_parameter('encoder_publish_period').value
        self.motor_publish_period = self.get_parameter('motor_publish_period').value

        self.get_logger().info('Serial Port path: {}'.format(self.serial_port_path))
        self.get_logger().info('Serial Port baud rate: {}'.format(self.serial_baud_rate))
        self.get_logger().info('Accelerometer publish period (sec): {}'.format(self.accelerometer_publish_period))
        self.get_logger().info('Gyroscope publish period (sec): {}'.format(self.gyroscope_publish_period))
        self.get_logger().info('Encoder publish period (sec): {}'.format(self.encoder_publish_period))
        self.get_logger().info('Motor publish period (sec): {}'.format(self.motor_publish_period))



        # Variables for storing latest responses from serial.

        self.serial_guard = self.create_guard_condition(self.serial_guard_cb)
        self.serial_guard_flag = False

        self.arduino = SerialComms(self.serial_port_path, self.serial_baud_rate)
        self.arduino.reset_encoders()

        self.accelerometer_latest = ['', '', '']
        self.gyroscope_latest = ['', '', '']
        # self.encoders_latest = ['', '']
        # self.motors_latest = ['', '']
        

        # Flags and data set by subscriptions.
        # Set Motor Power
        self.update_motor_power_right = False
        self.update_motor_power_left = False
        self.requested_motor_power_right = 0
        self.requested_motor_power_left = 0


        # Setup publishers, subscribers
        self.accelerometer_pub = self.create_publisher(Vector3, 'accelerometer', 10)
        self.gyroscope_pub = self.create_publisher(Vector3, 'gyroscope', 10)
        
        self.encoder_right_pub = self.create_publisher(Int32, 'encoder_right', 10)
        self.encoder_left_pub = self.create_publisher(Int32, 'encoder_left', 10)

        self.motor_right_pub = self.create_publisher(Int16, 'motor_power_right', 10)
        self.motor_left_pub = self.create_publisher(Int16, 'motor_power_left', 10)

        self.subscriber_motor_right = self.create_subscription(Int16, 'set_motor_power_right', self.subscribed_motor_right_cb, 10)
        self.subscriber_motors_left = self.create_subscription(Int16, 'set_motor_power_left', self.subscribed_motor_left_cb, 10)

        self.timer_publish_accelerometer = self.create_timer(self.accelerometer_publish_period, self.publish_accelerometer_cb)
        self.timer_publish_gyroscope = self.create_timer(self.gyroscope_publish_period, self.publish_gyroscope_cb)
        self.timer_publish_encoders = self.create_timer(self.encoder_publish_period, self.publish_encoders_cb)
        self.timer_publish_motors = self.create_timer(self.motor_publish_period, self.publish_motors_cb)

        self.add_on_set_parameters_callback(self.parameters_callback)

    def serial_guard_cb(self):

        self.serial_guard_flag = True

    def publish_accelerometer_cb(self):
        
        try:
            a = Vector3()
            a.x, a.y, a.z = self.arduino.query_accelerometer()
            self.accelerometer_pub.publish(a)
            self.get_logger().debug('Accel:\t{} {} {}'.format(a.x, a.y, a.z))

        except Exception as e:
            self.get_logger().error("Failed to parse Accelerometer message: '{}'".format(e))
            pass

    def publish_gyroscope_cb(self):

        try:
            g = Vector3()
            g.x, g.y, g.z = self.arduino.query_gyroscope()
            self.gyroscope_pub.publish(g)
            self.get_logger().debug('Gyro:\t{} {} {}'.format(g.x, g.y, g.z))

        except Exception as e:
            self.get_logger().error("Failed to parse Gyroscope message: '{}'".format(e))
            pass

    def publish_encoders_cb(self):

        try:
            self.encoder_right = Int32()
            self.encoder_left = Int32()
            self.encoder_right.data, self.encoder_left.data = self.arduino.query_encoders()
            self.encoder_right_pub.publish(self.encoder_right)
            self.encoder_left_pub.publish(self.encoder_left)
            self.get_logger().debug('Enc:\t{} {}'.format(self.encoder_right.data, self.encoder_left.data))
            
        except Exception as e:
            self.get_logger().error("Failed to parse Encoder message: '{}'".format(e))
            pass

    def publish_motors_cb(self):

        try:
            self.motor_right = Int16()
            self.motor_left = Int16()
            self.motor_right.data, self.motor_left.data = self.arduino.query_motors()
            self.motor_right_pub.publish(self.motor_right)
            self.motor_left_pub.publish(self.motor_left)
            self.get_logger().debug('Motor:\t{} {}'.format(self.motor_right.data, self.motor_left.data))

        except Exception as e:
            self.get_logger().error("Failed to parse Motor message: '{}'".format(e))
            pass

    def subscribed_motor_right_cb(self, message):

        try:
            self.arduino.set_motor_power(self.arduino.RIGHT, int(message.data))
        except Exception as e:
            self.get_logger().error("Failed to update Motor 1 power: '{}'".format(e))
            pass
   
    def subscribed_motor_left_cb(self, message):

        try:
            self.arduino.set_motor_power(self.arduino.LEFT, int(message.data))
        except Exception as e:
            self.get_logger().error("Failed to update Motor 2 power: '{}'".format(e))
            pass

    def parameters_callback(self, params):
        # Reference, https://roboticsbackend.com/ros2-rclpy-parameter-callback/
        success = False
        for param in params:
            if param.name == 'data_publish_period':
                self.data_publish_period = param.value
                self.timer_publish_all.timer_period_ns = self.data_publish_period * S_TO_NS
                success = True
        return SetParametersResult(successful=success)


class SerialComms():    
    RIGHT = 1
    LEFT = 2
    MAX_BYTES = 20
    TERMINATOR = '\r'

    def __init__(self, serial_port_path, serial_baud_rate, logger=None):

        self.serial_port = serial.Serial(serial_port_path, serial_baud_rate, timeout=None)
        self.logger = logger

    def serial_send(self, tx):
        # Clear buffers
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # Write given input as raw bytes; Arduino code is expected to handle any possible input.
        self.serial_port.write(bytes(tx + self.TERMINATOR, 'utf-8'))
        
        # Grab raw bytes up to the pre-agreed terminator.
        rx = self.serial_port.read_until(bytes(self.TERMINATOR, 'utf-8'), self.MAX_BYTES)

        # Convert bytes to string, remove expected terminator.
        rx = rx.decode('utf-8').strip(self.TERMINATOR)

        return rx    
    
    def query_accelerometer(self):
        _, ax, ay, az = self.serial_send('qa').split(':')
        return float(ax), float(ay), float(az)

    def query_gyroscope(self):
        _, gx, gy, gz = self.serial_send('qg').split(':')
        return float(gx), float(gy), float(gz)

    def query_encoders(self):
        _, right, left = self.serial_send('qe').split(':')
        return int(right), int(left)
       
    def query_motors(self):
        _, right, left = self.serial_send('qm').split(':')
        return int(right), int(left)

    def reset_encoder_right(self):
        rx = self.serial_send('xe1')

    def reset_encoder_left(self):
        rx = self.serial_send('xe2')

    def reset_encoders(self):
        self.reset_encoder_right()
        self.reset_encoder_left()

    def reset_imu(self):
        rx = self.serial_send('xi')

    def beep(self, num):
        ## TODO Expland beeps for length, duration, multiple notes, etc. (requires updating Arduino code)
        rx = self.serial_send('b{}'.format(num))

    def set_motor_power(self, motor, power):
        rx = self.serial_send('sm{} {}'.format(motor, power))

def main(args=None):

    rclpy.init(args=args)

    arbiter = SerialArbiter()
 
    arbiter.arduino.beep(1) # Ready!

    try:
        while rclpy.ok():
            # Process callbacks/etc.
            rclpy.spin_once(arbiter, timeout_sec=0.1)

    except KeyboardInterrupt:
        arbiter.get_logger().info("Keyboard Interrupt")
        pass

    except Exception as e:
        arbiter.get_logger().error(str(e))
        pass



    # Explicitly destroy node
    arbiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
