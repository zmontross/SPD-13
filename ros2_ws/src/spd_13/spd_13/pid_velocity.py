## TODO gross eyesore license agreement

## TODO Credit https://github.com/jfstepha/differential-drive/blob/master/scripts/py


from numpy import array

from math import pi

import rclpy

from rclpy.node import Node

from std_msgs.msg import Int16, Int32, Float32


# Need to convert incoming velocity, meters-per-second, into ticks-per-second

# Node will output a Motor Power value, -400 to +400
    # NEED LINK BETWEEN MOTOR POWER AND TICKS-PER-SECOND

# Motor Power is published by serial_arbiter
    # Use current power to inc/dec motor power

# Motor power values are ABSOLUTE values
    # We don't need to use negative values to slow-down; just successive smaller values.


class PidVelocity(Node):

    def __init__(self):

        super().__init__('pid_velocity_controller')

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
        self.declare_parameter('motor_power_minimum', -400)
        self.declare_parameter('motor_power_maximum', 400)
        self.declare_parameter('motor_power_maximum_step', 25)

        self.declare_parameter('velocity_threshold', 0.001)
        self.declare_parameter('num_velocity_samples', 5)

        self.declare_parameter('kp', 2)
        self.declare_parameter('ki', 1)
        self.declare_parameter('kd', 0)
        self.declare_parameter('pid_frequency_hz', 5)
        self.declare_parameter('pid_integral_windup_limit', 0.025)
        self.declare_parameter('pid_output_limit', 0.1)

        self.declare_parameter('motor_velocity_power_factor', 0.000245) # constant multiplier for motor power; delta-V per unit of motor power


        self.encoder_count_minimum = self.get_parameter('encoder_count_minimum').value
        self.encoder_count_maximum = self.get_parameter('encoder_count_maximum').value
        self.encoder_counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        self.wheel_diameter_meters = self.get_parameter('wheel_diameter_meters').value
        self.wheel_separation_meters = self.get_parameter('wheel_separation_meters').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.num_velocity_samples = self.get_parameter('num_velocity_samples').value
        self.motor_power_minimum = self.get_parameter('motor_power_minimum').value
        self.motor_power_maximum = self.get_parameter('motor_power_maximum').value
        self.motor_power_maximum_step = self.get_parameter('motor_power_maximum_step').value

        self.pid_kp = self.get_parameter('kp').value
        self.pid_ki = self.get_parameter('ki').value
        self.pid_kd = self.get_parameter('kd').value
        self.pid_step_dt = (1 / self.get_parameter('pid_frequency_hz').value)
        self.pid_integral_windup_limit = self.get_parameter('pid_integral_windup_limit').value
        self.pid_output_limit = self.get_parameter('pid_output_limit').value

        self.motor_velocity_power_factor = self.get_parameter('motor_velocity_power_factor').value


        # Compute miscellanous information
        self.encoder_counts_per_meter = self.encoder_counts_per_rev / (self.wheel_diameter_meters * pi)
        
        self.velocity_setpoint = 0.0
        self.velocity_samples = SimpleCircularBuffer(self.num_velocity_samples)
        self.velocity_mean = 0.0    # a.k.a The PID "PV" or "Plant Value"

        self.encoder_count_meters_latest = 0.0
        self.encoder_count_meters_previous = 0.0

        self.motor_power_latest = 0

        self.time_current = self.get_clock().now()
        self.time_previous = self.time_current
        self.dt = 0.000001  # Non-zero starting delta for initialization, 1us = 1e-6 sec


        self.pid_error_samples = SimpleCircularBuffer(self.num_velocity_samples)
        self.pid_output = 0.0

        self.get_logger().info("PID Constants: Kp={}, Ki={}, Kd={}".format(self.pid_kp, self.pid_ki, self.pid_kd))

        self.subscriber_encoder_count = self.create_subscription(Int32, "encoder", self.encoder_callback, 10)
        self.subscriber_motor_power = self.create_subscription(Int16, "motor_power", self.motor_power_callback, 10)
        self.subscriber_velocity_setpoint = self.create_subscription(Float32, "velocity_setpoint", self.velocity_setpoint_callback, 10)

        self.publisher_motor_power = self.create_publisher(Int16, "set_motor_power", 10)

        self.pid_timer = self.create_timer(self.pid_step_dt, self.pid_callback)
        

    def encoder_callback(self, message):
        """
        Shuffle/record encoder counts, converting the latest reading into meters.
        """
        self.time_previous = self.time_current
        self.encoder_count_meters_previous = self.encoder_count_meters_latest

        self.time_current = self.get_clock().now()
        self.encoder_count_meters_latest = message.data / self.encoder_counts_per_meter

        self.dt = (self.time_current.nanoseconds - self.time_previous.nanoseconds) / 1000000000.0 # nanosec to sec = 1e-9

    def motor_power_callback(self, message):

        self.motor_power_latest = message.data


    def velocity_setpoint_callback(self, message):
        """
        Record commanded velocity (meters-per-second)
        """

        self.velocity_setpoint = message.data


    def pid_callback(self):

        self.calculate_velocity()

        self.pid_step()

        motor_power = Int16()        
        motor_power.data = self.pid_to_motors(self.pid_output)
        self.publisher_motor_power.publish(motor_power)

        self.get_logger().info('\tSP: {0:.6f}\tPV: {1:.6f}\tPID: {2:.6f}\tMotor: {3}'.format(
            self.velocity_setpoint, self.velocity_mean, self.pid_output, motor_power.data
            )
        )


    def calculate_velocity(self):

        instantaneous_velocity = 0.0

        if(self.encoder_count_meters_latest != self.encoder_count_meters_previous):

            instantaneous_velocity = (self.encoder_count_meters_latest - self.encoder_count_meters_previous) / self.dt

        self.velocity_samples.shift(value=instantaneous_velocity)

        self.velocity_mean = self.velocity_samples.mean()


    def pid_step(self):
        """
        Single time-step of the PID algorithm.
        Errors are stored in a circular buffer after calculating current-time-step PID components.
        """
        # TODO Credit https://maldus512.medium.com/pid-control-explained-45b671f10bc7

        error = self.velocity_setpoint - self.velocity_mean

        error_integral = self.pid_error_samples.sum() + (error * self.dt)
        error_integral = clamp(error_integral, -self.pid_integral_windup_limit, self.pid_integral_windup_limit)

        error_derivative = (error - self.pid_error_samples.peek_latest()) / self.dt

        self.pid_error_samples.shift(error)

        # Raw PID computation
        self.pid_output = (self.pid_kp * error) \
                    + (self.pid_ki * error_integral) \
                    + (self.pid_kd * error_derivative)

        # PID Output clamped within bounds of maximum possible velocity
        self.pid_output = clamp(self.pid_output, -self.pid_output_limit, self.pid_output_limit)
        if abs(self.pid_output) < 0.0001:
            self.pid_output = 0.0

    def pid_to_motors(self, pid_output=0.0):
        """
        Translates the PID output into a useful motor value.
        """
        # Convert PID output into an equivalent motor power value
        motor_power = self.motor_power_latest + min(int(pid_output // self.motor_velocity_power_factor), self.motor_power_maximum_step)

        # Power given to motors is clamped to the parameterized maximum step size.
        motor_power = clamp(motor_power, 0, self.motor_power_maximum)

        return motor_power


class SimpleCircularBuffer():
    """
    Very basic implementation of a circular buffer of a fixed size using a Numpy array.
    This allows for the use of Numpy array methods such as 'mean()'.
    This class also allows the "latest index" logic to be contained so as to keep everything else clean.
    No shallow-copying or array-size shifting is performed. Instead the oldest element is tracked with an index value,
    and this index is updated each time a value is added.
    Elements are initialized to zero on instantiation.
    """

    def __init__(self, size=1):

        self._buffer = array([0.0] * size)
        self._index_oldest = 0
    
    def size(self):

        return self._buffer.size
    
    def get(self):

        return self._buffer

    def mean(self):

        return self._buffer.mean()

    def sum(self):

        return self._buffer.sum()

    def zero_clear(self):
        """
        Overwrites every array element with a zero
        """

        self._buffer.fill(0.0)

    def shift(self, value):
        """
        Adds `value` to the numpy array by overwriting the oldest element.
        """

        self._buffer[self._index_oldest] = value
        self._index_oldest = (self._index_oldest + 1) % self._buffer.size

    def peek_latest(self):
        """
        Returns the value of the latest-added element of the numpy array.
        """
        index_latest = (self._index_oldest + self._buffer.size - 1) % self._buffer.size
        return self._buffer[index_latest]



def clamp(x, low=0, high=100):

    return max(low, min(x, high))


def main(args=None):

    rclpy.init(args=args)

    pid_velocity = PidVelocity()

    try:

        while rclpy.ok():
            rclpy.spin_once(pid_velocity, timeout_sec=0.01)
            # do things

    except KeyboardInterrupt:
        pid_velocity.get_logger().info("Keyboard Interrupt")
        pass

    except Exception as e:
        print(e)
        pass

    pid_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
