## TODO gross eyesore license agreement

## TODO Credit https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py


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

        super().__init__('PID_Velocity')

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

        self.declare_parameter('velocity_threshold', 0.001)
        self.declare_parameter('num_velocity_samples', 5)

        self.declare_parameter('kp', 2)
        self.declare_parameter('ki', 1)
        self.declare_parameter('kd', 0)
        self.declare_parameter('pid_frequency_hz', 50)


        self.encoder_count_minimum = self.get_parameter('encoder_count_minimum').value
        self.encoder_count_maximum = self.get_parameter('encoder_count_maximum').value
        self.encoder_counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        self.wheel_diameter_meters = self.get_parameter('wheel_diameter_meters').value
        self.wheel_separation_meters = self.get_parameter('wheel_separation_meters').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.num_velocity_samples = self.get_parameter('num_velocity_samples').value
        self.motor_power_minimum = self.get_parameter('motor_power_minimum').value
        self.motor_power_maximum = self.get_parameter('motor_power_maximum').value

        self.pid_kp = self.get_parameter('kp').value
        self.pid_ki = self.get_parameter('ki').value
        self.pid_kd = self.get_parameter('kd').value
        self.pid_step_dt = (1 / self.get_parameter('pid_frequency_hz').value)
        


        # Compute miscellanous information
        self.encoder_counts_per_meter = self.encoder_counts_per_rev / (self.wheel_diameter_meters * pi)
        
        self.velocity_setpoint = 0.0
        self.velocity_samples = SimpleIntegerCircularBuffer(self.num_velocity_samples)
        self.velocity_mean = 0.0    # a.k.a The PID "PV" or "Plant Value"

        self.encoder_count_meters_latest = 0.0
        self.encoder_count_meters_previous = 0.0

        self.time_current = self.get_clock().now()
        self.time_previous = self.time_current
        self.dt = self.time_current - self.time_previous


        self.error_integral = 0.0
        self.error_last = 0.0


        self.subscriber_encoder_count = self.create_subscription(Int32, "encoder", self.encoder_callback, 10)
        self.subscriber_velocity_setpoint = self.create_subscription(Float32, "velocity_setpoint", self.velocity_setpoint_callback, 10)

        self.publisher_motor_power = self.create_publisher(Int16, "motor_power", 10)

        self.pid_timer = self.create_timer(self.pid_step_dt, self.pid_callback)
        

    def encoder_callback(self, message):
        """
        Shuffle/record encoder counts, converting the latest reading into meters.
        """

        self.encoder_count_meters_previous = self.encoder_count_meters_latest

        self.encoder_count_meters_latest = message.data / self.encoder_counts_per_meter


    def velocity_setpoint_callback(self, message):
        """
        Record commanded velocity (meters-per-second)
        """
        #TODO Ticks since Target
        
        self.velocity_setpoint = message.data


    def pid_callback(self):

        self.dt = self.pid_timer.timer_period_ns / 1000000000 # nanosec to sec = 1e-9

        self.calculate_velocity()

        pid_output = self.pid_step()

        pid_output = clamp(pid_output, self.motor_power_minimum, self.motor_power_maximum)

        motor_power= Int16()

        motor_power.data = int(pid_output)

        self.publisher_motor_power.publish(motor_power)


    def calculate_velocity(self):

        instantaneous_velocity = 0.0

        if(self.encoder_count_meters_latest == self.encoder_count_meters_previous):

            instantaneous_velocity = (1 / self.encoder_counts_per_meter) / self.dt

        else:

            instantaneous_velocity = (self.encoder_count_meters_latest - self.encoder_count_meters_previous) / self.dt

        self.velocity_samples.shift(instantaneous_velocity)

        self.velocity_mean = self.velocity_samples.mean()

        self.get_logger().info("Enc. Latest={0:.4f}m\tPrev.={1:.4f}m \tVelocity\tInst.= {2:.4f}m/s\tAvg.= {3:.4f}m/s".format(self.encoder_count_meters_latest, self.encoder_count_meters_previous, instantaneous_velocity, self.velocity_mean))


    def pid_step(self):

        # TODO Integral-windup protection

        error = self.velocity_setpoint - self.velocity_mean

        self.error_integral = self.error_integral + (error * self.dt)

        error_derivative = (error - self.error_last) / self.dt
        self.error_last = error

        pid_output = (self.pid_kp * error)\
                    + (self.pid_ki * self.error_integral)\
                    + (self.pid_kd * error_derivative)

        # self.get_logger().info("PID, 1/2 ~ SP={0}m/s\tPV={1}m/s\tError={2}\tInteg.={3}\tDeriv.={4}".format(
        #     self.velocity_setpoint, self.velocity_mean, error, self.error_integral, error_derivative
        # ))
        # self.get_logger().info("PID, 2/2 ~ Kp={0}\tKi={1}\tKd={2}\t\tOUTPUT={3}".format(
        #     self.pid_kp, self.pid_ki, self.pid_kd, pid_output
        # ))

        return pid_output


class SimpleIntegerCircularBuffer():
    """
    Very basic implementation of a circular buffer of a fixed size using a Numpy array.
    This allows for the use of Numpy array methods such as 'mean()'.
    This class also allows the "latest index" logic to be contained so as to keep everything else clean.
    No shallow-copying or array-size shifting is performed. Instead the oldest element is tracked with an index value,
    and this index is updated each time a value is added.
    Elements are initialized to zero on instantiation.
    """

    def __init__(self, size=1):

        self._buffer = array([0] * size)
        self._index_oldest = 0
    
    def size(self):

        return self._buffer.size
    
    def get(self):

        return self._buffer

    def mean(self):

        return self._buffer.mean()

    def zero_clear(self):
        """
        Overwrites every array element with a zero
        """

        self._buffer.fill(0)

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
        self.get_logger().info('Keyboard interrupt exception was caught. Shutting down.')
        pass

    pid_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
