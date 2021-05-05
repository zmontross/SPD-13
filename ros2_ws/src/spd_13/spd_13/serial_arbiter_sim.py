# serial_arbiter_sim


import rclpy

from rclpy.node import Node

from std_msgs.msg import Int32

## TODO Create 'PseudoMotor' class that can use "motor_power" and produce "encoder_ticks"

class PseudoEncoder():

    ## TODO allow passing-in an array or file or something to define a sequence of encoder increments. Simulate a path, basically.
    
    def __init__(self, ticks_per_publish=0):
        self.ticks_per_publish = ticks_per_publish
        self.encoder_ticks = 0
    
    def update(self):
        self.encoder_ticks = self.encoder_ticks + self.ticks_per_publish

    def reset(self):
        self.encoder_ticks = 0


class SerialArbiterSim(Node):

    def __init__(self):
        super().__init__('Serial Arbiter Simulator')

        self.declare_parameter('ticks_per_second', 2000)
        self.declare_parameter('encoder_publish_period_sec', 0.01)

        self.ticks_per_second = self.get_parameter('ticks_per_second').value
        self.encoder_publish_period_sec = self.get_parameter('encoder_publish_period_sec').value
        
        self.publisher_encoder_right = self.create_publisher(Int32, 'encoder_right', 10)
        self.publisher_encoder_left = self.create_publisher(Int32, 'encoder_left', 10)
        
        self.timer_publish_encoders = self.create_timer(self.encoder_publish_period_sec, self.encoder_publish_callback)


        # State variables
        self.ticks_per_publish = int(self.ticks_per_second * self.encoder_publish_period_sec)
        self.encoder_right = PseudoEncoder(self.ticks_per_publish)
        self.encoder_left = PseudoEncoder(self.ticks_per_publish)

        self.get_logger().info("Serial Arbiter Sim node started!")
        self.get_logger().info("Encoder publish period (sec): {}".format(self.encoder_publish_period_sec))
        self.get_logger().info("Encoder ticks-per-second: {}".format(self.ticks_per_second))


    def encoder_publish_callback(self):
        
        self.encoder_right.update()
        enc_right_msg = Int32()
        enc_right_msg.data = self.encoder_right.encoder_ticks
        self.publisher_encoder_right.publish(enc_right_msg)
                
        self.encoder_left.update()
        enc_left_msg = Int32()
        enc_left_msg.data = self.encoder_left.encoder_ticks
        self.publisher_encoder_left.publish(enc_left_msg)

        self.get_logger().info('Encoders\t[R]\t{}\t[L]\t{}'.format(self.encoder_right.encoder_ticks, self.encoder_left.encoder_ticks))



def main(args=None):

    rclpy.init(args=args)

    sas = SerialArbiterSim()

    try:
        rclpy.spin(sas)

    except:
        pass

    # Explicitly destroy node
    sas.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
