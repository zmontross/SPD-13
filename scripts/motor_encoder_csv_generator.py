
import serial

from numpy import array

import time as Time

from math import pi

import csv


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

    stop_m1 = 'sm1 0'
    stop_m2 = 'sm2 0'


class SerialCommunicator():

    def __init__(self):

        self.serial_port_path = '/dev/ttyACM0'
        self.serial_baud_rate = '115200'
        self.serial_terminator = '\r'

        self.serial_port = serial.Serial(self.serial_port_path, self.serial_baud_rate, timeout=2)


    def serial_send(self, tx):
        # Clear buffers
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # Write given input as raw bytes; Arduino code is expected to handle any possible input.
        self.serial_port.write(bytes(tx + self.serial_terminator, 'utf-8'))

        # Grab raw bytes up to the pre-agreed terminator.
        rx = self.serial_port.read_until(bytes(self.serial_terminator, 'utf-8'), SerialCommands.MAX_BYTES)

        # Convert bytes to string, remove expected terminator.
        rx = rx.decode('utf-8').strip(self.serial_terminator)

        return rx


def main(args=None):

    m1_data = {}
    m2_data = {}

    wheel_diameter_meters = 0.04186

    encoder_counts_per_rev = 2248.86

    encoder_counts_per_meter = encoder_counts_per_rev / (wheel_diameter_meters * pi)

    sc = SerialCommunicator()
    
    sc.serial_send(SerialCommands.stop_m1)
    sc.serial_send(SerialCommands.stop_m2)
    sc.serial_send(SerialCommands.reset_enc1)
    sc.serial_send(SerialCommands.reset_enc2)

    print('Initialized.')
    # sc.serial_send(SerialCommands.do_beep2)
    # Time.sleep(0.25)
    # sc.serial_send(SerialCommands.do_beep3)


    speed_max = 400
    speed_step = 25

    print('Beginning test with Motor 1 (right)')

    sc.serial_send('sm1 0')
    Time.sleep(0.25)
    for speed in range(0, (speed_max + speed_step), speed_step):

        print("Testing Speed={}".format(speed))

        encoder_count_start = 0
        encoder_count_end = 0
        time_start = 0.0
        time_end = 0.0

        ds = 0.0  # Distance delta
        dt = 0.0  # Time delta

        velocity = 0.0

        # Stop, and ramp-up to the target speed
        for ramp in range(speed, (speed + speed_step), 1):
            sc.serial_send('sm1 {}'.format(ramp))
            Time.sleep(0.1)
        Time.sleep(0.5)

        # Snapshot starting time
        time_start = Time.time()

        # Snapshot encoder initial
        _, encoder_count_start, _ = (sc.serial_send(SerialCommands.query_encoders)).split(':')
        
        # Run for a while...
        Time.sleep(15)

        # Snapshot ending time
        time_end = Time.time()

        # Snapshot encoder final
        _, encoder_count_end, _ = (sc.serial_send(SerialCommands.query_encoders)).split(':')

        ds = (int(encoder_count_end) - int(encoder_count_start)) / encoder_counts_per_meter
        dt = time_end - time_start

        velocity = ds / dt

        m1_data[speed] = velocity

        # print('Result ~ \tMotor Speed={}\t\t\tVelocity= {} m/s'.format(
        #     speed, velocity
        # ))
        

    # Ramp-down to stopping, then ensure a stop occurs
    for ramp in range(speed_max, -speed_step, -1):
        sc.serial_send('sm1 {}'.format(ramp))
        Time.sleep(0.1)
    sc.serial_send('sm1 0')
    Time.sleep(0.5)


    print('M1 Test complete.')
    print(m1_data.items())

    ################################################################################################

    print('Beginning test with Motor 2 (Left)')

    sc.serial_send('sm2 0')
    Time.sleep(0.25)
    for speed in range(0, (speed_max + speed_step), speed_step):

        print("Testing Speed={}".format(speed))

        encoder_count_start = 0
        encoder_count_end = 0
        time_start = 0.0
        time_end = 0.0

        ds = 0.0  # Distance delta
        dt = 0.0  # Time delta

        velocity = 0.0

        # Stop, and ramp-up to the target speed
        for ramp in range(speed, (speed + speed_step), 1):
            sc.serial_send('sm2 {}'.format(ramp))
            Time.sleep(0.1)
        Time.sleep(0.5)

        # Snapshot starting time
        time_start = Time.time()

        # Snapshot encoder initial
        _, _, encoder_count_start = (sc.serial_send(SerialCommands.query_encoders)).split(':')
        
        # Run for a while...
        Time.sleep(15)

        # Snapshot ending time
        time_end = Time.time()

        # Snapshot encoder final
        _, _, encoder_count_end = (sc.serial_send(SerialCommands.query_encoders)).split(':')

        ds = (int(encoder_count_end) - int(encoder_count_start)) / encoder_counts_per_meter
        dt = time_end - time_start

        velocity = ds / dt

        m2_data[speed] = velocity

        # print('Result ~ \tMotor Speed={}\t\t\tVelocity= {} m/s'.format(
        #     speed, velocity
        # ))
        

    # Ramp-down to stopping, then ensure a stop occurs
    for ramp in range(speed_max, -speed_step, -1):
        sc.serial_send('sm2 {}'.format(ramp))
        Time.sleep(0.1)
    sc.serial_send('sm2 0')
    Time.sleep(0.5)


    print('M2 Test complete.')
    print(m2_data.items())

    ################################################################################################

    with open('motor_encoder_relationship.csv', 'w') as csvdata:

        csvwriter = csv.writer(csvdata, delimiter=',')

        csvwriter.writerow(['Motor Speed', 'M1 Velocity, m/s', 'M2 Velocity, m/s'])

        for i in range(0, (speed_max+speed_step), speed_step):
            csvwriter.writerow([i, m1_data.get(i), m2_data.get(i)])

        csvdata.close()


if __name__ == '__main__':
    main()

    