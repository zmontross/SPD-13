
import serial

from numpy import array, trim_zeros

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


def MotorTest(sc=SerialCommunicator(), motor_number=1, motor_pwm_max=400, motor_pwm_step=25, test_period_sec=15):
    
    print("Testing Motor {}".format(motor_number))

    motor_data = []
    wheel_diameter_meters = 0.04186
    encoder_counts_per_rev = 2248.86
    encoder_counts_per_meter = encoder_counts_per_rev / (wheel_diameter_meters * pi)

    sc.serial_send('sm{} 0'.format(motor_number))
    Time.sleep(0.5)
    for pwm in range(0, (motor_pwm_max + motor_pwm_step), motor_pwm_step):

        print("Testing PWM={}".format(pwm))

        encoder_count_start = 0
        encoder_count_end = 0
        time_start = 0.0
        time_end = 0.0

        ds = 0.0  # Distance delta
        dt = 0.0  # Time delta

        velocity = 0.0

        # Ramp-up to the target speed, allow motor to maintain speed for a short period.
        for ramp in range(pwm, (pwm + motor_pwm_step), 1):
            sc.serial_send('sm{} {}'.format(motor_number, ramp))
            Time.sleep(0.1)
        Time.sleep(0.5)

        # Snapshot starting time
        time_start = Time.time()

        # Snapshot encoder initial
        encoder_count_start = (sc.serial_send(SerialCommands.query_encoders)).split(':')[motor_number]
        
        # Run for a while...
        Time.sleep(test_period_sec)

        # Snapshot ending time
        time_end = Time.time()

        # Snapshot encoder final
        encoder_count_end= (sc.serial_send(SerialCommands.query_encoders)).split(':')[motor_number]

        ds = (int(encoder_count_end) - int(encoder_count_start)) / encoder_counts_per_meter
        dt = time_end - time_start

        velocity = ds / dt

        motor_data.append([pwm, velocity])
        
    # Ramp-down to stopping, then ensure a stop occurs
    for ramp in range(motor_pwm_max, -motor_pwm_step, -1):
        sc.serial_send('sm{} {}'.format(motor_number, ramp))
        Time.sleep(0.1)
    sc.serial_send('sm{} 0'.format(motor_number))
    Time.sleep(0.5)

    return motor_data


def main(args=None):
    
    try:
        sc = SerialCommunicator()
        
        sc.serial_send(SerialCommands.stop_m1)
        sc.serial_send(SerialCommands.stop_m2)
        sc.serial_send(SerialCommands.reset_enc1)
        sc.serial_send(SerialCommands.reset_enc2)

    except:
        print('An error occurred while establishing serial comms. Aborting test.')
        sys.exit(1)

    
    pwm_max = 400
    pwm_step = 25
    test_period = 20

    print('Beginning test with Motor 1 (right).')
    m1_velocities = MotorTest(sc, motor_number=1, motor_pwm_max=pwm_max, motor_pwm_step=pwm_step, test_period_sec=test_period)

    print('Beginning test with Motor 2 (Left)')
    m2_velocities = MotorTest(sc, motor_number=2, motor_pwm_max=pwm_max, motor_pwm_step=pwm_step, test_period_sec=test_period)

    print('Analyzing results for Motor 1...')
    m1_deltas = array([0.00] * len(m1_velocities))
    for v in range(1, len(m1_velocities)):
        delta = m1_velocities[v][1] - m1_velocities[v-1][1]
        m1_deltas[v] = delta

    print('Analyzing results for Motor 2...')
    m2_deltas = array([0.00] * len(m2_velocities))
    for v in range(1, len(m2_velocities)):
        delta = m2_velocities[v][1] - m2_velocities[v-1][1]
        m2_deltas[v] = delta

    fname = 'motor_encoder_data.csv'
    print('Writing data to file "{}"'.format(fname))
    with open(fname, 'w') as fhandle:
        csvwriter = csv.writer(fhandle, delimiter=',')
        
        csvwriter.writerow(['Motor PWM', 'M1 Velocity, m/s', 'M1 Delta From Last', '', 'M1 Delta Average', '', 'Test Period, s'])
        csvwriter.writerow([m1_velocities[0][0], m1_velocities[0][1], m1_deltas[0], '', m1_deltas.mean(), '', test_period])
        for i in range(1, len(m1_velocities)):
            row = [m1_velocities[i][0], m1_velocities[i][1], m1_deltas[i]]
            csvwriter.writerow(row)
        csvwriter.writerow([])

        csvwriter.writerow(['Motor PWM', 'M2 Velocity, m/s', 'M2 Delta From Last', '', 'M2 Delta Average', '', 'Test Period, s'])
        csvwriter.writerow([m2_velocities[0][0], m2_velocities[0][1], m2_deltas[0], '', m2_deltas.mean(), '', test_period])
        for i in range(1, len(m2_velocities)):
            row = [m2_velocities[i][0], m2_velocities[i][1], m2_deltas[i]]
            csvwriter.writerow(row)
        csvwriter.writerow([])

        csvwriter.writerow(['Average of Averages, M1 Delta & M2 Delta'])
        motor_delta_avg_of_avgs = (trim_zeros(m1_deltas).mean() + trim_zeros(m2_deltas).mean()) / 2
        csvwriter.writerow([motor_delta_avg_of_avgs])
        csvwriter.writerow([])

        csvwriter.writerow(['Velocity Increase (m/s), per PWM unit'])
        motor_vel_inc_per_pwm = motor_delta_avg_of_avgs / pwm_step
        csvwriter.writerow([motor_vel_inc_per_pwm])
        csvwriter.writerow([])

        fhandle.close()


if __name__ == '__main__':
    main()
