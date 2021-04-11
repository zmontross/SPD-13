# Overview

The microcontroller side of the machine is a [Pololu A-Star 32U4 SV](https://www.pololu.com/product/3119) Raspberry Pi Hat which contains an Atmel ATMega 32u4 microcontroller. Arduino was used as the programming environment/ecosystem for the sake of utilizing existing code. Pololu has some very small, nothing-more-than-necessary, performance-focused libraries for the A-Star board and their other peripherals.

## Compilation

Simply load the source code into the Arduino editor and click the "verify" button. In the temporary files directory a build artifacts directory will be created that will contain a number of alternatives. The `arduino_sketch.ino.hex` file is the desired output.

Alternatively the compilation can be performed using the Arduino-CLI program (See Installation section)

## Installation

It is expected that the [Arduino-CLI](https://arduino.github.io/arduino-cli/latest/) has been installed to the robot.

Note: It is possible to compile *and* upload with this utility in the same step if desired, but this necessitates compiling source code *on the robot*.

Prior to installation the [Pololu Board Manager URL](https://www.pololu.com/docs/0J61/6.2) must be added.

```bash
### Typical Bash terminal process

# Name of hex file to upload.
ARDUINO_HEX_FILE='spd13_pololu_astar.ino.hex'

# Typical Linux default port for Arduino; actual number in 'ttyACMn' may vary.
ARDUINO_TTY_PORT='/dev/ttyACM0'

# Fully-Qualified Board Name required by Arduino during compilation/upload.
ARDUINO_FQBN='pololu-a-star:avr:a-star32U4'

# Upload command. Board, Port, In-file
arduino-cli upload -b $ARDUINO_FQBN -p $ARDUINO_TTY_PORT -i $ARDUINO_HEX_FILE

```

## Usage
This information is subject to change.

All commands are terminated with a Carriage Return(aka '\r' or 0x0D).
```c
//  Query commands:
//  qa  -   Query Accelerometer
//  qg  -   Query Gyroscope
//  qe  -   Query Encoders
//  qm  -   Query Motors
//
//  Reset commands:
//  xi  -   Reset IMU (acclerometer/gyroscope).
//  xe1 -   Reset Encoder 1 count.
//  xe2 -   Reset Encoder 2 count.
//
//  Set commands:
//  sm1 xxxx  -   Set Motor 1 Speed, -400 to 400
//  sm2 xxxx  -   Set Motor 2 Speed, -400 to 400
//
//  Miscellaneous:
//  @   -   Toggle character echo (for debug purposes).
//  b1  -   Run Beep 1
//  b2  -   Run Beep 2
//  b3  -   Run Beep 3
//  b4  -   Run Beep 4
```

Note: By default character echo, toggled by sending "@\r", is OFF by default. This feature is only intended for manual user debugging.

All responses are also terminated with Carriage Returns.

Most commands are acknowledged with "0" (the ASCII character 0x30) indicating success.

Unrecognized input is acknowledged with "-1" (ASCII 0x2D followed by ASCII 0x31) indicating failure/rejection.

Query commands return the following responses:
```c
// qa - 'ACCEL:0:0:0'
// qg - 'GYRO:0:0:0'
// qe - 'ENC:0:0'
// qm - 'MOTOR:0:0'
```

