/***** ***** ***** ***** ***** ***** ***** *****
 *  Description
***** ***** ***** ***** ***** ***** ***** *****/
// Arduino program for the Pololu A-Star 32u4 SV Raspberry Pi Hat
// Program interfaces with the following motor+encoder using hardware interrupts to track count values:
// https://www.pololu.com/product/4825
// Program interfaces with the following IMU using I2C to capture linear and angular acceleration values:
// https://www.pololu.com/product/2736

// Commands received via serial are kept simple, and only non-blocking code has been allowed.
// All commands are parsed one byte per loop.
// A carriage-return, '\r', must be sent to indicate a valid command.
// The buffer will be examined/cleared if it is full.
//
// Accepted commands are grouped into general categories:
//
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


/***** ***** ***** ***** ***** ***** ***** *****
 *  TODOs
***** ***** ***** ***** ***** ***** ***** *****/
// TODO - re-initialize-all command, 'x*'
// TODO - beep 'bxxxx' command to beep with specific freq
// TODO - set motor flip command 'sf1 0||1'


/***** ***** ***** ***** ***** ***** ***** *****
 *  Implementation Notes
***** ***** ***** ***** ***** ***** ***** *****/
// Motor encoders are rotary encoders with Phase 'A' and Phase 'B' pulse outputs.
// Encoder Phase 'A' of Motor 1 connected to Arduino 15, i.e. "PCINT1" on Atmel pin PB1 (Port B, Pin 1).
// Encoder Phase 'A' of Motor 2 connected to Arduino 16, i.e. "PCINT2" on Atmel pin PB2 (Port B, Pin 2).
// Phase 'B' for Motor 1 and Motor 2 are unused.



/***** ***** ***** ***** ***** ***** ***** *****
 *  Wheel Encoder Defines
 *  We multiply by all values by 100 and trim sigfigs to avoid float math (slower than int math).
***** ***** ***** ***** ***** ***** ***** *****/

// Diameter of wheel, using calipers, is 41.86mm.
#define WHEEL_DIAMETER_MM_X100 4186

// Number of encoder counts per wheel diameter is 2248.86.
#define ENC_CNT_WHEELREV_X100 224886

// Pre-computed number of encoder counts per millimeter is 53.723363593.
#define ENC_CNT_PER_MM_X100 5372

// Encoder Phase Inputs - Arduino Digital Pin numbers
#define ENC_M1_A 15
#define ENC_M2_A 16


#define ENC_RESET_SUCCESS 0

#define FLIP_STATE_M1 true
#define FLIP_STATE_M2 true

/***** ***** ***** ***** ***** ***** ***** *****
 *  LSM6 IMU Defines
 *  Values are of type int16_t (range 32767 to -32768)
***** ***** ***** ***** ***** ***** ***** *****/

// Initialized acceleration sensitivity of IMU is +/- 2g.
// i.e. (2*9.82 m/s^2=) +/-19.64 m/s^2.
// +32767 / 19.64 = 1668.38 per (+1 m/s^2)
// -32768 / 19.64 = 1668.43 per (-1 m/s^2)
#define IMU_2G_CNT_PER_MSECSQRD 1668

// Initialized gyroscope sensitivity of IMU is +/- 245 dps.
// VALUE IS ALREADY IN DEGREES PER SECOND. Delta-Theta is what we want.

#define IMU_INIT_SUCCESS 0
#define IMU_INIT_FAILURE -1


/***** ***** ***** ***** ***** ***** ***** *****
 *  Library Includes
***** ***** ***** ***** ***** ***** ***** *****/

#include <AStar32U4.h>
#include <Wire.h>
#include <LSM6.h>


/***** ***** ***** ***** ***** ***** ***** *****
 *  Type Definitions
***** ***** ***** ***** ***** ***** ***** *****/


/***** ***** ***** ***** ***** ***** ***** *****
 *  Global Variables and Objects
***** ***** ***** ***** ***** ***** ***** *****/

AStar32U4Motors motors;
AStar32U4Buzzer buzzer;
LSM6 imu;


volatile static int32_t m1enc;
volatile static int32_t m2enc;

volatile static bool m1rev;
volatile static bool m2rev;

#define LEN_REPORTS 80
char report[LEN_REPORTS];
const char* accelReportFormat = "ACCEL:%d:%d:%d";
const char* gyroReportFormat = "GYRO:%d:%d:%d";
const char* encReportFormat = "ENC:%ld:%ld";
const char* motorReportFormat = "MOTOR:%c%d:%c%d";


#define LEN_REQUESTS 20
#define REQUEST_TERMINATOR '\r'
char request[LEN_REQUESTS];
char reqIndex;
bool checkRequest;
bool reqEchoChars;



/***** ***** ***** ***** ***** ***** ***** *****
 *  Arduino Setup and Loop
***** ***** ***** ***** ***** ***** ***** *****/

void setup()
{
  // Motor Setup
  motors.flipM1(FLIP_STATE_M1);
  motors.flipM2(FLIP_STATE_M2);
  motors.setSpeeds(0, 0);
  m1rev = false;
  m2rev = false;

  // PortB Change Interrupt Setup
  pinMode(ENC_M1_A, INPUT_PULLUP);
  pinMode(ENC_M2_A, INPUT_PULLUP);
  PCMSK0 |= (1<<PCINT1) | (1<<PCINT2);
  PCICR |= (1<<PCIE0);

  // IMU Setup
  Wire.begin();
  if (imu.init())
  {
    imu.enableDefault();
  }

  // Serial Setup
  reqIndex = 0;
  checkRequest = false;
  reqEchoChars = false;
  Serial.begin(115200);
}


void loop()
{
  static uint32_t tlast, tnow = 0;
  tnow = millis();
  if((tnow - tlast) >= 125){
    tlast = tnow;
    imu.read();
  }

  handleSerial();

} // loop


/***** ***** ***** ***** ***** ***** ***** *****
 *  Custom Routines
***** ***** ***** ***** ***** ***** ***** *****/


void handleSerial(){

  if(Serial.available()){

    char c = Serial.read();

    if(reqEchoChars) Serial.print(c);
    
    request[reqIndex] = c;    
    
    if((c == REQUEST_TERMINATOR) || (reqIndex == LEN_REQUESTS)){
      request[reqIndex] = '\0'; // Ensure null-terminator at end of buffer.
      checkRequest = true;
      if(reqEchoChars) Serial.println();

      while(Serial.available()) Serial.read(); // Flush Serial buffer.
    }
    else{
      reqIndex++;
    }
  }


  if(checkRequest){

    if(request[0] == 'q'){    // Query
      switch(request[1]){
        case 'a':
          snprintf(report, LEN_REPORTS, accelReportFormat, imu.a.x, imu.a.y, imu.a.z);
          Serial.println(report);
          break;
        case 'g':
          snprintf(report, LEN_REPORTS, gyroReportFormat, imu.g.x, imu.g.y, imu.g.z);
          Serial.println(report);
          break;
        case 'e':
          snprintf(report, LEN_REPORTS, encReportFormat, m1enc, m2enc);
          Serial.println(report);
          break;
        case 'm':
          uint16_t m1speed = AStar32U4Motors__getM1SpeedReg(&motors);
          uint16_t m2speed = AStar32U4Motors__getM2SpeedReg(&motors);
          uint8_t m1sign = (m1rev ? '-' : ' ');
          uint8_t m2sign = (m2rev ? '-' : ' ');
          snprintf(report, LEN_REPORTS, motorReportFormat, m1sign, m1speed, m2sign, m2speed);
          Serial.println(report);
          break;
        default:
          Serial.println(-1);
          break;
      }
      
    }
    else if(request[0] == 'x'){   // Reset
      switch(request[1]){
        case 'i':
          if (imu.init())
          {
            imu.enableDefault();
            Serial.println(IMU_INIT_SUCCESS);
          }
          else{
            Serial.println(IMU_INIT_FAILURE);
          }
          break;
        case 'e':
          if(request[2] == '1'){
            m1enc = 0;
            Serial.println(ENC_RESET_SUCCESS);
          }
          else if(request[2] == '2'){
            m2enc = 0;
            Serial.println(ENC_RESET_SUCCESS);
          }
          else{
            Serial.println(-1);
          }
          break;
        default:
          Serial.println(-1);
          break;
      }
      
    }
    else if(request[0] == 's'){   // Set

      if(request[1] == 'm'){
        char speedChars[5];
        int16_t speedInt;
        memset(speedChars, '\0', 5);
        
        for(uint8_t i=3; i<reqIndex; i++){
          speedChars[i-3] = request[i];
        }

        speedInt = atoi(speedChars);
        
        switch(request[2]){
          case '1':
            motors.setM1Speed(speedInt);
            m1rev = ((speedInt < 0) ? true : false);
            Serial.println(0);
            break;
          case '2':
            motors.setM2Speed(atoi(speedChars));
            m2rev = ((speedInt < 0) ? true : false);
            Serial.println(0);
            break;
          default:
            Serial.println(-1);
            break;   
        }
      }
      
    }
    else if(request[0] == '@'){   // Echo Toggle
      reqEchoChars = !reqEchoChars;
    }
    else if(request[0] == 'b'){   // Beeps
      switch(request[1]){
        case '1':
          buzzer.playFrequency(440, 200, 15);
          Serial.println(0);
          break;
        case '2':
          buzzer.playFrequency(494, 200, 15);
          Serial.println(0);
          break;
        case '3':
          buzzer.playFrequency(523, 200, 15);
          Serial.println(0);
          break;
        case '4':
          buzzer.playFrequency(587, 200, 15);
          Serial.println(0);
          break;
        default:
          Serial.println(-1);
          break;
      }
    }
    
    else{                         // Unrecognized Request
      Serial.println(-1);
    }
    
    checkRequest = false;
    memset(request, '\0', LEN_REQUESTS);
    reqIndex = 0;
  } // checkRequest
  
} //handleSerial


int16_t AStar32U4Motors__getM1SpeedReg(AStar32U4Motors* m){

  return ((m == NULL) ? 0 : OCR1A);

} // AStar32U4Motors__getM1SpeedReg


int16_t AStar32U4Motors__getM2SpeedReg(AStar32U4Motors* m){

  return ((m == NULL) ? 0 : OCR1B);

} // AStar32U4Motors__getM2SpeedReg


/***** ***** ***** ***** ***** ***** ***** *****
 *  Interrupt Service Routines
***** ***** ***** ***** ***** ***** ***** *****/

ISR( PCINT0_vect ) { // ISR for PCINT0 aka PORT B change interrupts

  static uint8_t PINB_last = PINB;

  uint8_t change, m1pulse, m2pulse;

  change = PINB_last ^ PINB;

  m1pulse = PINB_last & (1<<PCINT1);
  m2pulse = PINB_last & (1<<PCINT2);

  // If pulses were 0 previously then we have a rising-edge transition.
  if ((m1pulse == 0) && (change & (1<<PCINT1)) ) (m1rev ? m1enc-- : m1enc++);
  if ((m2pulse == 0) && (change & (1<<PCINT2)) ) (m2rev ? m2enc-- : m2enc++);

  PINB_last = PINB;
}
