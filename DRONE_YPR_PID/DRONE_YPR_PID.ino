// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "PID_v1.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

boolean started = false;

double yawInput, yawOutput, yawSetPoint;
PID yawPID(&yawInput, &yawOutput, &yawSetPoint, 2, 5, 1, DIRECT);

double pitchInput, pitchOutput, pitchSetPoint;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetPoint, 2, 5, 1, DIRECT);

double rollInput, rollOutput, rollSetPoint;
PID rollPID(&rollInput, &rollOutput, &rollSetPoint, 2, 5, 1, DIRECT);

//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

boolean pid_initialized = false;
boolean dumpYpr = true;
boolean stabilize = false;
boolean ypr_steady = false;
char cmd[128];
int counter = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  cmd[0] = 0;
  counter = 0;
  Serial.begin(9600);
  engines_setup();
  pid_setup();
}

void pid_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    yawSetPoint = 0;
    pitchSetPoint = 0;
    rollSetPoint = 0;
    yawOutput = 0;
    pitchOutput = 0;
    rollOutput = 0;
    
    yawPID.SetOutputLimits(-180, 180);
    pitchPID.SetOutputLimits(-180, 180);
    rollPID.SetOutputLimits(-180, 180);    
    
    yawPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);    

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
//    pinMode(LED_PIN, OUTPUT);
    
    pid_initialized = true;
}

#include <Servo.h>

void echo(String msg) {
    Serial.println(msg);
}


unsigned long time = 0;
unsigned long ledState = 0;

// This is our motor.
Servo myMotor0, myMotor1, myMotor2, myMotor3;
Servo motors[] = {myMotor0, myMotor1, myMotor2, myMotor3};

int led = 13;
float voltageLevel = 0.0;
float threshold = 5.0;


int current_throttle[] = {0, 0, 0, 0};
// base values for arming the ESCs
// motor 1 (soldered): base == 70
// motor 2: base == 20
// motor 3: base == 70
int arming_bases[] = {25, 25, 25, 25};
int arm_base0 = arming_bases[0]; // arms A
int arm_base1 = arming_bases[1]; // arms C
int arm_base2 = arming_bases[2]; // arms B
int arm_base3 = arming_bases[3]; // arms D

//int throttling_bases[] = {70, 20, 70, 46};
int throttling_bases[] = {25, 25, 25, 25};
int base0 = throttling_bases[0];
int base1 = throttling_bases[1];
int base2 = throttling_bases[2];
int base3 = throttling_bases[3];

int A = 0;
int B = 1;
int C = 2;
int D = 3;

void delta_motor(int motor, int value) 
{
    current_throttle[motor] += value;
    motors[motor].write(current_throttle[motor]);
}

void set_motor(int motor, int value)
{
    current_throttle[motor] = value + throttling_bases[motor];
    motors[motor].write(current_throttle[motor]);
}

void set_motor_raw(int motor, int value)
{
    current_throttle[motor] = value;
    motors[motor].write(current_throttle[motor]);
}

// Set everything up
void engines_setup()
{
    pinMode(led, OUTPUT);
    digitalWrite(led, HIGH);
    // Put the motor to Arduino pin #9
    myMotor0.attach(5);
    myMotor1.attach(6);
    myMotor2.attach(9);
    myMotor3.attach(11);
    digitalWrite(led, LOW);

    // Required for I/O from Serial monitor
    Serial.println("ENGINE SETUP");    // Print a startup message
    Serial.println("initializing");
    // initialize the digital pin as an output.
    Serial.println("Ready");

    // sets up all esc throttling ranges 
    digitalWrite(led, HIGH);
    set_motor_raw(A, 179);
    set_motor_raw(B, 179);
    set_motor_raw(C, 179);
    set_motor_raw(D, 179);
    delay(3800);
    set_motor_raw(A, 25);
    set_motor_raw(B,  25);
    set_motor_raw(C, 25);
    set_motor_raw(D, 25);
    delay(2000);
    digitalWrite(led, LOW);
}

// This is the final output
// written to the motor.
String incoming_string = "";
unsigned long last_processed = 0;

boolean process_command(char* output)
{
  boolean cmd_received = false;
  unsigned long current_time = millis();
    
    if(current_time - last_processed  < 500) {
        return false;
    }
    
    last_processed = current_time;
  
       //Serial.println(Serial.available());
    while(Serial.available()) {
        // read the value
        char ch = Serial.read();        
        /*  
         *  If ch isn't a newline
         *  (linefeed) character,
         *  we will add the character
         *  to the incoming_string
         */
        if(ch == 10 || ch == 59 || ch == 46) {
            cmd_received = true;
            break;
        } else if(ch == 67) {
            output[0] = 0;             
            counter = 0;
          } else {
            output[counter++] = ch;
            output[counter] = 0;
        } 
    }
    
 
    if(cmd_received == true) {
//      Serial.print("CMD RECEIVED: ");
      Serial.println(output);
      counter = 0;
    }

    return cmd_received;
}

const int CMD_TYPE_NONE = 0;
const int CMD_TYPE_ARM = 1;
const int CMD_TYPE_PAIRED_LIFT = 2;
const int CMD_TYPE_SINGLE_LIFT= 3;
const int CMD_TYPE_ALL_LIFT= 4;
const int CMD_TYPE_CALIBRATE = 5;
const int CMD_TYPE_FULL_STOP = 6;
const int CMD_TYPE_RAW = 7;
const int CMD_TYPE_ALL_INC = 8;
const int CMD_TYPE_ALL_DEC = 9;
const int CMD_TYPE_PING = 10;
const int CMD_TYPE_INIT_PID = 11;
const int CMD_TYPE_DUMP_YPR = 12;
const int CMD_TYPE_STABILIZE = 13;

int command_type(char* cmd) {
    //cmd.toUpperCase();
//    Serial.print(">>>");
//    Serial.println(cmd);
    int type = CMD_TYPE_NONE;
    if(strcmp(cmd,"init_pid") == 0) {
        type = CMD_TYPE_INIT_PID;
    } else if(strcmp(cmd,"ping") == 0) {
        type = CMD_TYPE_PING;
    } else if(strcmp(cmd, "dump_ypr") == 0) {
        type = CMD_TYPE_DUMP_YPR;
    } else if(cmd[0] == 'f') {
        type = CMD_TYPE_FULL_STOP;
    } else if(strcmp(cmd, "arm") == 0) {
        type = CMD_TYPE_ARM;
    } else if(strcmp(cmd, "ac") == 0) {
        type = CMD_TYPE_STABILIZE;
    } else if(cmd[0] == 'c') {
        type = CMD_TYPE_CALIBRATE;
    } else if(cmd[0] == 'l') {
        type = CMD_TYPE_ALL_LIFT;
    } else if(cmd[0] == 's') {
        type = CMD_TYPE_SINGLE_LIFT;
    } else if(cmd[0] == 'p') {
        type = CMD_TYPE_PAIRED_LIFT;
    } else if(cmd[0] == 'i') {
        type = CMD_TYPE_ALL_INC;
    } else if(cmd[0] == 'd') {
        type = CMD_TYPE_ALL_DEC;
    }

    return type;
}

void do_nop(String(cmd)) {echo("nop");}
void do_calibrate(String(cmd)) {echo("cal");}

void do_inc(String cmd) {
  int value = 1;
  if(cmd.length() > 1) {
    value = cmd.substring(2).toInt();
  }
  delta_motor(A, value);
  delta_motor(B, value);
  delta_motor(C, value);
  delta_motor(D, value);
}

void do_dec(String cmd) {
  int value = 1;
  if(cmd.length() > 1) {
    value = cmd.substring(2).toInt();
  }
  delta_motor(A, -value);
  delta_motor(B, -value);
  delta_motor(C, -value);
  delta_motor(D, -value);
}

void do_full_stop(String(cmd)) {
    echo("FSTOP");
    set_motor(A, 0);
    set_motor(B, 0);
    set_motor(C, 0);
    set_motor(D, 0);
}

void do_arm(String cmd) {
    echo("ARM: " + cmd);
    if(cmd.length() > 3) {
        switch(cmd.charAt(3)) {
            case 'a':
                set_motor(A, arming_bases[A]);
                break;
            case 'b':
                set_motor(B, arming_bases[B]);
                break;
            case 'c':
                set_motor(C, arming_bases[C]);
                break;
            case 'd':
                set_motor(D, arming_bases[D]);
                break;
        }
    } else {
        set_motor(A, arming_bases[A]);
        set_motor(A, arming_bases[B]);
        set_motor(A, arming_bases[C]);
        set_motor(A, arming_bases[D]);
    }
}

void do_all_lift(String cmd, int type) {
    echo("LIFT: " + cmd); 

    int val = -1;
    char key = 'a';
    switch(type) {
        case CMD_TYPE_PAIRED_LIFT:
        case CMD_TYPE_SINGLE_LIFT:
            val = cmd.substring(2).toInt();
            Serial.println(val);
            key = cmd.charAt(1);
            if (val < 0 || val > 179)  
            {
                echo("Value is NOT between 0 and 180");
                echo("Error with the input");
            }
            switch(key) {
                case 'a':
                    set_motor(A, val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        set_motor(B, val);
                        echo("AB paired");
                    } else
                        echo("A single");
                    break;
                case 'b':
                    set_motor(B,  val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        set_motor(A, val);
                        echo("AB paired");
                    } else
                        echo("B single");
                    break;
                case 'c':
                    set_motor(C, val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        set_motor(D, val);
                        echo("CD paired");
                    } else
                        echo("C single");
                    break;
                case 'd':
                    set_motor(D, val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        set_motor(C,  val);
                        echo("CD paired");
                    } else
                        echo("D single");
                    break;
            }
            break;
        case CMD_TYPE_ALL_LIFT:
            val = cmd.substring(1).toInt();
            if (val < 0 || val > 179)  
            {
                echo("Value is NOT between 0 and 180");
                echo("Error with the input");
            }
            set_motor(A,  val);
            set_motor(B,  val);
            set_motor(C,  val);
            set_motor(D,  val);
            break;
    } 
}

void cmd_loop()
{
    //int analogPin = 0;
    //voltageLevel = analogRead(analogPin) * 5.0 / 1024.0;
    //Serial.print("5v rail: ");
    if(process_command(cmd)) {
        int i = 0;
        Serial.print("Out of process cmd: ");
        Serial.println(cmd);
        int type = command_type(cmd);
        Serial.println(type);
        switch(type) {
            case CMD_TYPE_ARM: 
                do_arm(cmd); 
                break;
            case CMD_TYPE_PAIRED_LIFT: 
            case CMD_TYPE_SINGLE_LIFT: 
            case CMD_TYPE_ALL_LIFT: 
                do_all_lift(cmd, type); 
                break;
            case CMD_TYPE_CALIBRATE: do_calibrate(cmd); break;
            case CMD_TYPE_FULL_STOP: stabilize = false; do_full_stop(cmd); break;
            case CMD_TYPE_ALL_INC: do_inc(cmd); break;
            case CMD_TYPE_ALL_DEC: do_dec(cmd); break;
            case CMD_TYPE_PING: ping(); break;
            case CMD_TYPE_INIT_PID: pid_setup(); break;
            case CMD_TYPE_DUMP_YPR: dumpYpr = !dumpYpr; break;
            case CMD_TYPE_STABILIZE: stabilize = !stabilize; Serial.println(stabilize ? "ATTITUDE CONTROL ENABLED" : "ATTITUDE CONTROL DISABLED"); break;
            default: do_nop(cmd); break;
        }
    }   

}

void sendMessage(char* message) {
  int messageLen = strlen(message);
  if(messageLen < 256) {
    Serial.write(messageLen);
    Serial.print(message);
  }
}

void ping()
{
  unsigned long currentTime = millis();
  if((currentTime - time) > 1000) {
     if(dumpYpr) {
              Serial.print("ypr\t");
              Serial.print(yawSetPoint - yawInput);
              Serial.print(", ");
              Serial.print(yawOutput);
              Serial.print("\t");
              Serial.print(pitchSetPoint - pitchInput);
              Serial.print(", ");            
              Serial.print(pitchOutput);
              Serial.print("\t");
              Serial.print(rollSetPoint - rollInput);
              Serial.print(", ");            
              Serial.println(rollOutput);            
              //Serial.print(", ");            
              //Serial.println(rollSetPoint);
     } else {
      Serial.println(".");
     }    

    time = currentTime;
    digitalWrite(led, ledState % 2 ? HIGH: LOW);
    ledState += 1;
    currentTime = time;
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
double previous_yaw = 180.0, previous_pitch  = 180.0, previous_roll = 180.0, ypr_steadiness_threshold = 0.01;
unsigned long previous_ypr_check = 0;
int steady_seconds = 0;

void pid_loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    int timeout = millis() + 1000;
    while (!mpuInterrupt && fifoCount < packetSize) {
      if(millis() > timeout) {
          Serial.println("Stuck within the pid loop, resetting and bailing out.");
          mpu.resetDMP();
          mpu.reset();
          delay(50);
          pid_setup();
          return;
      }
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawInput = ypr[0] * 180/M_PI;
            pitchInput = ypr[1] * 180/M_PI;
            rollInput = ypr[2] * 180/M_PI;
            unsigned long current_time = millis();
            if(!ypr_steady && current_time - previous_ypr_check > 1000) {
              Serial.println("Checking gyroscope stability ...");
              previous_ypr_check = current_time;
              
              double yaw_delta = previous_yaw - yawInput;
              double pitch_delta = previous_pitch - pitchInput;              
              double roll_delta = previous_roll - rollInput;
              
              previous_yaw = yawInput;
              previous_pitch = pitchInput;
              previous_roll = rollInput;
                           
              if(abs(yaw_delta) <= ypr_steadiness_threshold && abs(pitch_delta) <= ypr_steadiness_threshold && abs(roll_delta) <= ypr_steadiness_threshold) {
                steady_seconds += 1;
                if(!started && steady_seconds > 5) {
                  Serial.println("ALL GYROSCOPE GIMBALS STEADY: ready for takeoff.");
                  ypr_steady = true;
                  started = true;               
                  yawSetPoint = yawInput;
                  pitchSetPoint = pitchInput;
                  rollSetPoint = rollInput;              
                }
              } else {
                Serial.println("Reseting stability count.");
                steady_seconds = 0;
              }
            }            
            
            if(ypr_steady) {
              yawPID.Compute();
              pitchPID.Compute();
              rollPID.Compute();
            }
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
    }
}

int yaw_correction, pitch_correction, roll_correction;

void attitude_control() {
  // yaw > 0: rotating left (ccw)
  yaw_correction = yawOutput / 9.0;
  delta_motor(A, - yaw_correction);
  delta_motor(D, - yaw_correction);
  delta_motor(B, yaw_correction);
  delta_motor(C, yaw_correction);
  // pitch
  pitch_correction = pitchOutput / 9.0;
  delta_motor(A, -pitch_correction); 
  delta_motor(B, -pitch_correction);
  delta_motor(C, pitch_correction);
  delta_motor(D, pitch_correction);  
  // roll
  roll_correction = roll_correction / 9.0;
  delta_motor(A, -roll_correction);
  delta_motor(C, -roll_correction);
  delta_motor(B, roll_correction);
  delta_motor(D, roll_correction);    
}

void loop() {
  cmd_loop();
  
  if(pid_initialized == true) {
    pid_loop();
    
    if(stabilize == true && ypr_steady == true) {
      attitude_control();
    }
  }
  
  ping();
}
