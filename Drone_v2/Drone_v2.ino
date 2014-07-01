#include <Servo.h>

void echo(String msg) {
    Serial.println("\n" + msg);
}


unsigned long time = 0;
unsigned long ledState = 0;

// This is our motor.
Servo myMotor0, myMotor1, myMotor2, myMotor3;
Servo motors[] = {myMotor0, myMotor1, myMotor2, myMotor3};

int led = 13;
float voltageLevel = 0.0;
float threshold = 5.0;


// base values for arming the ESCs
// motor 1 (soldered): base == 70
// motor 2: base == 20
// motor 3: base == 70
int arming_bases[] = {70, 20, 70, 45};
int arm_base0 = arming_bases[0]; // arms A
int arm_base1 = arming_bases[1]; // arms C
int arm_base2 = arming_bases[2]; // arms B
int arm_base3 = arming_bases[3]; // arms D

int throttling_bases[] = {70, 20, 70, 46};
int base0 = throttling_bases[0];
int base1 = throttling_bases[1];
int base2 = throttling_bases[2];
int base3 = throttling_bases[3];

int A = 0;
int B = 1;
int C = 2;
int D = 3;

// Set everything up
void setup()
{
    // Put the motor to Arduino pin #9
    myMotor0.attach(3);
    myMotor1.attach(9);
    myMotor2.attach(5);
    myMotor3.attach(11);

    // Required for I/O from Serial monitor
    Serial.begin(9600);
    // Print a startup message
    echo("initializing");
    // initialize the digital pin as an output.
    echo("Ready");
    pinMode(led, OUTPUT);

    myMotor0.write(0);
    myMotor1.write(0);
    myMotor2.write(0);
    myMotor3.write(0);
}

// This is the final output
// written to the motor.
static String incoming_string;

boolean process_command(char* output)
{
    boolean cmd_received = false;
    while(Serial.available() > 0) {
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
            incoming_string = ""; 
        } else {
            incoming_string += ch; 
        } 
    }
 
    incoming_string.toUpperCase();
    incoming_string.toCharArray(output, 1024); 
    if(cmd_received == true) {
      incoming_string = "";
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

int command_type(String cmd) {
    cmd.toUpperCase();
    int type = CMD_TYPE_NONE;
    if(cmd.indexOf("F") == 0) {
        type = CMD_TYPE_FULL_STOP;
    } else if(cmd.indexOf("ARM") == 0) {
        type = CMD_TYPE_ARM;
    } else if(cmd.indexOf("C") == 0) {
        type = CMD_TYPE_CALIBRATE;
    } else if(cmd.indexOf("L") == 0) {
        type = CMD_TYPE_ALL_LIFT;
    } else if(cmd.indexOf("S") == 0) {
        type = CMD_TYPE_SINGLE_LIFT;
    } else if(cmd.indexOf("P") == 0) {
        type = CMD_TYPE_PAIRED_LIFT;
    } 
    return type;
}

void do_nop(String(cmd)) {echo("nop");}
void do_calibrate(String(cmd)) {echo("cal");}
void do_single_lift(String(cmd)) {echo("slift");}
void do_paired_lift(String(cmd)) {echo("plift");}

void do_full_stop(String(cmd)) {
    echo("FSTOP");
    myMotor0.write(base0);
    myMotor1.write(base1);
    myMotor2.write(base2);
    myMotor3.write(base3);
}

void do_arm(String cmd) {
    echo("ARM: " + cmd);
    if(cmd.length() > 3) {
        switch(cmd.charAt(3)) {
            case 'A':
                myMotor0.write(arm_base0);
                break;
            case 'C':
                myMotor1.write(arm_base1);
                break;
            case 'B':
                myMotor2.write(arm_base2);
                break;
            case 'D':
                myMotor3.write(arm_base3);
                break;
        }
    } else {
        myMotor0.write(arm_base0);
        myMotor1.write(arm_base1);
        myMotor2.write(arm_base2);
        myMotor3.write(arm_base3);
    }
}

void do_all_lift(String cmd, int type) {
    echo("LIFT: " + cmd); 

    int val = -1;
    char key = 'A';
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
                case 'A':
                    myMotor0.write(base0 + val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        myMotor2.write(base2 + val);
                        echo("AB paired");
                    } else
                        echo("A single");
                    break;
                case 'B':
                    myMotor2.write(base2 + val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        myMotor0.write(base0 + val);
                        echo("AB paired");
                    } else
                        echo("B single");
                    break;
                case 'C':
                    myMotor1.write(base1 + val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        myMotor3.write(base3 + val);
                        echo("CD paired");
                    } else
                        echo("C single");
                    break;
                case 'D':
                    myMotor3.write(base3 + val);
                    if (type == CMD_TYPE_PAIRED_LIFT) {
                        myMotor1.write(base1 + val);
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
            myMotor0.write(base0 + val);
            myMotor1.write(base1 + val);
            myMotor2.write(base2 + val);
            myMotor3.write(base3 + val);                
            break;
    } 
}

void loop()
{
    delay(100);
    //int analogPin = 0;
    //voltageLevel = analogRead(analogPin) * 5.0 / 1024.0;
    //Serial.print("5v rail: ");
    char cmd[1024];

    if(process_command(cmd)) {
        echo(cmd);
        int type = command_type(cmd);
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
            case CMD_TYPE_FULL_STOP: do_full_stop(cmd); break;
            default: do_nop(cmd); break;
        }
    }   
    /*  
    if(Serial.available() > 0)
    {
        } else {
            // received a newline (linefeed) character
            // this means we are done making a string
            // print the incoming string
            Serial.println("I am printing the entire string");
            Serial.println(incoming_string);

            // Convert the string to an integer
            int val = incoming_string.toInt();

            // print the integer
            Serial.println("Printing the value: ");
            Serial.println(val);

            if (val > -1 && val < 91)
            {
                // Print confirmation that the
                // value is between 0 and 180
                Serial.println("Value is between 0 and 180");
                // Write to Servo
                myMotor1.write(base1 + val);
                myMotor2.write(base2 + val);

                myMotor3.write(base3 + val);                
            }
            // The value is not between 0 and 180.
            // We do not want write this value to
            // the motor.
            else
            {
                Serial.println("Value is NOT between 0 and 180");

                // IT'S a TRAP!
                Serial.println("Error with the input");
            }

            // Reset the value of the incoming_string
            Serial.println(incoming_string);
            incoming_string = "";
        }

        if (incoming_string == "arm") {
            Serial.print("ARMING: M1 -> ");
            Serial.print(arm_base1);
            Serial.print("M2 -> ");
            Serial.println(arm_base2);
            Serial.print("M3 -> ");
            Serial.println(arm_base3);
            myMotor1.write(arm_base1);
            myMotor2.write(arm_base2);
            myMotor3.write(arm_base3);
            incoming_string = "";
        } 
    }*/

    ping();

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
    Serial.print(".");
    time = currentTime;
    digitalWrite(led, ledState % 2 ? HIGH: LOW);
    ledState += 1;
    currentTime = time;
  }
}

