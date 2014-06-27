/*
 *  This code is in the public domain.
 *  (Do whatever you want with it.)
 */

// Need the Servo library
#include <Servo.h>

unsigned long time = 0;
unsigned long ledState = 0;

// This is our motor.
Servo myMotor1, myMotor2, myMotor3;

// This is the final output
// written to the motor.
String incomingString;
int led = 13;
float voltageLevel = 0.0;
float threshold = 5.0;


// base values for arming the ESCs
// motor 1 (soldered): base == 70
// motor 2: base == 20
// motor 3: base == 70
int arming_bases[] = {70, 20, 45}; 
int arm_base1 = arming_bases[0]; // arms A and C
int arm_base2 = arming_bases[1]; // arms B
int arm_base3 = arming_bases[2]; // arms D

int throttling_bases[] = {70, 20, 46};
int base1 = throttling_bases[0];
int base2 = throttling_bases[1];
int base3 = throttling_bases[2];

// Set everything up
void setup()
{
    // Put the motor to Arduino pin #9
    myMotor1.attach(5);
    myMotor2.attach(9);
    myMotor3.attach(11);

    // Required for I/O from Serial monitor
    Serial.begin(9600);
    // Print a startup message
    Serial.println("initializing");
    // initialize the digital pin as an output.
    Serial.println("Ready");
    pinMode(led, OUTPUT);     

    myMotor1.write(0);
    myMotor2.write(0);
    myMotor3.write(0);
}

void loop()
{
    delay(100);
    //int analogPin = 0;
    //voltageLevel = analogRead(analogPin) * 5.0 / 1024.0;
    //Serial.print("5v rail: ");
    //Serial.println(voltageLevel);

    // If there is incoming value
    if(Serial.available() > 0)
    {
        // read the value
        char ch = Serial.read();

        /*
         *  If ch isn't a newline
         *  (linefeed) character,
         *  we will add the character
         *  to the incomingString
         */
        Serial.print(ch);
        Serial.println(" >>");
        if (ch != 10 && ch != 65 && ch != 67){
            // Print out the value received
            // so that we can see what is
            // happening
            Serial.print("I have received: ");
            Serial.print(ch, DEC);
            Serial.print('\n');

            // Add the character to
            // the incomingString
            incomingString += ch;
        } else if(ch == 67) {
            incomingString = "";
            Serial.println("Cleared buffer");
        } else {
            // received a newline (linefeed) character
            // this means we are done making a string
            // print the incoming string
            Serial.println("I am printing the entire string");
            Serial.println(incomingString);

            // Convert the string to an integer
            int val = incomingString.toInt();

            // print the integer
            Serial.println("Printing the value: ");
            Serial.println(val);

            /*
             *  We only want to write an integer between
             *  0 and 180 to the motor. 
             */
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

            // Reset the value of the incomingString
            Serial.println(incomingString);
            incomingString = "";
        }

        if (incomingString == "arm") {
            Serial.print("ARMING: M1 -> ");
            Serial.print(arm_base1);
            Serial.print("M2 -> ");
            Serial.println(arm_base2);
            Serial.print("M3 -> ");
            Serial.println(arm_base3);
            myMotor1.write(arm_base1);
            myMotor2.write(arm_base2);
            myMotor3.write(arm_base3);
            incomingString = "";
        } 
    }

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
        sendMessage("ping\n");
        time = currentTime;
        digitalWrite(led, ledState % 2 ? HIGH: LOW);
        ledState += 1;
        currentTime = time;
      }
}
