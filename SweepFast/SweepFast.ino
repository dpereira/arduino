// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
                
Servo myservo2;
 
int pos = 0;    // variable to store the servo position 
int led = 13;
int ledMode = HIGH;
int analogPin = 0;
int val = 0;
float prior1 = 0.0;
float prior2 = 0.0;
float zero1 = 0.0;
float zero2 = 0.0;
float threshold = 4.4;
float thresholdLow = 4.0;
float surplus1 = 0.0;
float surplus2 = 0.0;
float average = 0.0;
float averageCycle = 0.0;
int acumCycle = 0;
int cycle = 10;
int truePos1 = 0;

boolean attached = false;

void setup() 
{
 attached = false; 
  pinMode(led, OUTPUT);     
  Serial.begin(115200); 
} 
  
void loop() 
{
  boolean moved = false;
  pos += 1;
  
  if(pos % cycle == 0) {
    acumCycle = 0;
    averageCycle = 0.0;
  }
  
  boolean prior1on = myservo.attached();
  prior1 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin  
  average += prior1;
  averageCycle += prior1;
  acumCycle += 1;

  zero1 = prior1;
  if(!myservo.attached() && prior1 > threshold){  
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  }
  surplus1 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin
  if(myservo.attached() && surplus1 > threshold) {
    myservo.write((truePos1 % 2) * 180);              // tell servo to go to position in variable 'pos' 
    truePos1 += 1;
    moved = true;
  }
  prior2 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin
  zero2 = prior2;
  if(!myservo2.attached() && prior2 > threshold && prior1on) {
    myservo2.attach(10);  // attaches the servo on pin 9 to the servo object 
  }
  surplus2 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin
  if(myservo2.attached() && surplus2 > threshold ) {
    myservo2.write((pos % 2) * 180);              // tell servo to go to position in variable 'pos'     
  }
  prior2 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin
  if(prior2 < thresholdLow && myservo2.attached()) {
      myservo2.detach();
  }
  prior1 = analogRead(analogPin) * 5.0 / 1024.0;    // read the input pin
  if(prior1 < thresholdLow && myservo.attached()) {
      myservo.detach();
  }

  if(myservo.attached()) { 
    myservo.write((pos % 2) * 180);              // tell servo to go to position in variable 'pos' 
  }
  
  if(myservo2.attached()) {
    myservo2.write((pos % 2) * 180);
  }
    digitalWrite(led, ledMode);
    delay(1000);                       // waits 15ms for the servo to reach the position 
    if(ledMode == HIGH) {
      ledMode = LOW;
    } else {
      ledMode = HIGH;
    }
  val = analogRead(analogPin);    // read the input pin
  Serial.print(">> ");
  Serial.print(pos);
  Serial.print(" z1: ");
  Serial.print(zero1);
  Serial.print(" z2: ");
  Serial.print(zero2);
  Serial.print(" p1: ");
  Serial.print(prior1);
  Serial.print(" p2: ");
  Serial.print(prior2);
  Serial.print(" s1: ");
  Serial.print(surplus1);
  Serial.print(" s2: ");
  Serial.print(surplus2);
  Serial.print(" to: ");
  Serial.print(val * 5.0 / 1024.0);             // debug value    
  Serial.print("\t");
  Serial.print(myservo.attached() ? "A1": "D1");
  Serial.print(" / ");
  Serial.print(myservo2.attached() ? "A2": "D2");
  Serial.print("\tAVG: ");
  Serial.print(average / pos);
  Serial.print(" AVG CYCLE: ");
  Serial.print(averageCycle / acumCycle);
  Serial.println("");
}
