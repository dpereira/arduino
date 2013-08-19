
int ledDisplayPins[] = { 2, 3, 4, 5, 6, 7, 8, 9 };
int analogPin = 0;
float voltageLevel = 0.0;

void setup() {
  Serial.begin(115200);
  
  for(int i = 0; i < 8; i++) {
      int pinNumber = ledDisplayPins[i];
      Serial.print("INIT: ");
      Serial.println(pinNumber);
      pinMode(pinNumber, OUTPUT);
      digitalWrite(pinNumber, HIGH);
      delay(1000);
      digitalWrite(pinNumber, LOW);  
  }
  
  float test_numbers[] = {
    5.10, 0.00, 0.02, 
    0.04, 0.08, 0.16, 
    0.32, 0.64, 1.28, 
    2.56,
  }; 
  
  for(int i = 0; i < 10; i++) {
    Serial.print("ENCODING TEST: ");
    Serial.println(test_numbers[i]); 
    encode(test_numbers[i]);
    delay(1000);
  }
  
  for(int i = 1; i < 9; i++) {
    Serial.print("ENCODING TEST: ");
    Serial.println(test_numbers[i]); 
    encode(test_numbers[i] + test_numbers[i + 1]);
    delay(1000);
  }
  
  encode(0.00);
  
  Serial.println("INIT DONE");
}

/**
 * Given a floating point number,
 * from 0.0 to 5.0,
 * encodes it into the 0 - 255 range,
 * representing its 3 most significant
 * digits. The encoded number is actually
 * number / 2.0, so it can fit in the image range.
 */
void encode(float number)
{
    float to_encode = number / 2.0;
    int converted = int(to_encode * 100);
    
    Serial.print("WILL TRY TO DISPLAY: ");
    Serial.println(converted);
    
    for(int i = 0; i < 8; i++) {
      if(converted >= 128) {
        digitalWrite(i + 2, HIGH);
      } else {
        digitalWrite(i + 2, LOW);
      }
      converted = (converted * 2) & 255;
    }
}

float readInput()
{
  int bytesRead = 0;
  float value = 0;
  if(Serial.available() == 3) {
    int readByte = Serial.read() - '0';
    value += readByte * pow(10, -bytesRead);
    bytesRead++;
  }
  
  return value;
}

void loop()
{
  voltageLevel = analogRead(analogPin) * 5.0 / 1024.0;
  encode(voltageLevel);
  Serial.print(">>");
  Serial.print(voltageLevel);
  Serial.println("");
  delay(5000);
}
