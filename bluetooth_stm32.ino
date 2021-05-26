
#include <Arduino.h>
#include <time.h>

const int LEDPin = PC13;
char inputData = 0;

void setup()
{
  Serial1.begin(9600);
  Serial1.println(" 123456789012345678901234567890");
  pinMode(LEDPin,OUTPUT);
}


void loop()
{
  if(Serial1.available() > 0)
  {
    inputData = Serial1.read();    
  }

if(inputData == '0')
{
    digitalWrite(LEDPin, HIGH);
    Serial1.println("LED is turned ON");
}
else if (inputData == '1')
{
    digitalWrite(LEDPin, LOW);
    Serial1.println("LED is turned OFF");
}

delay(100);

}
