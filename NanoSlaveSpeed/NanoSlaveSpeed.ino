//i2c Slave(LEONARDO)
#include <Wire.h>
int i = 0;
int pin = 7;
float rpm = 0;
double rpm2;
double timerrevCalc = 0;
double duration;

void setup()
{
  Serial.begin(9600);
  Wire.begin(55);
  Wire.onRequest(requestEvent);
  pinMode(pin, INPUT_PULLUP);
}

void loop()
{
  delay(100);
  

}

void requestEvent()
{
  int bigNum = 9999;
  byte myArray[2];


  myArray[0] = (bigNum >> 8) & 0xFF;
  myArray[1] = bigNum & 0xFF;

  Wire.write(myArray, 2);
}
