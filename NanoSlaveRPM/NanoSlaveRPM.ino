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
  Wire.begin(54);
  Wire.onRequest(requestEvent);
  pinMode(pin, INPUT_PULLUP);
}

void loop()
{
  //delay(100);
  duration = pulseIn(pin, HIGH);
  Serial.print(duration);
    Serial.print("           //////     ");
  Serial.println(rpm2);
  if (duration <= 0 ) {
    rpm2 = 0.0;
  }
  else {
    timerrevCalc = duration * 60;
    rpm = 60000000 / timerrevCalc / 100;
    rpm2 = rpm*100;
  }

}

void requestEvent()
{
  int bigNum = rpm2;
  byte myArray[2];


  myArray[0] = (bigNum >> 8) & 0xFF;
  myArray[1] = bigNum & 0xFF;

  Wire.write(myArray, 2);
}
