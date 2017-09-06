#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;
//test
int as = 0; //jak szybko wykonuje sie pętla
int initial = 0;
int final = 0;

#include "U8glib.h"
int btn = 2;
int draw_state = 0;
//const int temperaturePin = A0;

/*/////////////////////////////////RPM////////////////////////////////////*/
int pinRPM = A2;
long rpm = 0;
double timerrevCalc = 0;
double duration;
/*///////////////////////////////FUEL//////////////////////////////////////*/
int f1 = 37;
int f2 = 35;
int f3 = 33;
int f4 = 31;
int f5 = 29;
int f6 = 27;
int f7 = 25;
int f8 = 38;
int f9 = 36;
int f10 = 34;
/*/////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////RGB tyl//////////////////////////////////////*/
#include <Adafruit_NeoPixel.h>
#define PIN 12
int NUMPIXELS = 50;
int stopBtn = 11;
int leftBtn = 10;
int rightBtn = 9;
int policeBtn = 8;
int D1 = 0;
double D2 = 0.5;
int R = 225;
int G = 0;
int B = 0;
int st = 10;
int it  = 1;
int pv = 50;
int pi  = 200;
int pausePolice = 30;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
/*/////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////smoth////////////////////////////////////*/
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = A0;
/*/////////////////////////////////////////////////////////////////////*/
//U8GLIB_SH1106_128X64 u8g(26, 24, 28, 30);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
U8GLIB_SH1106_128X64 u8g(30, 26, 24, 28);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9



void desk1(void) {
  DateTime now = RTC.now();
  u8g.setFont(u8g_font_fub30r);
  u8g.setPrintPos(0, 40);
  u8g.print(now.hour(), DEC);

  u8g.print(":");
  if ((now.minute()) <= 9) {
    u8g.print("0");
    u8g.print(now.minute(), DEC);
  }
  else {
    u8g.print(now.minute(), DEC);
  }
  u8g.setFont(u8g_font_fub11n);
  u8g.print(":");
  if ((now.second()) <= 9) {
    u8g.print("0");
    u8g.print(now.second(), DEC);
  }
  else {
    u8g.print(now.second(), DEC);
  }

  int odczytEnd = tempAv();
  u8g.setFont(u8g_font_04b_03br);
  u8g.setPrintPos(0, 62);
  u8g.print("@ ");
  u8g.print(odczytEnd);

  u8g.setPrintPos(50, 62);
  u8g.print(now.day(), DEC);
  u8g.print("/");
  u8g.print(now.month(), DEC);
  u8g.print("/");
  u8g.print(now.year(), DEC);
}

void desk2(void) {
  int odczytEnd = tempAv();
  u8g.setFont(u8g_font_fub30r);
  u8g.setPrintPos(0, 42);
  u8g.print("@ ");
  u8g.print(odczytEnd);

  u8g.setContrast(0);
  u8g.setFont(u8g_font_04b_03br);
  u8g.setPrintPos(0, 62);
  u8g.print("TAK BARDZO BARDZO KOCHAM CIĘ!");
}

float tempAv() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  //delay(1);        // delay in between reads for stability
  float voltage = (average * 0.004882814);
  float temp = (voltage - 0.5) * 100.0;
  return (temp);
}

void desk3(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 20);
  u8g.print("ELOx SIEMA CO TAM?");
}

void we(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 20);
  u8g.print("Welcome");
}

void draw(void) {
  switch (draw_state >> 0) {
    case 0: desk1(); break;
    case 1: desk2(); break;
    case 2: desk3(); break;
  }
}


/*/////////////////////////////////////////////////////////////////////*/
void setAllLed(int R, int G, int B) {
  pixels.setPixelColor(0, R, G, B);
  pixels.setPixelColor(1, R, G, B);
  pixels.setPixelColor(2, R, G, B);
  pixels.setPixelColor(3, R, G, B);
  pixels.setPixelColor(4, R, G, B);
  pixels.setPixelColor(5, R, G, B);
  pixels.setPixelColor(6, R, G, B);
  pixels.setPixelColor(7, R, G, B);
  pixels.setPixelColor(8, R, G, B);
  pixels.setPixelColor(9, R, G, B);
  pixels.setPixelColor(10, R, G, B);
  pixels.setPixelColor(11, R, G, B);
  pixels.setPixelColor(12, R, G, B);
  pixels.setPixelColor(13, R, G, B);
  pixels.setPixelColor(14, R, G, B);
  pixels.setPixelColor(15, R, G, B);
  pixels.setPixelColor(16, R, G, B);
  pixels.setPixelColor(17, R, G, B);
  pixels.setPixelColor(18, R, G, B);
  pixels.setPixelColor(19, R, G, B);
  pixels.setPixelColor(20, R, G, B);
  pixels.setPixelColor(21, R, G, B);
  pixels.setPixelColor(22, R, G, B);
  pixels.setPixelColor(23, R, G, B);
  pixels.setPixelColor(24, R, G, B);
  pixels.setPixelColor(25, R, G, B);
  pixels.setPixelColor(26, R, G, B);
  pixels.setPixelColor(27, R, G, B);
  pixels.setPixelColor(28, R, G, B);
  pixels.setPixelColor(29, R, G, B);
  pixels.setPixelColor(30, R, G, B);
  pixels.setPixelColor(31, R, G, B);
  pixels.setPixelColor(32, R, G, B);
  pixels.setPixelColor(33, R, G, B);
  pixels.setPixelColor(34, R, G, B);
  pixels.setPixelColor(35, R, G, B);
  pixels.setPixelColor(36, R, G, B);
  pixels.setPixelColor(37, R, G, B);
  pixels.setPixelColor(38, R, G, B);
  pixels.setPixelColor(39, R, G, B);
  pixels.setPixelColor(40, R, G, B);
  pixels.setPixelColor(41, R, G, B);
  pixels.setPixelColor(42, R, G, B);
  pixels.setPixelColor(43, R, G, B);
  pixels.setPixelColor(44, R, G, B);
  pixels.setPixelColor(45, R, G, B);
  pixels.setPixelColor(46, R, G, B);
  pixels.setPixelColor(47, R, G, B);
  pixels.setPixelColor(48, R, G, B);
  pixels.setPixelColor(49, R, G, B);
  pixels.setPixelColor(50, R, G, B);
  pixels.show();
}



void leftAc() {
  for (int i = 25; i <= NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(50);

  for (int i = 25; i <= NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(255, 40, 0));
    pixels.show();
    delay(it);
  }

}
void rightAc() {
  for (int i = 25; i >= 0; i--) {

    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(100);

  for (int i = 25; i >= 0; i--) {

    pixels.setPixelColor(i, pixels.Color(255, 40, 0));
    pixels.show();
    delay(it);
  }
  delay(200);
  for (int i = 25; i >= 0; i--) {

    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
    delay(it);
  }
  delay(100);

  for (int i = 25; i >= 0; i--) {

    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(0);
}

void blueBlink() {
  for (int i = NUMPIXELS; i >= 0; i--) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    pixels.show();
  }
  delay(pv);
  for (int j = NUMPIXELS; j >= 0; j--) {
    pixels.setPixelColor(j, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(pi);

}
void redBlink() {

  for (int i = NUMPIXELS; i >= 0; i--) {
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();

  }
  delay(pv);
  for (int j = NUMPIXELS; j >= 0; j--) {
    pixels.setPixelColor(j, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(pi);
}

void policeAc(int t) {

  for (int i = 0; i < t; i++) {
    for (int f = 0; f < 4; f++) {
      setAllLed(0, 0, 255);
      delay(pausePolice);
      setAllLed(0, 0, 0);
      delay(pausePolice);
    }
    delay(pi);
    for (int f = 0; f < 4; f++) {
      setAllLed(255, 0, 0);
      delay(pausePolice);
      setAllLed(0, 0, 0);
      delay(pausePolice);
    }
    delay(pi);
  }
}
/*/////////////////////////////////////////////////////////////////////*/


void setup(void) {

  /*//////////////////////////////RPM///////////////////////////////////////*/
  pinMode(pinRPM, INPUT_PULLUP);
  /*////////////////////////////////RGB tyl/////////////////////////////////////*/
  pinMode(stopBtn, INPUT_PULLUP);
  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(policeBtn, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pixels.begin();
  /*//////////////////////////////smoth///////////////////////////////////////*/
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  /*/////////////////////////////////////////////////////////////////////*/
  // flip screen, if required
  u8g.setRot180();

  pinMode(btn, INPUT_PULLUP);
  // flip screen, if required
  // u8g.setRot180();
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled

    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  /*
    // picture loop
    u8g.firstPage();
    do {
    we();

    } while ( u8g.nextPage() );
  */
}

void loop(void) {
  as++;


  /*
    duration = pulseIn(pinRPM, HIGH, 10000000);
    timerrevCalc = duration * 60;

    rpm = 60000000 / duration / 4;

    Serial.println(rpm);
  */


  /*/////////////////////////////////////////////////////////////////////*/
  if (digitalRead (policeBtn) == LOW) {
    policeAc(5);
  }
  else if (digitalRead (leftBtn) == LOW) {
    leftAc();
  }
  else if (digitalRead (rightBtn) == LOW) {
    rightAc();
  }
  else if (digitalRead (stopBtn) == LOW) {
    setAllLed(255, 0, 0);
    delay(1);
  }
  else if (digitalRead (stopBtn) == HIGH) {
    setAllLed(30, 0, 0);
    //delay(1);
  }
  else {
    setAllLed(0, 0, 0);
  }

  /*/////////////////////////////////////////////////////////////////////

  // picture loop
  /*u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage() );


  if (digitalRead(btn) == LOW) {
    draw_state++;
    //delay(30);
    if ( draw_state >= 3 ) {
      draw_state = 0;
    }

  }
  // rebuild the picture after some delay*/

  Serial.println(as);

  
}

