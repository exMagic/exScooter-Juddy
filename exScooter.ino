#include <Wire.h>
#include <RTClib.h>
RTC_DS1307 RTC;
////////////////////////////////GYRO////////////////////////////////////////////
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
//MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



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
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
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
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////
int bigNum;
byte a, b;
int rmp3;
/*/////////////////////////////////--Display--////////////////////////////////////*/
#include <U8glib.h>
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_FAST); // Dev 0, Fast I2C / TWI
int btn = 2; // btn display
int draw_state = 0;
int f11 = 24; // GND for display
/*/////////////////////////////////--RPM--////////////////////////////////////*/
int pinRPM = A2;
float rpm = 0;
double rpm2;
double timerrevCalc = 0;
double duration;
/*///////////////////////////////--RGB--//////////////////////////////////////*/
#include <Adafruit_NeoPixel.h>
#define PIN 12
int NUMPIXELS = 50;
int stopBtn = 11;
int leftBtn = 9;
int rightBtn = 10;
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

/*/////////////////////////////////--smoth--////////////////////////////////////*/
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = A0;
/*/////////////////////////////////////////////////////////////////////*/
unsigned long time;
int prevTime;
int loopTime;


void desk1(void) {
  DateTime now = RTC.now();
  /*///////////////////////////////--Dispaly Time--//////////////////////////////////////*/
  u8g.setFont(u8g_font_fur14);
  u8g.setPrintPos(0, 35);
  u8g.print(now.hour(), DEC);
  u8g.print(":");
  if ((now.minute()) <= 9) {
    u8g.print("0");
    u8g.print(now.minute(), DEC);
  }
  else {
    u8g.print(now.minute(), DEC);
  }
  u8g.setFont(u8g_font_7x13B);
  u8g.setPrintPos(87, 30);
  u8g.print(rmp3);

  /*///////////////////////////////--Dispaly TEMP and Date--//////////////////////////////////////*/
  int odczytEnd = tempAv();
  u8g.setFont(u8g_font_7x13B);
  u8g.setPrintPos(0, 60);
  u8g.print("@");
  u8g.print(odczytEnd);
  u8g.setPrintPos(50, 60);
  u8g.print(now.day(), DEC);
  u8g.print("/");
  u8g.print(now.month(), DEC);
  u8g.print("/");
  u8g.print(now.year(), DEC);
}
void desk2(void) {
  u8g.setFont(u8g_font_fur14);
  u8g.setPrintPos(0, 32);
  u8g.print("exScooter");
  u8g.setFont(u8g_font_fur14);
  u8g.setPrintPos(0, 60);
  u8g.print("______------");
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
  u8g.setFont(u8g_font_fub30r);
  u8g.setPrintPos(0, 40);
  u8g.print(rpm);

}
void we(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 20);
  u8g.print(rpm2);
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
void setRightLed(int R, int G, int B) {
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
void setLeftLed(int R, int G, int B) {
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

  pixels.show();
}
void leftAc() {
  setLeftLed(255, 40, 0);
}
void rightAc() {
  setRightLed(255, 40, 0);
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

//#####################################################################################################################################
//#####################################################################################################################################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                          SETUP                                                             ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#####################################################################################################################################
//#####################################################################################################################################

void setup(void) {
  //////////////////////////////////////////GYRO/////////////////////////////////////////////////////////
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(78);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(81);
  mpu.setZAccelOffset(1600); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
  pinMode(LED_PIN, OUTPUT);
 /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////
  pinMode(f11, OUTPUT);
  /*//////////////////////////////RPM///////////////////////////////////////*/
  pinMode(pinRPM, INPUT_PULLUP);
  /*////////////////////////////////RGB tyl/////////////////////////////////////*/
//  pinMode(stopBtn, INPUT_PULLUP);
//  pinMode(leftBtn, INPUT_PULLUP);
//  pinMode(rightBtn, INPUT_PULLUP);
//  pinMode(policeBtn, INPUT_PULLUP);
//  pinMode(A1, INPUT_PULLUP);
//  pixels.begin();
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
  //Serial.begin(9600);
  Wire.begin();


  ///////////////////////////////////RPM
  int16_t bigNum;
  byte a, b;
  Wire.requestFrom(54, 2);
  a = Wire.read();
  b = Wire.read();
  bigNum = a;
  bigNum = (bigNum << 8) | b;
  Serial.print(a);
  Serial.print("\n");
  /////////////////////////////////////SPEED
  int16_t bigNum2;
  byte a2, b2;
  Wire.requestFrom(55, 2);
  a2 = Wire.read();
  b2 = Wire.read();
  bigNum2 = a2;
  bigNum2 = (bigNum2 << 8) | b2;
  Serial.print(a2);
  Serial.print("\n");




  RTC.begin();
  RTC.adjust(DateTime(__DATE__, __TIME__));
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
  }
}

//#####################################################################################################################################
//#####################################################################################################################################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                          LOOP                                                              ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#################                                                                                            ########################
//#####################################################################################################################################
//#####################################################################################################################################

void loop(void) {
  /////////////////////////////////////////////////////////////////GYRO
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  //   wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
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

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print(",00");
    //Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print(",");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    
#endif

//#ifdef OUTPUT_READABLE_REALACCEL
//    // display real acceleration, adjusted to remove gravity
//    mpu.dmpGetQuaternion(&q, fifoBuffer);
//    mpu.dmpGetAccel(&aa, fifoBuffer);
//    mpu.dmpGetGravity(&gravity, &q);
//    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//    Serial.print(",");
//    Serial.print(aaReal.x);
//    Serial.print(",");
//    Serial.print(aaReal.y);
//    Serial.print(",");
//    Serial.println(aaReal.z);
//#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  /////////////////////////////////////////////////////////////////
  
  //delay(2000);
  ////////////////////////////////////RPM
  int32_t bigNum;
  byte a, b, c, d;
  Wire.requestFrom(54, 4);
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();
  bigNum = a;
  bigNum = (bigNum << 8) | b;
  rmp3 = bigNum;
  Serial.print(bigNum);

  ////////////////////////////////////SPEED
  int32_t bigNum2;
  byte a2, b2, c2, d2;
  Wire.requestFrom(55, 4);
  a2 = Wire.read();
  b2 = Wire.read();
  c2 = Wire.read();
  d2 = Wire.read();
  bigNum2 = a2;
  bigNum2 = (bigNum2 << 8) | b2;
  //rmp32 = bigNum2;
  Serial.print("\t");
  Serial.print(bigNum2);

  ////////////////////////////////////TMP
  int temp5 = tempAv();
  Serial.print("\t");
  Serial.print(temp5);

  ////////////////////////////////////LOOP TIME
  time = millis();
  //prints time since program started
  loopTime = time - prevTime;
  Serial.print("\t");
  Serial.print(loopTime);
  prevTime = time;


  ////////////////////////////////////TIME
  Serial.print("\t");
  DateTime now = RTC.now();
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print("  ");
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.println(now.year(), DEC);






  duration = pulseIn(pinRPM, HIGH, 30000);
  if (duration <= 0 ) {
    rpm2 = 0.0;
  }
  else {
    timerrevCalc = duration * 60;
    rpm = 60000000 / timerrevCalc / 100;
    rpm2 = rpm;
  }



  /*///////////////////////////////--check state of indicators--//////////////////////////////////////*/
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
    // delay(1);
  }
  else if (digitalRead (stopBtn) == HIGH) {
    setAllLed(30, 0, 0);
    //delay(1);
  }
  else {
    setAllLed(0, 0, 0);
  }
  /*/////////////////////////////////--picture loop--////////////////////////////////////////////*/
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage() );
  if (digitalRead(btn) == LOW) {
    draw_state++;
    delay(300);
    if ( draw_state >= 3 ) {
      draw_state = 0;
    }
  }
}
