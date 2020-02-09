#define DISTANCE_SENSOR_trigPin_1 11
#define DISTANCE_SENSOR_echoPin_1 12
#define DISTANCE_SENSOR_trigPin_2 7
#define DISTANCE_SENSOR_echoPin_2 8
#define MOUTH_LED_pin 5
#define ARM_LED_pin 4

/* If you want to modify the BAUDRATE, modify the ros_serial bash script as well */
#define BAUDRATE 115200

//#include "distance_sensor.h"
#include "AllEffects_NeoPixel.h"

//DistanceSensor distanceSensor;
Leds mouthLeds(MOUTH_LED_pin, 8 /*num_leds*/);
Leds armLed(ARM_LED_pin, 1 /* num_leds */, 255 /* Full brightness*/);

void setup() {
  Serial.begin(BAUDRATE);
  while(!Serial) {
     // wait
  }
  //distanceSensor.setup(DISTANCE_SENSOR_trigPin_1, DISTANCE_SENSOR_echoPin_1,
  //                     DISTANCE_SENSOR_trigPin_2, DISTANCE_SENSOR_echoPin_2);
  mouthLeds.setup();
  armLed.setup();
  for (int i= 0; i < 3; ++i) {
    mouthLeds.setAll(255,0,0);
    armLed.setAll(255,0,0);
    delay(100);
    mouthLeds.setAll(0,255,0);
    armLed.setAll(0,255,0);
    delay(100);
    mouthLeds.setAll(0,0,255);
    armLed.setAll(0,0,255);
    delay(100);
  }
  mouthLeds.setAll(0,0,0);
  armLed.setAll(0,0,0);
}

void readFromSerial() {
  char c;
  while((c = Serial.read()) != -1) {
    //Serial.print("Found:");
    Serial.print(c);
    if (c >= '0' && c <= '9') {
      //Serial.print("showing digit ");
      //Serial.println((int)(c-'0'));
      mouthLeds.showNumber(c - '0');
      delay(1500);
      mouthLeds.clear();
      delay(200);
    } else if (c == '.') {
      //Serial.println("showing dot");
      mouthLeds.showDot();
      delay(1000);
      mouthLeds.clear();
      delay(200);
    } else if (c == 'r') {
      armLed.setAll(255,0,0);
      Serial.print("set to red");
    } else if (c == 'g') {
      armLed.setAll(0,255,0);
      Serial.print("set to green");
    } else if (c == 'b') {
      armLed.setAll(0,0,255);
      Serial.print("set to blue");
    } else if (c == 'o') {
      armLed.setAll(0,0,0);
      Serial.print("set to off");
    }
  }
}

void loop() {
  Serial.println("Send an ip address like 2.4.5.2, or one of r,g,b or o (for off) for the arm led");
  mouthLeds.loop();
  for (int i = 0; i < random(50,100); ++i) {
    readFromSerial();
    delay(100);
  }
  readFromSerial();
}
