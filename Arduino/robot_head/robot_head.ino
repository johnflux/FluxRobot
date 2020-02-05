#define DISTANCE_SENSOR_trigPin_1 11
#define DISTANCE_SENSOR_echoPin_1 12
#define DISTANCE_SENSOR_trigPin_2 7
#define DISTANCE_SENSOR_echoPin_2 8
#define MOUTH_LED_pin 4

/* If you want to modify the BAUDRATE, modify the ros_serial bash script as well */
#define BAUDRATE 115200

void custom_crash_function(uint16_t return_address);

#include <ApplicationMonitor.h>
#include "distance_sensor.h"
#include "AllEffects_NeoPixel.h"
#include "ros_communication.h"

Watchdog::CApplicationMonitor ApplicationMonitor;
RosCommunication rosCommunication;
DistanceSensor distanceSensor;
MouthLeds mouthLeds;

bool hasConnected=false;

void custom_crash_function(uint16_t return_address) {
  rosCommunication.sendCrashInfo(return_address);
}

void setup() {
  Serial.begin(BAUDRATE); // Change via the #define if you want to change this, since ros also needs to know about this, in ros_communication.h
  //ApplicationMonitor.Dump(Serial, false);
  //ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_2s); // This should be plenty of time to do anything - sonar requires just 60ms between measurements

  //distanceSensor.setup(DISTANCE_SENSOR_trigPin_1, DISTANCE_SENSOR_echoPin_1,
  //                     DISTANCE_SENSOR_trigPin_2, DISTANCE_SENSOR_echoPin_2);
  mouthLeds.setup(MOUTH_LED_pin);
  //rosCommunication.setup();
}

void readFromSerial() {
  while(Serial.available() > 0) {
    char c = Serial.read();
    //Serial.print("Found:");
    Serial.print(c);
    if (c >= '0' && c <= '9') {
      //Serial.print("showing digit ");
      //Serial.println((int)(c-'0'));
      mouthLeds.showNumber(c - '0');
      delay(3000);
    } else if(c == '.') {
      //Serial.println("showing dot");
      mouthLeds.showDot();
      delay(2000);
    }
  }
}

void loop() {
  /*bool ok1;
  bool ok2;
  float distance1_m;
  float distance2_m;*/
  Serial.println("Send an ip address like 2.4.5.2");
  mouthLeds.loop();
  for (int i = 0; i < random(50,100); ++i) {
    readFromSerial();
    delay(100);
  }
  readFromSerial();
  /*

  // the program is alive...for now. 
  ApplicationMonitor.IAmAlive();
  distanceSensor.getDistance_m(ok1, distance1_m, ok2, distance2_m);
  if(!rosCommunication.isConnected()) {
    Serial.println(distance1_m);
  } else if (!hasConnected) {
    hasConnected = true;
    rosCommunication.dumpWatchdogInfo(false, ApplicationMonitor);
  }
  if(ok1) {
    mouthLeds.setMouthWidth(distance1_m*10.0 + 0.5);
  } else {
    mouthLeds.setMouthBad();
  }
  rosCommunication.sendDistanceInfo(distance1_m, distance2_m);
  rosCommunication.spinOnce();*/
}
