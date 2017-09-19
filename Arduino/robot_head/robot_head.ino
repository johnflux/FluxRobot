#define DISTANCE_SENSOR_trigPin_1 11
#define DISTANCE_SENSOR_echoPin_1 12
#define DISTANCE_SENSOR_trigPin_2 7
#define DISTANCE_SENSOR_echoPin_2 8
#define MOUTH_LED_pin 4

#include <avr/wdt.h>

#include "distance_sensor.h"
#include "mouth_leds.h"
#include "ros_communication.h"

RosCommunication rosCommunication;
DistanceSensor distanceSensor;
MouthLeds mouthLeds;

void setup() {
  Serial.begin(115200);
  wdt_enable(WDTO_1S); // Enable watchdog timer - reset device if we die
  distanceSensor.setup(DISTANCE_SENSOR_trigPin_1, DISTANCE_SENSOR_echoPin_1,
                       DISTANCE_SENSOR_trigPin_2, DISTANCE_SENSOR_echoPin_2);
  mouthLeds.setup(MOUTH_LED_pin);
  rosCommunication.setup();
}

void loop() {
  bool ok;
  float distance1_m;
  float distance2_m;

  // the program is alive...for now. 
  wdt_reset();

  distanceSensor.getDistance_m(ok, distance1_m, distance2_m);
  if(ok) {
    mouthLeds.setMouthWidth(distance1_m*10.0 + 0.5);
    rosCommunication.sendDistanceInfo(distance1_m, distance2_m);
  } else {
    mouthLeds.setMouthBad();
  }
}
