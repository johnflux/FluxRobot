#define DISTANCE_SENSOR_trigPin_1 11
#define DISTANCE_SENSOR_echoPin_1 12
#define DISTANCE_SENSOR_trigPin_2 7
#define DISTANCE_SENSOR_echoPin_2 8
#define MOUTH_LED_pin 4


#include "distance_sensor.h"
#include "mouth_leds.h"
#include "ros_communication.h"

RosCommunication rosCommunication;
DistanceSensor distanceSensor;
MouthLeds mouthLeds;

void setup() {
  Serial.begin(115200);
  distanceSensor.setup(DISTANCE_SENSOR_trigPin_1, DISTANCE_SENSOR_echoPin_1,
                       DISTANCE_SENSOR_trigPin_2, DISTANCE_SENSOR_echoPin_2);
  mouthLeds.setup(MOUTH_LED_pin);
  rosCommunication.setup();
}

void loop() {
  bool ok;
  float distance1;
  float distance2;
  distanceSensor.getDistance_cm(ok, distance1, distance2);
  if(ok) {
    mouthLeds.setMouthWidth(distance1/10.0 + 0.5);
    rosCommunication.sendDistanceInfo(distance1, distance2);
  }
}
