#define DISTANCE_SENSOR_trigPin_1 11
#define DISTANCE_SENSOR_echoPin_1 12
#define DISTANCE_SENSOR_trigPin_2 7
#define DISTANCE_SENSOR_echoPin_2 8
#define MOUTH_LED_pin 4

#include <ros.h>

#include "distance_sensor.h"
#include "mouth_leds.h"



DistanceSensor distanceSensor;
MouthLeds mouthLeds;

void setup() {
  Serial.begin(9600);
  distanceSensor.setup(DISTANCE_SENSOR_trigPin_1, DISTANCE_SENSOR_echoPin_1,
                       DISTANCE_SENSOR_trigPin_2, DISTANCE_SENSOR_echoPin_2);
  mouthLeds.setup(MOUTH_LED_pin);
}

void loop() {
  bool ok;
  double distance = distanceSensor.getDistance_cm(ok);
  mouthLeds.setMouthWidth(distance/10.0 + 0.5);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" okay= ");
  Serial.println(ok);
  
}
