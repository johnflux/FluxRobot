/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers
Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed

Adafruit_DCMotor *motorMiddleLeft = AFMStop.getMotor(2);    //Backwards
Adafruit_DCMotor *motorFrontLeft = AFMStop.getMotor(3);     // Forwards
Adafruit_DCMotor *motorBackLeft = AFMStop.getMotor(1);      // Backwards
Adafruit_DCMotor *motorBackRight = AFMSbot.getMotor(2);     // Backwards
Adafruit_DCMotor *motorFrontRight = AFMSbot.getMotor(3);    // Backwards
Adafruit_DCMotor *motorMiddleRight = AFMSbot.getMotor(4);    // Backwards


void setup() {
  while (!Serial);
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("MMMMotor party!");

  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
   
  // turn on the DC motor
  motorMiddleLeft->setSpeed(100);
  motorMiddleLeft->run(BACKWARD);
  motorFrontLeft->setSpeed(100);
  motorFrontLeft->run(FORWARD);
  motorBackLeft->setSpeed(100);
  motorBackLeft->run(BACKWARD);

  motorBackRight->setSpeed(100);
  motorBackRight->run(BACKWARD);
  motorFrontRight->setSpeed(100);
  motorFrontRight->run(BACKWARD);
  motorMiddleRight->setSpeed(100);
  motorMiddleRight->run(BACKWARD);
}

void loop() {
}
