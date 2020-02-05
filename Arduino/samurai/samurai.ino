
#include <Servo.h>

Servo up_down_servo;     // create servo object to control a servo
Servo left_right_servo;  // create servo object to control a servo

int servoAngleToMicroseconds(float angle) {
  /* 544 default used by Servo object, but ~800 on MG996R) */
  const int min_duty_cycle_us = MIN_PULSE_WIDTH;
  /* 2400 default used by Servo object, but ~2200 on MG996R) */
  const int max_duty_cycle_us = MAX_PULSE_WIDTH; 
  /* Default used by Servo object, but ~130 on MG996R */
  const int min_to_max_angle = 180;
  const int duty_cycle_us = min_duty_cycle_us + angle * (max_duty_cycle_us - min_duty_cycle_us)/min_to_max_angle;
  return duty_cycle_us;
}

void servoAccurateWrite(Servo &servo, float angle) {
  int duty_cycle_us = servoAngleToMicroseconds(angle);
  servo.writeMicroseconds(duty_cycle_us);
}

void setup() {
  up_down_servo.write(170); // Maximum is 170 and that's looking forward, and smallest is about 140 and that is looking a bit downwards
  up_down_servo.attach(8);  // attaches the servo on pin 8 to the servo object
  
  left_right_servo.write(170); // 155 is facing forward.  170 is looking to the left for him
  left_right_servo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  Serial.begin(115200);
  Serial.println("up_down, left_right");
}

void loop() {
  const float graduations = 1000.0;

  for (int pos = 0; pos < graduations; pos++) {
    const float up_down_angle = 160+10*sin(pos / graduations * 2.0 * 3.1415);
    const float left_right_angle = 155+5*cos(pos / graduations * 2.0 * 3.1415);
    const int left_right_us = servoAngleToMicroseconds(left_right_angle);
    const int up_down_us = servoAngleToMicroseconds(up_down_angle);

    left_right_servo.writeMicroseconds(left_right_us);
    up_down_servo.writeMicroseconds(up_down_us);

    servoAccurateWrite(left_right_servo, left_right_angle);
    servoAccurateWrite(up_down_servo, up_down_angle);
    Serial.print(up_down_angle);
    Serial.print(",");
    Serial.println(up_down_us);
    delay(15);
  }
}
