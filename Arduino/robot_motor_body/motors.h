#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PCA9685.h>

Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers
Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed

Adafruit_DCMotor *motorFrontLeft = AFMStop.getMotor(3);     // Forwards
Adafruit_DCMotor *motorMiddleLeft = AFMStop.getMotor(2);    // Backwards
Adafruit_DCMotor *motorBackLeft = AFMStop.getMotor(1);      // Backwards
Adafruit_DCMotor *motorMiddleRight = AFMSbot.getMotor(4);   // Backwards
Adafruit_DCMotor *motorFrontRight = AFMSbot.getMotor(3);    // Backwards
Adafruit_DCMotor *motorBackRight = AFMSbot.getMotor(2);     // Backwards

#define NUM_MOTORS 6
const Adafruit_DCMotor *allMotors[NUM_MOTORS] = { motorFrontLeft, motorMiddleLeft, motorBackLeft, motorMiddleRight, motorFrontRight, motorBackRight };
const bool motorIsBackwards[NUM_MOTORS] = {false, true, true, true, true, true};

int servoBackRight = 0;
int servoBackLeft = 1;
int servoFrontRight = 15;
int servoFrontLeft = 14;

PCA9685 pwmController;                  // Library using default Wire and default linear phase balancing scheme
void allMotorsForward(int speed=100);

class Motors {

  public:
    void setup() {
      Wire.setClock(100000);              // Supported baud rates of the PCA9685 servo controller are 100kHz, 400kHz, and 1000kHz
      Wire.begin();
    
      pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line
    // pwmController.setChannelPWM(0, 128 << 4); // Set PWM to 128/255, but in 4096 land
    //pwmController.setPWMFrequency(200); // Default is 200Hz, supports 24Hz to 1526Hz
    
      AFMSbot.begin(); // Start the bottom shield
      AFMStop.begin(); // Start the top shield
    
      // We have motor driver on 0x60 (and broadcast all on 0x70) and servo driver on 0x40  
      pwmController.init(B000000);        // Address pins A5-A0 set to B000000
    
      Serial.println(pwmController.getChannelPWM(0)); // Should output 2048, which is 128 << 4
    
      setAnglePWM(servoBackRight,83);
      setAnglePWM(servoBackLeft,95);
      setAnglePWM(servoFrontRight,63);
      setAnglePWM(servoFrontLeft,90);
    
      allMotorsForward();
    }

    void setAnglePWM(int motorNum, int pwm) {
      pwmController.setChannelPWM(motorNum, pwm << 4); // Set PWM to 128/255, but in 4096 land
    }

    void servoTurnAngle(double angle) {
      setAnglePWM(servoFrontRight,63 + angle);
      setAnglePWM(servoFrontLeft,90 + angle);
    }

    void stopAll() {
      for( int i = 0; i < NUM_MOTORS; ++i)
        allMotors[i]->setSpeed(0);
    }

    void releaseAll() {
      for( int i = 0; i < NUM_MOTORS; ++i)
        allMotors[i]->run(RELEASE);
    }

    /* Linear speed forwards.  Can be negative to go backwards */
    void allMotorsSetSpeed(int speed=100) {
    {
      for( int i = 0; i < NUM_MOTORS; ++i) {
        allMotors[i]->setSpeed(abs(speed));
        const bool goBackwards == motorIsBackwards[i] == (speed > 0)
        allMotors[i]->run(goBackwards ? BACKWARD : FORWARD);
      }
    }
}

};

