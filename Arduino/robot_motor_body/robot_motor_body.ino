#include <ros.h>
#include <geometry_msgs/Twist.h>

/* If you want to modify the BAUDRATE, modify the ros_serial bash script as well */
#define BAUDRATE 115200

void custom_crash_function(uint16_t return_address);

#include "motors.h"
#include "ApplicationMonitor.h"
#include "ros_communication.h"

void doTwistCallback(const geometry_msgs::Twist& msg);
Watchdog::CApplicationMonitor ApplicationMonitor;
RosCommunication rosCommunication(doTwistCallback);
Motors motors;
JoystickControl joystick;

bool hasConnected=false;

void custom_crash_function(uint16_t return_address) {
  rosCommunication.sendCrashInfo(return_address);
}

void doTwistCallback(const geometry_msgs::Twist& msg) {
  if (msg.angular.z == 0 && msg.linear.x == 0)
  {
    motors.stopAll();
  }
  else
  {
    servoTurnAngle(msg.angular.z);
    allMotorsSetSpeed(msg.linear.x * 100);
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  ApplicationMonitor.Dump(Serial, false);
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_2s);

  motors.setup();
  rosCommunication.setup();
  joystick.setup();
}

void loop() {
  // the program is alive...for now.
  ApplicationMonitor.IAmAlive();
  if (!hasConnected && rosCommunication.isConnected()) {
    hasConnected = true;
    rosCommunication.dumpWatchdogInfo(false, ApplicationMonitor);
  }

  int joyX = joystick.joyXValue();
  int joyY = joystick.joyYValue();
  rosCommunication.sendTwistCommand( joyX, joyY );
  rosCommunication.spinOnce();
}

