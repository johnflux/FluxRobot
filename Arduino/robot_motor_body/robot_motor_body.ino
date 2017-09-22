
/* If you want to modify the BAUDRATE, modify the ros_serial bash script as well */
#define BAUDRATE 115200

void custom_crash_function(uint16_t return_address);

#include "motors.h"
#include "ApplicationMonitor.h"
#include "ros_communication.h"

Watchdog::CApplicationMonitor ApplicationMonitor;
RosCommunication rosCommunication;
Motors motors;

bool hasConnected=false;

void custom_crash_function(uint16_t return_address) {
  rosCommunication.sendCrashInfo(return_address);
}

void setup() {
  Serial.begin(BAUDRATE);
  ApplicationMonitor.Dump(Serial, false);
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_2s); // This should be plenty of time to do anything - sonar requires just 60ms between measurements

  motors.setup();
  rosCommunication.setup();
}



void loop() {
  // the program is alive...for now. 
  ApplicationMonitor.IAmAlive();
  if (!hasConnected && rosCommunication.isConnected()) {
    hasConnected = true;
    rosCommunication.dumpWatchdogInfo(false, ApplicationMonitor);
  }
  rosCommunication.spinOnce();
}

