#include <ros.h>
#include <geometry_msgs/Twist.h>

namespace ros {
  // Make these numbers as small as possible to save memory
  typedef NodeHandle_<ArduinoHardware, 1/*max subscribers*/, 2/*max publishers*/, 100/*input size*/, 100/*output size*/> NodeHandle;
};

#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "ApplicationMonitor.h"

class RosCommunication {

  private:
    ros::NodeHandle  nh;
    ros::Subscriber<geometry_msgs::Twist> sub_twist;
    ros::Publisher pub_twist;
    geometry_msgs::Twist pub_twist_msg;

    const char *frameid = "/cmd_vel";

  public:

    RosCommunication(void (*doTwistCallback)(const geometry_msgs::Twist&)) :
                     sub_twist(ros::Subscriber<geometry_msgs::Twist>(frameid, doTwistCallback)),
                     pub_twist(ros::Publisher(frameid, &pub_twist_msg)) {
    }

    void setup() {
      nh.getHardware()->setBaud(BAUDRATE);
      nh.initNode();
      nh.subscribe(sub_twist);
      nh.advertise(pub_twist);
    }

    bool isConnected() {
      return nh.connected();
    }

    void spinOnce() {
      nh.spinOnce();
    }

    void logfatal(const char *msg, uint32_t number) {
      char buf[30];
      snprintf_P(buf, 30, msg, number);
      nh.logfatal(buf);
    }

    void dumpWatchdogInfo(bool bOnlyIfPresent, const Watchdog::CApplicationMonitor &ApplicationMonitor) {

      Watchdog::CApplicationMonitorHeader Header;
      Watchdog::CCrashReport Report;
      uint8_t uReport;
      uint32_t uAddress;

      ApplicationMonitor.LoadHeader(Header);
      if (!bOnlyIfPresent || Header.m_uSavedReports != 0)
      {
        nh.logfatal("Logged crashes:");

        for (uReport = 0; uReport < Header.m_uSavedReports; ++uReport)
        {
          ApplicationMonitor.LoadReport(uReport, Report);
          uAddress = 0;
          memcpy(&uAddress, Report.m_auAddress, PROGRAM_COUNTER_SIZE);
          logfatal(PSTR(": byte-address= 0x%x"), (uint32_t)uAddress*2);
        }
      }
    }

    void sendCrashInfo(uint16_t uAddress) {
      logfatal(PSTR("Just crashed!: byte-address= 0x%x"), ((uint32_t)uAddress)*2);
    }

    void sendTwistCommand( float joyX, float joyY ) {
      pub_twist_msg.linear.x = joyY;
      pub_twist_msg.angular.z = joyX;
      pub_twist_msg.header.frame_id =  frameid;
      pub_twist_msg.header.stamp = nh.now();

      pub_twist.publish(pub_twist_msg);
    }
};


