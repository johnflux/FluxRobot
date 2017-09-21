
/* Replace ros.h with our own definition of a nodehandle, to save memory */

#include <ros.h>

namespace ros {
  // Make these numbers as small as possible to save memory
  typedef NodeHandle_<ArduinoHardware, 2/*max subscribers*/, 4/*max publishers*/, 100/*input size*/, 200/*output size*/> NodeHandle;
};

#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "ApplicationMonitor.h"

class RosCommunication {

  private:
    ros::NodeHandle  nh;
    sensor_msgs::Range range_msg;
    ros::Publisher pub_range1;
    ros::Publisher pub_range2;
    const char *frameid1 = "/ultrasound";
    const char *frameid2 = "/ultrasound2";


  public:

    RosCommunication() : pub_range1(ros::Publisher("/ultrasound", &range_msg)), pub_range2(ros::Publisher("/ultrasound2", &range_msg)) {
    }

    void setup() {
      nh.getHardware()->setBaud(BAUDRATE);
      nh.initNode();
      nh.advertise(pub_range1);
      nh.advertise(pub_range2);
      setupDistanceSensorMessage();
    }

    bool isConnected() {
      return nh.connected();
    }
    
    void sendDistanceInfo(float distance1_m, float distance2_m) {
      range_msg.range = distance1_m;
      range_msg.header.stamp = nh.now();
      range_msg.header.frame_id =  frameid1;
      pub_range1.publish(&range_msg);
      range_msg.range = distance2_m;
      range_msg.header.frame_id =  frameid2;
      pub_range2.publish(&range_msg);
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


   private:
    void setupDistanceSensorMessage() {
      range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range_msg.header.frame_id =  frameid1;
      range_msg.field_of_view = 0.01;
      range_msg.min_range = 0;
      range_msg.max_range = 3;
    }


};


