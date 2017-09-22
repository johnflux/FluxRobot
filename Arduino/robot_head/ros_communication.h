
/* Replace ros.h with our own definition of a nodehandle, to save memory */

#include <ros.h>

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
    sensor_msgs::Range range_msg1;
    sensor_msgs::Range range_msg2;
    ros::Publisher pub_range1;
    ros::Publisher pub_range2;
    const char *frameid1 = "/ultrasound";
    const char *frameid2 = "/ultrasound2";


  public:

    RosCommunication() : pub_range1(ros::Publisher("/ultrasound", &range_msg1)), pub_range2(ros::Publisher("/ultrasound2", &range_msg2)) {
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

    void spinOnce() {
      nh.spinOnce();
    }
    
    void sendDistanceInfo(float distance1_m, float distance2_m) {
      range_msg1.range = distance1_m;
      range_msg2.header.stamp = range_msg1.header.stamp = nh.now();
      pub_range1.publish(&range_msg1);
      range_msg2.range = distance2_m;
      pub_range2.publish(&range_msg2);
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
      range_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range_msg1.header.frame_id =  frameid1;
      range_msg1.field_of_view = 0.01;
      range_msg1.min_range = 0;
      range_msg1.max_range = 100;

      range_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range_msg2.header.frame_id =  frameid2;
      range_msg2.field_of_view = 0.01;
      range_msg2.min_range = 0;
      range_msg2.max_range = 100;
    }


};


