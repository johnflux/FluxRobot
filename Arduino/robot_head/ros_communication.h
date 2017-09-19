
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

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
      ;
      nh.initNode();
      nh.advertise(pub_range1);
      nh.advertise(pub_range2);
      setupDistanceSensorMessage();
    }
    
    void sendDistanceInfo(float distance1_cm, float distance2_cm) {
      range_msg.range = distance1_cm/10.0;
      range_msg.header.stamp = nh.now();
      range_msg.header.frame_id =  frameid1;
      pub_range1.publish(&range_msg);

      range_msg.range = distance2_cm/10.0;
      range_msg.header.frame_id =  frameid2;
      pub_range2.publish(&range_msg);
      nh.spinOnce();
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


