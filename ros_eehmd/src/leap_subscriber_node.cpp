#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leapmsg.h"

// callback which displays the x coordinate of the thumb_metacarpal measurement
void chatterCallback(const ros_eehmd::leapmsgConstPtr& leap_msg )
{
  ROS_INFO("I heard: x coordinate of thumb metacarpal= [%f]", leap_msg->thumb_metacarpal.x);
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "leap_motion_subscriber");

 ros::NodeHandle n;

ros::Subscriber subleap = n.subscribe("Leapmotion_raw", 1000, chatterCallback);

 ros::spin();

  return 0;
}   
