#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

std_msgs::UInt8 msg;
ros::Publisher ard_chatter("ard_chatter", &msg);

void messageCb(const std_msgs::UInt8& chat)
{
  msg.data = chat.data;
  ard_chatter.publish(&msg);
}

ros::Subscriber<std_msgs::UInt8> sub("ros_chatter", &messageCb);

void setup()
{
  nh.initNode();
  nh.advertise(ard_chatter);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
