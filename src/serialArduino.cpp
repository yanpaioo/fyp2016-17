

#include "ros/ros.h"
#include "std_msgs/Int8.h"

void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("Received: [%d]", msg->data);

  if (msg->data ==0)
  {
    ROS_INFO("Lifting...");
  }
  else if (msg->data == 1)
  {
    ROS_INFO("Lifting up done!");
  }
  else if (msg->data == 2)
  {
    ROS_INFO("Lifting failed!");
  }
  else if (msg->data == 3)
  {
    ROS_INFO("Dropping...");
  }
  else if (msg->data == 4)
  {
    ROS_INFO("Dropping done!");
  }
  else if (msg->data == 5)
  {
    ROS_INFO("Dropping failed");
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("toROS", 1000, chatterCallback);

  ros::spin();

  return 0;
}
