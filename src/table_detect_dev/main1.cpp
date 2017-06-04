#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

/*
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser: [%f]", msg->ranges[0]);
}


int main(int argc, char **argv)
{

  float ranges[10] = {1, 1, 1, 5, 1, 1, 1, 5, 1, 1};
  float fil_ranges[10];
  float pre_val;
  
  
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<sensor_msgs::LaserScan>("scan1", 1000);


  while (ros::ok())
  {
    sensor_msgs::LaserScan scan1;
    pre_val = ranges[0];
    for (int i = 1; i < 10; ++i)
    {
      if (abs(pre_val - ranges[i]) > 3)
      {
	fil_ranges[i] = ranges[i];
      }
      else
      {
	fil_ranges[i] = 0;
      }
      pre_val = ranges[i];
      ROS_INFO("%f", fil_ranges[i]);
    }
    ros::spinOnce();
  }
  
  
  return 0;
}
*/

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser: [%f]", msg->ranges[0]);
}


int main(int argc, char **argv)
{

  float ranges[3] = {1, 2, 3};
  float fil_ranges[3];
  
  
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<sensor_msgs::LaserScan>("scan1", 1000);


  while (ros::ok())
  {
    sensor_msgs::LaserScan scan1;
    scan1.ranges.resize(513);
    for (int i = 0; i < 3; ++i)
    {
      scan1.ranges[i] = ranges[i];
      //ROS_INFO("fil_ranges: [%f]", fil_ranges[i]);
    }
    ros::spinOnce();
  }
  
  
  return 0;
} 