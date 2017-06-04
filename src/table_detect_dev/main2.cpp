#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <sstream>
//#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
//#include <nav_msgs/Odometry.h>
#include <cmath>


float gpre_val;
float gnum_readings;
float grange = 3; //range of consideration in m
sensor_msgs::LaserScan laserscan;


void filter()
{
  /*laserscan.ranges[0]=0;
  //val = laserscan.ranges[0];
  for (int i = 0; i < gnum_readings; ++i)
  {
    if (laserscan.ranges[i] > grange) laserscan.ranges[i] = 0.0; 
  }*/
  
}

void clustering()
{
  //main_pub.publish(laserscan);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  
  laserscan.angle_increment = msg->angle_increment;
  laserscan.angle_min = msg->angle_min;
  laserscan.angle_max = msg->angle_max;

  
  gnum_readings = 1 + ((laserscan.angle_max - laserscan.angle_min)/laserscan.angle_increment);
  laserscan.ranges.resize(gnum_readings);
   
  for (int i = 0; i < gnum_readings; ++i)
  {
    laserscan.ranges[i] = msg->ranges[i];
  }
  
  laserscan.ranges[0]=0;

  for (int i = 0; i < gnum_readings; ++i)
  {
    if (laserscan.ranges[i] > grange) laserscan.ranges[i] = 0.0; 
  }
  
  gpre_val = laserscan.ranges[0];
  
  for (int i = 1; i < gnum_readings; ++i)
  {
    if (abs(gpre_val - laserscan.ranges[i]) < 0.01) 
    {
      gpre_val = laserscan.ranges[i];
      laserscan.ranges[i] = 0;
    }
    
    else gpre_val = laserscan.ranges[i];
  }
  
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<sensor_msgs::LaserScan>("points", 100);
  ros::Subscriber sublaser = n.subscribe("scan", 100, laserCallback);
  
  
  while (ros::ok())
  {
    //filter();
    
    //clustering();
    
    
    main_pub.publish(laserscan);
    ros::spinOnce();
  }
  
  ros::spin();
  return 0;
} 