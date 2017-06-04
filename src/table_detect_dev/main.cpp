#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <list>

float ranges[513];

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  /*float prev_val;
  
  std::list<float> mylist;
  std::list<int> it;
  
  for (int i = 0; i <513; ++i) //filters out value >3
  {
    if (msg->ranges[i] > 3) ranges[i] = 0; //filters out value > 3
  }
  
  prev_val = ranges[0];
  for (int i = 0; i < 513; ++i)
  {
    if (abs(prev_val - ranges[i]) > 0.1)
    {
      it.push_back(i);
      mylist.push_back(ranges[i]);
      ROS_INFO("%f", mylist[i]);
    }
    
    prev_val = ranges[0];
  } */

}

/*void gyroCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Gyro: [%s]", msg->data.c_str());
} */

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Odom: [%f %f %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Subscriber sublaser = n.subscribe("scan", 1000, laserCallback);
  //ros::Subscriber subgyro = n.subscribe("gyro", 1000, gyroCallback);
  //ros::Subscriber subodom = n.subscribe("odom", 1000, odomCallback);
  ros::Publisher main_pub = n.advertise<geometry_msgs::Twist>("my_cmd_vel", 1000);

  //ros::Rate loop_rate(10);


  while (ros::ok())
  {
/*
    geometry_msgs::Twist msg;
    msg.linear.x = 1;
    msg.angular.z = 0;

    //ROS_INFO("x-%f z-%f", msg.linear.x, msg.angular.z);
    main_pub.publish(msg);
    */

    ros::spinOnce();

    //loop_rate.sleep();
  }

  return 0;
}

