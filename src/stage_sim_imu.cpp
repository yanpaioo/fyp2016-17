#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string.h>
#include <stdlib.h> 
#include <stdio.h> 
#include "tf/tf.h"
#include <tf/transform_datatypes.h>


double _roll, _pitch, _yaw;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{  
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3(q).getEulerYPR(_yaw, _pitch, _roll);

	//tf::Matrix3x3 poseOrientationMat(msg->pose.pose.orientation);
	//poseOrientationMat.getRPY(_roll, _pitch, _yaw);
  _yaw  = _yaw * 180 / 3.14159265359;
	std::cout << "Roll: " << _roll << ", Pitch: " << _pitch << ", Yaw: " << _yaw << std::endl;
}

int main(int argc, char** argv )
{

ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Subscriber subodom = n.subscribe("odom", 10, odomCallback);
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu_rpy", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "IMU RPY : " << _roll << " , " << _pitch << " , " << _yaw;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


}
