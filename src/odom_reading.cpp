#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

double D_yaw, D_pitch, D_roll, F_cur_x, F_cur_y;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getEulerYPR(D_yaw, D_pitch, D_roll);

  D_yaw = (D_yaw * 180 / 3.14159265359);
  F_cur_x = msg->pose.pose.position.x;
  F_cur_y = msg->pose.pose.position.y;

  printf("Current yaw = %lf\n", D_yaw);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_reading");
	ros::NodeHandle n;
	ros::Subscriber subodom = n.subscribe("odom", 10, odomCallback);

	ros::spin();
	return 0;

}

