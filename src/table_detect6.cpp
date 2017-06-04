#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>

ros::Publisher Marker_pub;
ros::Publisher Table_pub;
ros::Publisher Laser_pub;

sensor_msgs::LaserScan Msg_laser;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 40;
  float f_clus_sum = 0;
  int i_no_readings = 0;
  float f_range_thres = 2.5;
  float f_clus_centre[50][2];     // {x, y}
  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.15;

  Msg_laser.angle_increment = msg->angle_increment;
  Msg_laser.angle_min = msg->angle_min;
  Msg_laser.angle_max = msg->angle_max;
  Msg_laser.time_increment = msg->time_increment;
  Msg_laser.scan_time = msg->scan_time;
  Msg_laser.range_min = msg->range_min;
  Msg_laser.range_max = msg->range_max;
  Msg_laser.header.frame_id = "laser";

  visualization_msgs::Marker points;
  points.header.frame_id = "laser";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  i_no_readings = 1 + ((Msg_laser.angle_max - Msg_laser.angle_min)/Msg_laser.angle_increment);
  Msg_laser.ranges.resize(i_no_readings);

  for (int i = 0; i < i_no_readings; ++i)
  {
    Msg_laser.ranges[i] = msg->ranges[i];
  }
  Laser_pub.publish(Msg_laser);

  for (int i = 1; i < i_no_readings; ++i)
  {
    f_pre_pose[0] = Msg_laser.ranges[i-1] * sin((i-1) * Msg_laser.angle_increment);
    f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1] * cos((i-1) * Msg_laser.angle_increment);
    f_cur_pose[0] = Msg_laser.ranges[i] * sin((i) * Msg_laser.angle_increment);
    f_cur_pose[1] = (-1) * Msg_laser.ranges[i] * cos((i) * Msg_laser.angle_increment);
    f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));

    while(f_dist_diff < f_diff_thres)
    {
      if (i_no_readings < i+i_pt_ct) break;
      i_pt_ct++;
      f_pre_pose[0] = Msg_laser.ranges[i-1+i_pt_ct] * sin((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1+i_pt_ct] * cos((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[0] = Msg_laser.ranges[i+i_pt_ct] * sin((i+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[1] = (-1) * Msg_laser.ranges[i+i_pt_ct] * cos((i+i_pt_ct) * Msg_laser.angle_increment);
      f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));
    }
    ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    i += i_pt_ct;
    i_pt_ct = 0;

  }

  Marker_pub.publish(points);
}

int main(int argc, char** argv )
{
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle n;
  Marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  //Table_pub = n.advertise<visualization_msgs::Marker>("table_marker", 10);
  ros::Subscriber sublaser = n.subscribe("base_scan", 10, laserCallback);

  ros::spin();
}