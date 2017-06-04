#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

#define ROBOT 1

ros::Publisher Marker_pub;
ros::Publisher Chair_pub;
ros::Publisher Laser_pub;

sensor_msgs::LaserScan Msg_laser;

float F_chair_dim[2] = {0.3, 0.38};
float F_chair_dim_thres = 0.04;

float F_clus_centre[50][2];     // {x, y}
float F_table_pose[5][2];       // {center, 2 pts making length, 2 pts making width}
int I_length_maker;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int i_comb_ct = 0;
  int i_no_ptthres = 20;
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 20;
  float f_clus_sum = 0;
  int i_no_readings = 0;
  float f_range_thres = 1.5;

  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.05;

  Msg_laser.angle_increment = msg->angle_increment;
  Msg_laser.angle_min = msg->angle_min;
  Msg_laser.angle_max = msg->angle_max;
  Msg_laser.time_increment = msg->time_increment;
  Msg_laser.scan_time = msg->scan_time;
  Msg_laser.range_min = msg->range_min;
  Msg_laser.range_max = msg->range_max;

  #ifdef ROBOT  
  Msg_laser.header.frame_id = "base_laser_link";
#else
  Msg_laser.header.frame_id = "laser";
 #endif 

  visualization_msgs::Marker points;

#ifdef ROBOT  
  points.header.frame_id = "base_laser_link";
#else
  points.header.frame_id = "laser";
#endif

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

    if(isnan(Msg_laser.ranges[i]) || isinf(Msg_laser.ranges[i]))
    {
      Msg_laser.ranges[i] = 5;
    }
  }
  Laser_pub.publish(Msg_laser);

  for (int i = 1; i < i_no_readings; ++i)
  {
    f_pre_pose[0] = Msg_laser.ranges[i-1] * sin((i-1) * Msg_laser.angle_increment);
    f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1] * cos((i-1) * Msg_laser.angle_increment);
    f_cur_pose[0] = Msg_laser.ranges[i] * sin((i) * Msg_laser.angle_increment);
    f_cur_pose[1] = (-1) * Msg_laser.ranges[i] * cos((i) * Msg_laser.angle_increment);

    f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0], 2) + pow(f_cur_pose[1]-f_pre_pose[1], 2));

    while(f_dist_diff < f_diff_thres)
    {
      i_pt_ct++;
      if (i_no_readings < i+i_pt_ct) break;
      f_pre_pose[0] = Msg_laser.ranges[i-1+i_pt_ct] * sin((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1+i_pt_ct] * cos((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[0] = Msg_laser.ranges[i+i_pt_ct] * sin((i+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[1] = (-1) * Msg_laser.ranges[i+i_pt_ct] * cos((i+i_pt_ct) * Msg_laser.angle_increment);
      f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));
      
    }
    //ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    
    if (i_pt_ct == 0)
    {
      f_clus_sum = Msg_laser.ranges[i];
      F_clus_centre[i_clus_ct][0] = Msg_laser.ranges[i-1] * sin((i) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * Msg_laser.ranges[i-1] * cos((i) * Msg_laser.angle_increment);
    } 

    else
    {
      for (int y = 0; y < (i_pt_ct); ++y)
      {
        f_clus_sum += Msg_laser.ranges[i+y];
      }
      float ave;
      ave = f_clus_sum / (i_pt_ct);
      F_clus_centre[i_clus_ct][0] = ave * sin((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * ave * cos((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
    }

    float check_range;
    check_range = sqrt(pow(F_clus_centre[i_clus_ct][0], 2) + pow(F_clus_centre[i_clus_ct][1], 2));
    if ((check_range > f_range_thres) || (i_pt_ct > i_no_threspt))
    {
      F_clus_centre[i_clus_ct][0] = F_clus_centre[i_clus_ct][1] = 0;
    }
    else
    {
      i_clus_ct++;
    }
    
    i = i + i_pt_ct;
    i_pt_ct = 0;
    f_clus_sum = 0;
  }    


  for (int i = 0; i < i_clus_ct; ++i)
  {
    geometry_msgs::Point p;
    p.x = F_clus_centre[i][0];
    p.y = F_clus_centre[i][1];
    points.points.push_back(p);
  }
  
  Marker_pub.publish(points);

}


int main(int argc, char** argv )
{
  ros::init(argc, argv, "chair_detection");
  ros::NodeHandle n;
  Marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  Chair_pub = n.advertise<visualization_msgs::Marker>("chair_marker", 10);
  ros::Subscriber sublaser = n.subscribe("scan", 10, laserCallback);

  ros::spin();
}