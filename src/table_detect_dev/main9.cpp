#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>

float gpre_val;
float gdiff;
float gtheta = 0;
float gnum_readings;
float thres = 0.1;
const float grange = 2.5; //range of consideration in m

ros::Publisher marker_pub;
ros::Publisher laser_pub;

using namespace std;

vector<int> pts;
vector<int> comb;

float position[10][2];
float table_pose[4][2];
float table_dim[3] = {67, 110, 129};


void go(int offset, int k)
{
  if ( k == 0)
  {
    ROS_INFO("%d %d %d", comb[0], comb[1], comb[2]);
    return;
  }
  for (int i = offset; i <= pts.size() - k; ++i)
  {
    comb.push_back(pts[i]);
    go(i+1, k-1);
    comb.pop_back();
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan laserscan;
  int count = 0;
  int num = 0;

  float x1, x2, y1, y2, dist;
  int ctr = 1;
  int i_count;
  int k = 3;

  pts.resize(0);
  comb.resize(0);


  laserscan.angle_increment = msg->angle_increment;
  laserscan.angle_min = msg->angle_min;
  laserscan.angle_max = msg->angle_max;

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = "laser";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.15;
  points.scale.y = 0.15;

  line_strip.scale.x = 0.05;

  points.color.g = 1.0f;
  points.color.a = 1.0;

  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  
  gnum_readings = 1 + ((laserscan.angle_max - laserscan.angle_min)/laserscan.angle_increment);
  laserscan.ranges.resize(gnum_readings);
   
  for (int i = 0; i < gnum_readings; ++i)
  {
    laserscan.ranges[i] = msg->ranges[i];
  }
  
  laserscan.ranges[0]=0;
  
  gpre_val = laserscan.ranges[0];

  for (int i = 1; i < gnum_readings; ++i)
  {
    i_count = i;
    gdiff = gpre_val - laserscan.ranges[i];

    if (gdiff < 0)
    {
      gdiff = -gdiff;
    }
    
    if (gdiff >= 0.20)
    {
      gpre_val = laserscan.ranges[i_count];

      gdiff = gpre_val - laserscan.ranges[i_count+1];

      if (gdiff < 0)
      {
        gdiff = -gdiff;
      }

      while (gdiff < 0.20)
      {

        if (gnum_readings == i_count+count+1) goto checking;
        gpre_val = laserscan.ranges[i_count+count+1];
        count++;

        gdiff = gpre_val - laserscan.ranges[i_count+count+1];

        if (gdiff < 0)
        {
          gdiff = -gdiff;
        }
      }
      checking:

      if ((count < 15) && (laserscan.ranges[i] <= 2.5))
      {
        position[num][0] = i;
        position[num][1] = laserscan.ranges[i_count];
        num++;
      }
      i = i + count;
      gpre_val = 0;
      count = 0;
        
    }
    else 
      {
        gpre_val = laserscan.ranges[i_count];
      }
  }

  if (num < 3) goto pointcheck;


  ROS_INFO("Num - %d", num);
  pts.resize(0);
  comb.resize(0);

  for (int i = 0; i < num; ++i)
  {
    pts.push_back(i);
  }
  go(0, k);
  
  pointcheck:
  
  for (int i = 0; i < (num-1); ++i)
  {
    float theta1, theta2;
    if (num < i+ctr) goto next;

    theta1 = position[i][0] * laserscan.angle_increment;
    theta2 = position[i+ctr][0] * laserscan.angle_increment;

    x1 = (position[i][1] * sin(theta1)) * 100;
    x2 = (position[i+ctr][1] * sin(theta2)) * 100;
    y1 = (position[i][1] * cos(theta1)) * (-100);
    y2 = (position[i+ctr][1] * cos(theta2)) * (-100);

    dist = pow(pow((x2-x1),2) + pow((y2-y1),2), 0.5);
    ROS_INFO("distance = %f, %d-%d", dist, i, i+1);
  }
  
  
  
  next:

  for (int i = 0; i < num; ++i)
  {
    geometry_msgs::Point p;
    gtheta = position[i][0] * laserscan.angle_increment;

    p.x = position[i][1] * sin(gtheta);
    p.y = -(position[i][1] * cos(gtheta));
    p.z = 0;

    points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);
  laser_pub.publish(laserscan);

}

int main(int argc, char** argv )
{
  float abc = 1;
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  ros::Subscriber sublaser = n.subscribe("scan", 100, laserCallback);

  ros::spin();
}
