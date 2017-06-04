#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>

float gpre_val;
float thres = 5;
float gdiff;
float gtheta = 0;
float gnum_readings;
const float grange = 2.5; //range of consideration in m
sensor_msgs::LaserScan laserscan;

ros::Publisher marker_pub;
ros::Publisher laser_pub;
ros::Publisher table_pub;

using namespace std;

vector<int> pts;
vector<int> comb;

float position[10][2];
float table_pose[4][2];
float table_centre[2];

void go(int offset, int k)
{
  float dist1, dist2, dist3;
  float x[4], y[4], theta[3];
  float table_dim[3] = {67, 110, 129};
  int r = 0;
  

  if ( k == 0)
  {
    int check[3];
    int diag_check[2];
    


    ROS_INFO("%d %d %d", comb[0], comb[1], comb[2]);

    theta[0] = position[comb[0]][0] * laserscan.angle_increment;
    theta[1] = position[comb[1]][0] * laserscan.angle_increment;
    theta[2] = position[comb[2]][0] * laserscan.angle_increment;

    x[0] = (position[comb[0]][1] * sin(theta[0])) * 100;
    x[1] = (position[comb[1]][1] * sin(theta[1])) * 100;
    x[2] = (position[comb[2]][1] * sin(theta[2])) * 100;

    y[0] = (position[comb[0]][1] * cos(theta[0])) * (-100);
    y[1] = (position[comb[1]][1] * cos(theta[1])) * (-100);
    y[2] = (position[comb[2]][1] * cos(theta[2])) * (-100);

    dist1 = pow(pow(abs(x[1]-x[0]),2) + pow(abs(y[1]-y[0]), 2), 0.5);
    dist2 = pow(pow(abs(x[2]-x[1]),2) + pow(abs(y[2]-y[1]), 2), 0.5);
    dist3 = pow(pow(abs(x[0]-x[2]),2) + pow(abs(y[0]-y[2]), 2), 0.5);

    ROS_INFO("%f %f %f", dist1, dist2, dist3);

    if (((dist1 > table_dim[0]-thres) && (dist1 < table_dim[0]+thres)) ||  ((dist2 > table_dim[0]-thres) && (dist2 < table_dim[0]+thres))   ||  ((dist3 > table_dim[0]-thres) && (dist3 < table_dim[0]+thres)))
    {
      check[0] = 1;
      ROS_INFO("check1!");
    }
    if (((dist1 > table_dim[1]-thres) && (dist1 < table_dim[1]+thres)) ||  ((dist2 > table_dim[1]-thres) && (dist2 < table_dim[1]+thres))   ||  ((dist3 > table_dim[1]-thres) && (dist3 < table_dim[1]+thres)))
    {
      check[1] = 1;
      ROS_INFO("check2!");
    }
    if (((dist1 > table_dim[2]-thres) && (dist1 < table_dim[2]+thres)) ||  ((dist2 > table_dim[2]-thres) && (dist2 < table_dim[2]+thres))   ||  ((dist3 > table_dim[2]-thres) && (dist3 < table_dim[2]+thres)))
    {
      check[2] = 1;
      ROS_INFO("check3!");
    }
    if (check[0] == check [1] == check[2] == 1)
    {
      visualization_msgs::Marker table;
      table.header.frame_id = "laser";
      table.header.stamp = ros::Time::now();
      table.ns = "table";
      table.action = visualization_msgs::Marker::ADD;
      table.pose.orientation.w = 1.0;
      table.id = 2;
      table.type = visualization_msgs::Marker::POINTS;
      table.scale.x = 0.1;
      table.scale.y = 0.1;
      table.color.g = 0.0;
      table.color.a = 1.0;
      table.color.r = 1.0;

      geometry_msgs::Point p;

      ROS_INFO("Table found");

      p.x = p.y = p.z = 0.0;
      table.points.push_back(p); //robot location

      if ((dist1 > table_dim[2]-thres) && (dist1 < table_dim[2]+thres))
      {

        table_centre[0] = (x[0]+x[1])/2;
        table_centre[1] = (y[0]+y[1])/2;
        p.x = (x[0]+x[1])/200;
        p.y = (y[0]+y[1])/200;
        p.z = 0;

        x[3] = (2 * table_centre[0]) - x[2];
        y[3] = (2 * table_centre[1]) - y[2];

      }
      else if ((dist2 > table_dim[2]-thres) && (dist2 < table_dim[2]+thres))
      {
        table_centre[0] = (x[1]+x[2])/2;
        table_centre[1] = (y[1]+y[2])/2;
        p.x = (x[1]+x[2])/200;
        p.y = (y[1]+y[2])/200;
        p.z = 0;

        x[3] = (2 * table_centre[0]) - x[0];
        y[3] = (2 * table_centre[1]) - y[0];

      }
      else
      {

        table_centre[0] = (x[2]+x[0])/2;
        table_centre[1] = (y[2]+y[0])/2;
        p.x = (x[2]+x[0])/200;
        p.y = (y[2]+y[0])/200;
        p.z = 0;

        x[3] = (2 * table_centre[0]) - x[1];
        y[3] = (2 * table_centre[1]) - y[1];

      }
      table.points.push_back(p);

      

      if ((dist1 > table_dim[0]-10) && (dist1 < table_dim[0]+10))
      {
        p.x = x[0] / 100;
        p.y = y[0] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[1] / 100;
        p.y = y[1] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[2] / 100;
        p.y = y[2] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[3] / 100;
        p.y = y[3] / 100;
        p.z = 0.0;
        table.points.push_back(p);
      }
      else if ((dist2 > table_dim[0]-10) && (dist2 < table_dim[0]+10))
      {
        p.x = x[1] / 100;
        p.y = y[1] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[2] / 100;
        p.y = y[2] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[3] / 100;
        p.y = y[3] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[0] / 100;
        p.y = y[0] / 100;
        p.z = 0.0;
        table.points.push_back(p);
      }
      else
      {
        p.x = x[2] / 100;
        p.y = y[2] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[0] / 100;
        p.y = y[0] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[1] / 100;
        p.y = y[1] / 100;
        p.z = 0.0;
        table.points.push_back(p);

        p.x = x[3] / 100;
        p.y = y[3] / 100;
        p.z = 0.0;
        table.points.push_back(p);
      }

      table_pub.publish(table);
      r = -1;
    }

    return; 
  }
  for (int i = offset; i <= pts.size() - k; ++i)
  {
    comb.push_back(pts[i]);
    if (r < 0) goto endpub;
    go(i+1, k-1);
    endpub:
    comb.pop_back();
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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

  points.scale.x = 0.1;
  points.scale.y = 0.1;

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
  //marker_pub.publish(line_strip);
  laser_pub.publish(laserscan);


}

int main(int argc, char** argv )
{
  float abc = 1;
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  table_pub = n.advertise<visualization_msgs::Marker>("table_marker", 10);
  ros::Subscriber sublaser = n.subscribe("base_scan", 100, laserCallback);

  ros::spin();
}
