#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>


const float gthres = 0.01;
const int ptcount = 6;
float gpre_val;
float gtheta = 0;
float gnum_readings;
const float grange = 3; //range of consideration in m
sensor_msgs::LaserScan laserscan;
ros::Publisher marker_pub;



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
    int count = 0;
    int for_i = i;

    if (abs(gpre_val - laserscan.ranges[i]) < gthres) 
    {
      gpre_val = laserscan.ranges[i];
      laserscan.ranges[i] = 0;
      count++;
    }

    if (count > 0)
    {

      for (int i = 0; i < ptcount; ++i)
      {
        if (abs(laserscan.ranges[for_i + i - 1] - laserscan.ranges[for_i + i]) < gthres)
        {
          count++;
        }

        else break;
      }

      if (count == 6)
      {
        for (int i = 0; i < count; ++i)
        {
          laserscan.ranges[for_i + i] = 0;
        }
      }

      i = i + count;
      count = 0;
    }
    

    


    
    
    gpre_val = laserscan.ranges[i];
  }
  

  visualization_msgs::Marker points;
  points.header.frame_id = "laser";
  points.header.stamp = ros::Time::now();
  points.ns = "points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  points.scale.x = 0.1;
  points.scale.y = 0.1;

  points.color.g = 1.0f;
  points.color.a = 1.0;

    
  for (int i = 0; i < (int)gnum_readings; ++i)
  {
    
    if (laserscan.ranges[i] > 0)
    {
      geometry_msgs::Point p;
      gtheta = i * laserscan.angle_increment;
      
      p.x = laserscan.ranges[i] * sin(gtheta);
      p.y = -(laserscan.ranges[i] * cos(gtheta));
      p.z = 0;

      points.points.push_back(p);
    }
  }
    
  marker_pub.publish(points);

}

int main(int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber sublaser = n.subscribe("scan", 100, laserCallback);
  

  ros::spin();
}