#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

ros::Publisher path_pub;

void laserCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  float d = 1.0;
  float x[3], y[3];
  float m, c; //slope, intercept
  geometry_msgs::Point p;

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = "laser";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "pathline";
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

  for (int i = 0; i < 4; ++i)
  {
    x[i] = msg->points[i+1].x;
    y[i] = msg->points[i+1].y;
  }

  m = (y[1] - y[2]) / (x[1] - x[2]);
  c = y[0] - (m*x[0]);

  ROS_INFO("Eqn - y = %fx + %f", m, c);
  ROS_INFO("slope - %f", m);

  p.x = x[0];
  p.y = y[0];
  points.points.push_back(p);
  line_strip.points.push_back(p);

  p.x = (-sqrt(-pow(c,2) - 2*c*m*x[0] + 2*c*y[0] + pow(d,2)*pow(m,2) + pow(d,2) - pow(m,2)*pow(x[0],2) + 2*m*x[0]*y[0] - pow(y[0],2)) - c*m + m*y[0] + x[0]) / (pow(m,2) + 1); 
  p.y = m*(p.x) + c;

  ROS_INFO("x- %f  y- %f", p.x, p.y);
  points.points.push_back(p);
  line_strip.points.push_back(p);

  p.x = 0.0;
  p.y = 0.0;
  points.points.push_back(p);
  line_strip.points.push_back(p);

  path_pub.publish(line_strip);
  path_pub.publish(points);


}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

}

int main(int argc, char** argv )
{
  float abc = 1;
  ros::init(argc, argv, "path_plan");
  ros::NodeHandle n;
  path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 10);
  ros::Subscriber sublaser = n.subscribe("table_marker", 10, laserCallback);
  ros::Subscriber subodom = n.subscribe("odom", 10, odomCallback);

  ros::Rate r(5);

  while(ros::ok)
  {
    ROS_INFO("while loop");
    r.sleep();
  }

  ros::spin();
}
