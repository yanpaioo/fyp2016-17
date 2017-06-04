#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <string.h>
#include <stdlib.h> 
#include <stdio.h> 

ros::Publisher path_pub;
ros::Publisher cmd_vel;
float move_cmd[2][2];
float fyaw;
float xd[2];

void laserCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  float d = 1.0;
  float x[3], y[3];
  float m, c; //slope, intercept
  geometry_msgs::Point p;
  
  float cmd[3][2];
  float da, db, dc, A, B, C;

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

  p.x = x[0];
  p.y = y[0];
  cmd[2][0] = x[0];
  cmd[2][1] = y[0];
  points.points.push_back(p);
  line_strip.points.push_back(p);

  p.x = (-sqrt(-pow(c,2) - 2*c*m*x[0] + 2*c*y[0] + pow(d,2)*pow(m,2) + pow(d,2) - pow(m,2)*pow(x[0],2) + 2*m*x[0]*y[0] - pow(y[0],2)) - c*m + m*y[0] + x[0]) / (pow(m,2) + 1); 
  p.y = m*(p.x) + c;
  cmd[1][0] = p.x;
  cmd[1][1] = p.y;

  points.points.push_back(p);
  line_strip.points.push_back(p);

  p.x = 0.0;
  p.y = 0.0;
  cmd[0][0] = p.x;
  cmd[0][1] = p.y;
  points.points.push_back(p);
  line_strip.points.push_back(p);

  path_pub.publish(line_strip);
  path_pub.publish(points);

  move_cmd[0][0] =  -(180/ 3.1416 * atan((cmd[1][1]) / cmd[1][0]));
  move_cmd[0][1] = sqrt(pow(cmd[1][1],2) + pow(cmd[1][0], 2));
  move_cmd[1][1] = pow(pow(cmd[2][0]-cmd[1][0],2) + pow(cmd[2][1]-cmd[1][1], 2), 0.5);

  da = move_cmd[0][1];
  db = move_cmd[1][1];
  dc = pow(pow(cmd[2][0]-cmd[0][0], 2) + pow(cmd[2][1]-cmd[0][1], 2), 0.5);
  A = acos((db*db + dc*dc - da*da)/(2*db*dc));
  B = acos((da*da + dc*dc - db*db)/(2*da*dc));
  C = 180 - (180/3.1416*(A+B));

  move_cmd[1][0] = 180 - C;

  ROS_INFO("a - %f", da);
  ROS_INFO("move_cmd rotation1 - %f", move_cmd[0][0]);
  ROS_INFO("move_cmd linear1 - %f", move_cmd[0][1]);
  ROS_INFO("move_cmd rotation2 - %f", move_cmd[1][0]);
  ROS_INFO("move_cmd linear2 - %f", move_cmd[1][1]);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  float t3 = +2.0f * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
  float t4 = +1.0f - 2.0f * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z); 
  float yaw = std::atan2(t3, t4);
  xd[0] = msg->pose.pose.position.x;
  xd[1] = msg->pose.pose.position.y;

}

void imuCallback(const std_msgs::String msg)
{
  char c[100];
  char yaw[5];
  
  strcpy(c, msg.data.c_str());
  int countcom=0;

  for (int i = 0; i < strlen(c)+1; ++i)
  {
    if (c[i] == ',') countcom++;
    if (countcom == 2)
    {
      for (int y = 0; y < 6; ++y)
      {
        if (c[countcom+y+i] == ' ') goto next;
        yaw[y] = c[countcom+y+i];
      }
      goto next;
    }
  }
  next:
  fyaw = strtof(yaw, NULL);
  ROS_INFO("yaw %.2f", fyaw);
    
}

int main(int argc, char** argv )
{
  float abc = 1;
  float dist_check, pre_pose[2];
  ros::init(argc, argv, "path_plan");
  ros::NodeHandle n;
  path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 10);
  cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sublaser = n.subscribe("table_marker", 10, laserCallback);
  ros::Subscriber subodom = n.subscribe("odom", 10, odomCallback);
  ros::Subscriber subimu = n.subscribe("imu_rpy", 10, imuCallback);
  float rmove_cmd[2][2];
  float goal[2][2];

  geometry_msgs::Twist move;

  while(ros::ok())
  {
    ros::Rate r(5);
    
    again:
    ROS_INFO("x odom - %f", xd[0]);
    if (move_cmd[0][0] == 0) 
      {
        ros::spinOnce();
        r.sleep();
        goto endofop;
      }

    rmove_cmd[0][0] = move_cmd[0][0];
    rmove_cmd[0][1] = move_cmd[0][1];
    rmove_cmd[1][0] = move_cmd[1][0];
    rmove_cmd[1][1] = move_cmd[1][1];

    goal[0][0] = fyaw + rmove_cmd[0][0];
    //goal[1][0] = fyaw + rmove_cmd[1][0];
    //ROS_INFO("goal yaw %f", goal[0][0]);
    

    if (rmove_cmd[0][0] > 0)
    {
      while (goal[0][0] > fyaw)
      {
        move.angular.z = 0.26;
        cmd_vel.publish(move);
        r.sleep();
        ros::spinOnce();
      }

    } 
    else
    {
      while (goal[0][0] < fyaw)
      {
        move.angular.z = -0.26;
        cmd_vel.publish(move);
        r.sleep();
        ros::spinOnce();
      }    
    }
    move.angular.z = 0.0;
    cmd_vel.publish(move);
    ROS_INFO("Rotation done!");

    //goal[0][1] = xd + rmove_cmd[0][1];
    pre_pose[0] = xd[0];
    pre_pose[1] = xd[1];
    while(1)
    {
      ROS_INFO("xd - %f", xd[0]);
      dist_check = pow(pow(pre_pose[0]-xd[0],2) + pow(pre_pose[1]-xd[1], 2), 0.5);
      if (dist_check < rmove_cmd[0][1])
      {
        ROS_INFO("Linear done!");
        move.linear.x = 0.0;
        cmd_vel.publish(move);
        goto rotate2;
      }
      move.linear.x = 0.05;
      cmd_vel.publish(move);
      r.sleep();
      ros::spinOnce();
    }
    rotate2:
    

    goal[1][0] = fyaw + rmove_cmd[1][0];

     if (rmove_cmd[1][0] > 0)
    {
      while (goal[1][0] > fyaw)
      {
        move.angular.z = 0.26;
        cmd_vel.publish(move);
        r.sleep();
        ros::spinOnce();
      }

    } 
    else
    {
      while (goal[1][0] < fyaw)
      {
        move.angular.z = -0.26;
        cmd_vel.publish(move);
        r.sleep();
        ros::spinOnce();
      }    
    }

    pre_pose[0] = xd[0];
    pre_pose[1] = xd[1];
    while(1)
    {
      ROS_INFO("xd - %f", xd[0]);
      dist_check = pow(pow(pre_pose[0]-xd[0],2) + pow(pre_pose[1]-xd[1], 2), 0.5);
      if (dist_check < rmove_cmd[1][1])
      {
        ROS_INFO("Linear done!");
        move.linear.x = 0.0;
        cmd_vel.publish(move);
        goto endofop;
      }
      move.linear.x = 0.05;
      cmd_vel.publish(move);
      r.sleep();
      ros::spinOnce();
    }
    endofop:

    ros::spinOnce();
  }

  ros::spin();
  return 0;
  
}
