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
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <vector>

#define ROBOT 1

using namespace std;
ros::Publisher Path_pub;
ros::Publisher Cmd_vel_pub;

float F_move_cmd[2][2];
double D_yaw, D_roll, D_pitch;
float F_cur_x, F_cur_y;

enum GoalState  { NO_GOAL, IN_PROGRESS};
GoalState _goalState = NO_GOAL;

vector<geometry_msgs::Point> GenerateAndDrawPath(std::vector<geometry_msgs::Point> vpt)
{

  float f_m_line, f_c_line;
  float f_d = 0.7;
  vector<geometry_msgs::Point> f_move;

  visualization_msgs::Marker points, line_strip;

#ifdef ROBOT  
  points.header.frame_id = line_strip.header.frame_id = "base_laser_link";
#else
  points.header.frame_id = line_strip.header.frame_id = "laser";
#endif

  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "pathline";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  line_strip.scale.x = 0.05;
  points.color.g = 1.0;
  points.color.r = 1.0;
  points.color.b = 0.0;
  points.color.a = 1.0;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.27;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p;

  f_m_line = (vpt[2].y - vpt[3].y) / (vpt[2].x - vpt[3].x);
  f_c_line = vpt[1].y - f_m_line *  (vpt[1].x);

  p.x = 0.0 - 0.2;
  p.y = 0.0;
  points.points.push_back(p);
  line_strip.points.push_back(p);
  f_move.push_back(p);

  //p.x = (-sqrt(-pow(f_c_line,2) - 2*f_c_line*f_m_line*f_x[1] + 2*f_c_line*f_y[1] + pow(f_d,2)*pow(f_m_line,2) + pow(f_d,2) - pow(f_m_line,2)*pow(f_x[1],2) + 2*f_m_line*f_x[1]*f_y[1] - pow(f_y[1],2)) - f_c_line*f_m_line + f_m_line*f_y[1] + f_x[1]) / (pow(f_m_line,2) + 1); 
  p.x = (-sqrt(-pow(f_c_line,2) - 2*f_c_line*f_m_line*vpt[1].x + 2*f_c_line*vpt[1].y + pow(f_d,2)*pow(f_m_line,2) + pow(f_d,2) - pow(f_m_line,2)*pow(vpt[1].x,2) + 2*f_m_line*vpt[1].x*vpt[1].y - pow(vpt[1].y,2)) - f_c_line*f_m_line + f_m_line*vpt[1].y + vpt[1].x) / (pow(f_m_line,2) + 1); 
  p.y = f_m_line*(p.x) + f_c_line;
  points.points.push_back(p);
  line_strip.points.push_back(p);
  f_move.push_back(p);


  p.x =  vpt[1].x;
  p.y =  vpt[1].y;
  ROS_INFO("Centre x=%f y = %f", p.x, p.y);
  points.points.push_back(p);
  line_strip.points.push_back(p);
  f_move.push_back(p);

  Path_pub.publish(points);
  Path_pub.publish(line_strip);

  return f_move;

}

geometry_msgs::Point _prev_pose;

float f_goalAngle = 0;

void GenerateAndPubCmdVel(const vector<geometry_msgs::Point> goalpts)
{

  float goalOrientation1 = -(180/3.1416 * atan2((goalpts[1].y - goalpts[0].y) , goalpts[1].x - goalpts[0].x));
  float goalDistance1  = sqrt(pow(goalpts[1].y, 2) + pow(goalpts[1].x, 2));
  float goalOrientation2 = -(180/3.1416 * atan2((goalpts[2].y - goalpts[1].y) , goalpts[2].x - goalpts[1].x)) - goalOrientation1;
  float goalDistance2 = sqrt(pow(goalpts[2].x-goalpts[1].x, 2) + pow(goalpts[2].y-goalpts[1].y, 2));  

  // float goalOrientation1 = -(180/3.1416 * atan2((goalpts[1].y - goalpts[0].y) , goalpts[1].x - goalpts[0].x));
  // float goalDistance1  = sqrt(pow(goalpts[1].y, 2) + pow(goalpts[1].x, 2));
  // float goalOrientation2 = -(180/3.1416 * atan2((goalpts[2].y - goalpts[1].y) , goalpts[2].x - goalpts[1].x)) - goalOrientation1;
  // float goalDistance2 = sqrt(pow(goalpts[2].x-goalpts[1].x, 2) + pow(goalpts[2].y-goalpts[1].y, 2));  


  F_move_cmd[0][0] = goalOrientation1;
  F_move_cmd[0][1] = goalDistance1;
  F_move_cmd[1][0] = goalOrientation2;
  F_move_cmd[1][1] = goalDistance2;

  _goalState = IN_PROGRESS;


}

// void tableCallback(const visualization_msgs::Marker::ConstPtr& msg)
void tableCallback(const visualization_msgs::Marker msg)
{

  vector<geometry_msgs::Point> vpt;
  for (int i = 0; i < 6; i++)
  {
    geometry_msgs::Point pt;
    pt.x = msg.points[i].x;
    pt.y = msg.points[i].y;
    pt.z = msg.points[i].z;
    vpt.push_back(pt);
  }

  vector<geometry_msgs::Point> goalpts = GenerateAndDrawPath(vpt);

  GenerateAndPubCmdVel(goalpts); 


}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getEulerYPR(D_yaw, D_pitch, D_roll);

  D_yaw = (D_yaw * 180 / 3.14159265359)+360;
  F_cur_x = msg->pose.pose.position.x;
  F_cur_y = msg->pose.pose.position.y;
}


int main(int argc, char** argv )
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n;
  Path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 10);
  Cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sublaser = n.subscribe("table_marker", 2, tableCallback);
  ros::Subscriber subodom = n.subscribe("odom", 2, odomCallback);

  geometry_msgs::Twist move;
  


  float f_move_cmd[2][2];
  float f_goal;
  float f_pre_goal;
  float f_pre_pose[2];
  vector<geometry_msgs::Point> v_pre_pose;

  ros::Rate r(100);
  while(ros::ok())
  {
    if (_goalState == NO_GOAL)
    {
      ROS_INFO("NO_GOAL!");
    } 

    else
    {
      geometry_msgs::Twist cmd_vel;

      ROS_INFO("IN_PROGRESS!");

      f_move_cmd[0][0] = F_move_cmd[0][0];
      f_move_cmd[0][1] = F_move_cmd[0][1];
      f_move_cmd[1][0] = F_move_cmd[1][0];
      f_move_cmd[1][1] = F_move_cmd[1][1];

      ROS_INFO("Orientation1 = %f", f_move_cmd[0][0]);
      ROS_INFO("Movement1 = %f", f_move_cmd[0][1]);
      ROS_INFO("Orientation2 = %f", f_move_cmd[1][0]);
      ROS_INFO("Movement2 = %f", f_move_cmd[1][1]);

      getchar();


      f_goal = D_yaw - f_move_cmd[0][0];
      f_pre_goal = f_goal;

      ROS_INFO("D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);

      if (f_move_cmd[0][0] > 0)
      {
        cmd_vel.angular.z = -0.3;
        Cmd_vel_pub.publish(cmd_vel);

        while((f_goal < D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 1 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = -0.3;
          Cmd_vel_pub.publish(cmd_vel);
          r.sleep();
          ros::spinOnce();
        }
        cmd_vel.angular.z = 0;
        Cmd_vel_pub.publish(cmd_vel);
      }

      else
      {
        cmd_vel.angular.z = 0.3;
        Cmd_vel_pub.publish(cmd_vel);

        while((f_goal > D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 2 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = 0.3;
          Cmd_vel_pub.publish(cmd_vel);
          r.sleep();
          ros::spinOnce();
        }
        cmd_vel.angular.z = 0;
        Cmd_vel_pub.publish(cmd_vel);
      }

      f_goal = f_move_cmd[0][1];

      float f_dist_travel = 0;
      geometry_msgs::Point p;
      p.x = F_cur_x;
      p.y = F_cur_y;
      p.z = D_yaw;
      v_pre_pose.push_back(p);

      while((f_dist_travel < f_move_cmd[0][1]) && (ros::ok()))
      {
        ROS_INFO("Linear moving!");
        f_dist_travel = sqrt(pow(v_pre_pose[0].x - F_cur_x, 2) + pow(v_pre_pose[0].y - F_cur_y, 2));
        ROS_INFO("Dist_travel= %f, dist-to-travel= %f", f_dist_travel, f_move_cmd[0][1]);
        cmd_vel.linear.x = 0.3;
        Cmd_vel_pub.publish(cmd_vel);
        r.sleep();
        ros::spinOnce();
      }

      cmd_vel.linear.x = 0;
      Cmd_vel_pub.publish(cmd_vel);

     
      f_goal = f_pre_goal - f_move_cmd[1][0];

      ROS_INFO("Angle to turn2 = %f", f_move_cmd[1][0]);

      ROS_INFO("D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);

      if (f_move_cmd[1][0] > 0)
      {
        cmd_vel.angular.z = -0.3;

        while((f_goal < D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 1 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = -0.3;
          Cmd_vel_pub.publish(cmd_vel);
          //ROS_INFO("Turning!");
          r.sleep();
          ros::spinOnce();
        }
        cmd_vel.angular.z = 0;
        Cmd_vel_pub.publish(cmd_vel);
      }

      else
      {
        cmd_vel.angular.z = 0.3;

        while((f_goal > D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 2 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = 0.3;
          Cmd_vel_pub.publish(cmd_vel);
          r.sleep();
          ros::spinOnce();
        }
        cmd_vel.angular.z = 0;
        Cmd_vel_pub.publish(cmd_vel);
      }



      p.x = F_cur_x;
      p.y = F_cur_y;
      p.z = D_yaw;
      v_pre_pose.push_back(p);
      f_dist_travel = 0;

      while((f_dist_travel < f_move_cmd[1][1]) && (ros::ok()))
      {
        ROS_INFO("Linear moving!");
        f_dist_travel = sqrt(pow(v_pre_pose[1].x - F_cur_x, 2) + pow(v_pre_pose[1].y - F_cur_y, 2));
        ROS_INFO("Dist_travel= %f, dist-to-travel= %f", f_dist_travel, f_move_cmd[1][1]);
        cmd_vel.linear.x = 0.3;
        Cmd_vel_pub.publish(cmd_vel);
        r.sleep();
        ros::spinOnce();
      }

      cmd_vel.linear.x = 0;
      Cmd_vel_pub.publish(cmd_vel);


       _goalState = NO_GOAL;
       ROS_INFO("Process complete!");
       goto next;


    }
    r.sleep();
    ros::spinOnce();
  }
  next:
  ros::spin();
  return 0;
}
