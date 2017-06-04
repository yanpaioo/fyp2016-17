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

ros::Publisher Path_pub;
ros::Publisher Cmd_vel_pub;

float F_move_cmd[2][2];
//float F_table_pose[6][2];
double D_yaw, D_roll, D_pitch;
float F_cur_x, F_cur_y;

std::vector<geometry_msgs::Point> GenerateAndDrawPath(std::vector<geometry_msgs::Point> vpt)
{

  float f_m_line, f_c_line;
  float f_d = 1.0;
  std::vector<geometry_msgs::Point> f_move;

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
  f_c_line = vpt[1].y - f_m_line *  vpt[1].x;

  p.x = 0.0;
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
  points.points.push_back(p);
  line_strip.points.push_back(p);
  f_move.push_back(p);

  Path_pub.publish(points);
  Path_pub.publish(line_strip);

  return f_move;

}


geometry_msgs::Point _prev_pose;
enum GoalState  { NEW_GOAL, TURN_TO_GOAL, MOVE_TO_GOAL, GOAL_REACHED, NO_GOAL};
GoalState _goalState = NEW_GOAL;
float f_goalAngle = 0;
int _dGoalID = 0;

bool PublishCmdVel(const std::vector<geometry_msgs::Point> goalpts)
{
  //float f_goalOrientation = f_pre_yaw + D_yaw;
   while(_dGoalID < goalpts.size())
  {
        float f_angle_to_turn = goalpts[_dGoalID].x;
        float f_dist_to_move = goalpts[_dGoalID].y;

        if(_goalState == NEW_GOAL) //update prev pose;
        {

          _prev_pose.x =  F_cur_x;
          _prev_pose.y =  F_cur_y;
          _prev_pose.z = D_yaw;
          f_goalAngle = D_yaw + f_angle_to_turn;

          _goalState = TURN_TO_GOAL;
        }

      else if(_goalState == TURN_TO_GOAL)
      {

        ROS_INFO("IN PROCESS: D_yaw=%f  f_goalAngle=%f   f_angle_to_turn=%f", D_yaw, f_goalAngle, f_angle_to_turn);
        geometry_msgs::Twist cmd_vel;
        if (f_angle_to_turn > 0)
        {
          cmd_vel.angular.z = -0.26;
          Cmd_vel_pub.publish(cmd_vel);
          if (D_yaw > f_goalAngle)
          {
            cmd_vel.angular.z = 0;
            Cmd_vel_pub.publish(cmd_vel);
            _goalState = MOVE_TO_GOAL;
          }
        }
        else
        {
          cmd_vel.angular.z = 0.26;
          Cmd_vel_pub.publish(cmd_vel);
          if (D_yaw < f_goalAngle)
          {
            cmd_vel.angular.z = 0;
            Cmd_vel_pub.publish(cmd_vel);
            _goalState = MOVE_TO_GOAL;
          }
        }

      }
      else if (_goalState == MOVE_TO_GOAL)
      {
        float dist_travelled;
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.1; 
        dist_travelled = sqrt(pow(_prev_pose.x - F_cur_x, 2)+pow(_prev_pose.y - F_cur_y, 2));
        if (dist_travelled > f_dist_to_move)
        {
          cmd_vel.linear.x = 0.0;
          _goalState = GOAL_REACHED;
        }
        Cmd_vel_pub.publish(cmd_vel);

      }

      else if(_goalState == GOAL_REACHED)
      {
        _dGoalID++;
      }

  }




  
}

void GenerateAndPubCmdVel(const std::vector<geometry_msgs::Point> goalpts)
{
  // F_move_cmd[0][0] = -(180/3.1416 * atan((f_move[1][1]) / f_move[1][0]));
  // F_move_cmd[0][1] = sqrt(pow(f_move[1][1], 2) + pow(f_move[1][0], 2));
  //F_move_cmd[1][1] = pow(pow(f_move[2][0]-f_move[1][0], 2) + pow(f_move[2][1]-f_move[1][1], 2), 0.5);

  std::vector<geometry_msgs::Point> goals;

  float goalOrientation1 = -(180/3.1416 * atan((goalpts[1].y) / goalpts[1].x));
  float goalDistance1  = sqrt(pow(goalpts[1].y, 2) + pow(goalpts[1].x,2));
  float goalDistance2 = sqrt(pow(goalpts[2].x-goalpts[1].x, 2) + pow(goalpts[2].y-goalpts[1].y, 2));

  float a, b, c, A, B, C;
  //a = F_move_cmd[0][1];
  a = goalpts[0].y;
  b = goalpts[1].y;
  //b = F_move_cmd[1][1];
  //c = pow(pow(f_move[2][0]-f_move[0][0], 2) + pow(f_move[2][1]-f_move[0][1], 2), 0.5);
  c = sqrt(pow(goalpts[2].x-goalpts[0].x, 2) + pow(goalpts[2].y-goalpts[0].y, 2));
  A = acos((b*b + c*c - a*a)/(2*b*c));
  B = acos((a*a + c*c - b*b)/(2*a*c));
  C = 180 - (180/3.1416*(A+B));

  //F_move_cmd[1][0] = 180 - C;
  float goalOrientation2 = 180 - C;

  ROS_INFO("Angle to turn 1 = %f", goalOrientation1);


  geometry_msgs::Point goal;
  goal.x = goalOrientation1;
  goal.y = goalDistance1;
  goals.push_back(goal);

  goal.x = goalOrientation2;
  goal.y = goalDistance2;
  goals.push_back(goal);


  //if(_goalState == NO_GOAL)
  
  PublishCmdVel(goals);

 


}

void tableCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  // OBTAIN THE TABLE POINTS AND PASS IT TO GENERATE PATH:
  std::vector<geometry_msgs::Point> vpt;
    for (int i = 0; i < 6; i++)
  {
    geometry_msgs::Point pt;
    pt.x = msg->points[i].x;
    pt.y = msg->points[i].y;
    vpt.push_back(pt);
    //ROS_INFO("x= %f   y=%f", f_x[i], f_y[i]);
  }

  std::vector<geometry_msgs::Point> goalpts = GenerateAndDrawPath(vpt);


  ROS_INFO("Table callback") ;
 

  // F_move_cmd[0][0] = -(180/3.1416 * atan((f_move[1][1]) / f_move[1][0]));
  // F_move_cmd[0][1] = sqrt(pow(f_move[1][1], 2) + pow(f_move[1][0], 2));
  // F_move_cmd[1][1] = pow(pow(f_move[2][0]-f_move[1][0], 2) + pow(f_move[2][1]-f_move[1][1], 2), 0.5);

  // float a, b, c, A, B, C;
  // a = F_move_cmd[0][1];
  // b = F_move_cmd[1][1];
  // c = pow(pow(f_move[2][0]-f_move[0][0], 2) + pow(f_move[2][1]-f_move[0][1], 2), 0.5);
  // A = acos((b*b + c*c - a*a)/(2*b*c));
  // B = acos((a*a + c*c - b*b)/(2*a*c));
  // C = 180 - (180/3.1416*(A+B));

  // F_move_cmd[1][0] = 180 - C;

}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getEulerYPR(D_yaw, D_pitch, D_roll);

  D_yaw = D_yaw * 180 / 3.14159265359;
  F_cur_x = msg->pose.pose.position.x;
  F_cur_y = msg->pose.pose.position.y;
  ROS_INFO("Yaw= %f   x= %f   y= %f", D_yaw, F_cur_x, F_cur_y);
}


int main(int argc, char** argv )
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n;
  Path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 10);
  Cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sublaser = n.subscribe("table_marker", 10, tableCallback);
  ros::Subscriber subodom = n.subscribe("odom", 10, odomCallback);
  //ros::Subscriber subimu = n.subscribe("imu_rpy", 10, imuCallback);

  geometry_msgs::Twist move;
  ros::Rate r(5);

  float f_cmd_vel[2][2];
  float f_goal;
  float f_pre_pose[2];

  while(ros::ok())
  {
      if(there is a table)
      {
        GenerateAndPubCmdVel(goalpts); 
      }

      
    r.sleep();
    ros::spinOnce();

  }

  ros::spin();
  return 0;
}
