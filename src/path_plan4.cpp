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

using namespace std;
ros::Publisher Path_pub;
ros::Publisher Cmd_vel_pub;

//vector<geometry_msgs::Point> F_move_cmd;
float F_move_cmd[2][2];
//float F_table_pose[6][2];
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

float f_goalAngle = 0;

// bool movetoGoal(float f_angle_to_turn, float f_dist_to_move)
// {
//   //float f_goalOrientation = f_pre_yaw + D_yaw;

//   if(_goalState == NEW_GOAL) //update prev pose;
//   {

//     _prev_pose.x =  F_cur_x;
//     _prev_pose.y =  F_cur_y;
//     _prev_pose.z = D_yaw;
//     f_goalAngle = D_yaw + f_angle_to_turn;

//     _goalState = TURN_TO_GOAL;
//   }

//   else if(_goalState == TURN_TO_GOAL)
//   {

//     ROS_INFO("IN PROCESS: D_yaw=%f  f_goalAngle=%f   f_angle_to_turn=%f", D_yaw, f_goalAngle, f_angle_to_turn);
//     geometry_msgs::Twist cmd_vel;
//     if (f_angle_to_turn > 0)
//     {
//       cmd_vel.angular.z = -0.26;
//       Cmd_vel_pub.publish(cmd_vel);
//       if (D_yaw > f_goalAngle)
//       {
//         cmd_vel.angular.z = 0;
//         Cmd_vel_pub.publish(cmd_vel);
//         _goalState = MOVE_TO_GOAL;
//       }
//     }
//     else
//     {
//       cmd_vel.angular.z = 0.26;
//       Cmd_vel_pub.publish(cmd_vel);
//       if (D_yaw < f_goalAngle)
//       {
//         cmd_vel.angular.z = 0;
//         Cmd_vel_pub.publish(cmd_vel);
//         _goalState = MOVE_TO_GOAL;
//       }
//     }

//   }
//   else if (_goalState == MOVE_TO_GOAL)
//   {
//     float dist_travelled;
//     geometry_msgs::Twist cmd_vel;
//     cmd_vel.linear.x = 0.1; 
//     dist_travelled = sqrt(pow(_prev_pose.x - F_cur_x, 2)+pow(_prev_pose.y - F_cur_y, 2));
//     if (dist_travelled > f_dist_to_move)
//     {
//       cmd_vel.linear.x = 0.0; //goal
//     }
//     Cmd_vel_pub.publish(cmd_vel);

//   }

//   //float f_pre_yaw = D_yaw;
//   //geometry_msgs::Point pre_pose;
//   //pre_pose.x = F_cur_x;
//   //pre_pose.y = F_cur_y;
  
//   float f_goalPose;

// }

void GenerateAndPubCmdVel(const vector<geometry_msgs::Point> goalpts)
{
  float goalOrientation1 = -(180/3.1416 * atan((goalpts[1].y) / goalpts[1].x));
  float goalDistance1  = sqrt(pow(goalpts[1].y, 2) + pow(goalpts[1].x,2));
  float goalDistance2 = sqrt(pow(goalpts[2].x-goalpts[1].x, 2) + pow(goalpts[2].y-goalpts[1].y, 2));

  float a, b, c, dA, dB, dC;

  //a = goalpts[0].y;
  //b = goalpts[1].y;
  //c = sqrt(pow(goalpts[2].x-goalpts[0].x, 2) + pow(goalpts[2].y-goalpts[0].y, 2));
  a = goalDistance1;
  b = goalDistance2;
  c = sqrt(pow(goalpts[2].x-goalpts[0].x, 2) + pow(goalpts[2].y-goalpts[0].y, 2));
  dA = acos((b*b + c*c - a*a)/(2*b*c));
  dB = acos((a*a + c*c - b*b)/(2*a*c));
  dC = 180 - (180/3.1416*(dA+dB));

  float goalOrientation2 = -(180 - dC);
  // ROS_INFO("a= %f   b=%f   c=%f", a, b, c);
  // ROS_INFO("A= %f   B=%f   C=%f", dA, dB, dC);

  F_move_cmd[0][0] = goalOrientation1;
  F_move_cmd[0][1] = goalDistance1;
  F_move_cmd[1][0] = goalOrientation2;
  F_move_cmd[1][1] = goalDistance2;

  ROS_INFO("goalOrientation2=%f", goalOrientation2);



  _goalState = IN_PROGRESS;


  //movetoGoal(goalOrientation1, goalDistance1);

}

void tableCallback(const visualization_msgs::Marker::ConstPtr& msg)
{

  // OBTAIN THE TABLE POINTS AND PASS IT TO GENERATE PATH:
  vector<geometry_msgs::Point> vpt;
  for (int i = 0; i < 6; i++)
  {
    geometry_msgs::Point pt;
    pt.x = msg->points[i].x;
    pt.y = msg->points[i].y;
    vpt.push_back(pt);
    //ROS_INFO("x= %f   y=%f", f_x[i], f_y[i]);
  }

  vector<geometry_msgs::Point> goalpts = GenerateAndDrawPath(vpt);

  GenerateAndPubCmdVel(goalpts); 
  //ROS_INFO("Table callback") ;
 

}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getEulerYPR(D_yaw, D_pitch, D_roll);

  D_yaw = D_yaw * 180 / 3.14159265359;
  F_cur_x = msg->pose.pose.position.x;
  F_cur_y = msg->pose.pose.position.y;
  //ROS_INFO("Yaw= %f   x= %f   y= %f", D_yaw, F_cur_x, F_cur_y);
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
  

  //vector<geometry_msgs::Point> v_cmd_vel;
  float f_move_cmd[2][2];
  float f_goal;
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
      getchar();

      f_move_cmd[0][0] = F_move_cmd[0][0];
      f_move_cmd[0][1] = F_move_cmd[0][1];
      f_move_cmd[1][0] = F_move_cmd[1][0];
      f_move_cmd[1][1] = F_move_cmd[1][1];

      ROS_INFO("Orientation1 = %f", f_move_cmd[0][0]);
      ROS_INFO("Movement1 = %f", f_move_cmd[0][1]);
      ROS_INFO("Orientation2 = %f", f_move_cmd[1][0]);
      ROS_INFO("Movement2 = %f", f_move_cmd[1][1]);


      // geometry_msgs::Point p;
      // p.x = F_cur_x;
      // p.y = F_cur_y;
      // p.z = D_yaw;
      // v_pre_pose.push_back(p);

      f_goal = D_yaw - f_move_cmd[0][0];

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
        Cmd_vel_pub.publish(cmd_vel);

        while((f_goal > D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 2 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = 0.3;
          Cmd_vel_pub.publish(cmd_vel);
          //ROS_INFO("Turning!");
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

     
      f_goal = D_yaw - f_move_cmd[1][0];

      ROS_INFO("Angle to turn2 = %f", f_move_cmd[1][0]);

      ROS_INFO("D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);

      if (f_move_cmd[1][0] > 0)
      {
        cmd_vel.angular.z = -0.3;
        //Cmd_vel_pub.publish(cmd_vel);

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
        //Cmd_vel_pub.publish(cmd_vel);

        while((f_goal > D_yaw) && (ros::ok()))
        {
          ROS_INFO("Turn 2 D_yaw= %f    Goal_yaw= %f", D_yaw, f_goal);
          cmd_vel.angular.z = 0.3;
          Cmd_vel_pub.publish(cmd_vel);
          //ROS_INFO("Turning!");
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
    //F_move_cmd[0][0] = F_move_cmd[0][1] = F_move_cmd[1][0] = F_move_cmd[1][1] = 0;

  }
  next:
  ros::spin();
  return 0;
}
