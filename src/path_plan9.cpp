#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <vector>

#define ROBOT 1

using namespace std;

ros::Publisher Path_pub;
ros::Publisher Cmd_vel_pub;

// general variable
float Pi = 3.1416;

#ifdef ROBOT
	float Stop_dist = 0.3;	// stop dist between laser and table centre
	float Complete_dist = 0.3;	// stop dist bet laser and back legs
#else
	float Stop_dist = 0.55;
	float Complete_dist = 0.6;	// stop dist bet laser and back legs
#endif
int Stop_iteration = 50;	// stop after this iteration after step 5
int Stop_iteration1 = 200;
float Pre_x_pose;
float Pre_x_maintain;

// state
uint State = 0;

// odom 
float F_cur_x, F_cur_y;
double D_yaw, D_pitch, D_roll;

// table position
geometry_msgs::Point table_pose[7];
geometry_msgs::Point backleg_pose[3];

// PID variables
float Time_step = 20;	// ms
float Kp_lin = 1;
float Ki_lin = 0.01;
float Kd_lin = 10;
float Kp_ang = 1;
float Ki_ang = 0.01;
float Kd_ang = 10;

float Cur_x_err = 0;
float Pre_x_err = 0;
float Cur_yaw_err = 0;
float Pre_yaw_err = 0;
float Cur_y_err = 0;
float Pre_y_err = 0;
float Cur_x2_err = 0;
float Pre_x2_err = 0;
float Cur_yaw2_err = 0;
float Pre_yaw2_err = 0;
float Cur_y2_err = 0;
float Pre_y2_err = 0;
float X_err_thres = 0.05;	// m
float Y_err_thres = 0.05;	// m
float Yaw_err_thres = 0.2 ;	// rad
float Vel_lin_thres = 0.3;	// m/s
float Vel_ang_thres = 0.5;	// rad/s
float Integral_yaw = 0;
float Integral_y = 0;
float Integral_x = 0;
float Integral_yaw1 = 0;
float Integral_y1 = 0;
float Integral_x1 = 0;
float Integral_yaw2 = 0;
float Integral_y2 = 0;
float Integral_x2 = 0;

float PID_yaw_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_ang;
	ki = Ki_ang;
	kd = Kd_ang;

	pre_err = Pre_yaw_err;
	cur_err = atan2(table_pose[1].y, table_pose[1].x);

	ROS_INFO("Cur_yaw_err = %f", cur_err);

	integral = Integral_yaw + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_ang_thres) pid_out = Vel_ang_thres;
	else if (pid_out < -Vel_ang_thres) pid_out = -Vel_ang_thres;

	ROS_INFO("PID_out_yaw1 = %f", pid_out);

	//if (abs(cur_err) < Yaw_err_thres) State = 2;
	//else State = 1;

	Pre_yaw_err = cur_err;

	return pid_out;
}


float PID_x_maintain()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_x_err;

	if (Pre_x_maintain == 0)
	{
		Pre_x_pose = table_pose[1].x;
		Pre_x_maintain = table_pose[1].x;
		ROS_INFO("No pre x err");
		return 0.0;
	}

	cur_err = Pre_x_pose - table_pose[1].x;

	integral = Integral_x + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_x_maintain = %f", cur_err);
	ROS_INFO("PID_out_x_maintain = %f", pid_out);

	Pre_x_maintain = cur_err;
	Pre_x_pose = table_pose[1].x;

	return pid_out;
}

float PID_y_cmd()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_y_err;

	cur_err = -table_pose[1].x;

	integral = Integral_x1 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_y1 = %f", cur_err);
	ROS_INFO("PID_out_y1 = %f", pid_out);

	// if (abs(cur_err) < Y_err_thres) State = 3;
	// else State = 2;

	Pre_y_err = cur_err;

	// if (table_pose[6].x == 1)
	// {
	// 	State = 3;
	// }

	pid_out = -Vel_lin_thres;

	if (table_pose[6].x == 1)
	{
		State = 2;
	}
	return pid_out;
}

float PID_y1_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_y2_err;

	float m, c;	//  slope and y-intercept of line perpendular out from table centre

	m = (table_pose[2].y - table_pose[3].y) / (table_pose[2].x - table_pose[3].x);
	c = table_pose[1].y - m * table_pose[1].x;

	cur_err = c / sqrt(m*m + 1);

	integral = Integral_y1 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_y2 = %f", cur_err);
	ROS_INFO("PID_out_y2 = %f", pid_out);

	 if (abs(cur_err) < Y_err_thres) State = 3;
	//else State = 4;

	Pre_y2_err = cur_err;

	return pid_out;
}

float PID_x_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_x2_err;

	cur_err = table_pose[1].x - Stop_dist;

	integral = Integral_x1 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_x = %f", cur_err);
	ROS_INFO("PID_out_x = %f", pid_out);


	Pre_x2_err = cur_err;

	if (abs(cur_err) < X_err_thres) State = 4;

	return pid_out;
}

float PID_yaw2_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_ang;
	ki = Ki_ang;
	kd = Kd_ang;

	pre_err = Pre_yaw_err;
	cur_err = backleg_pose[2].y;

	ROS_INFO("Cur_yaw_err = %f", cur_err);

	integral = Integral_yaw2 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_ang_thres) pid_out = Vel_ang_thres;
	else if (pid_out < -Vel_ang_thres) pid_out = -Vel_ang_thres;

	ROS_INFO("PID_out_yaw2 = %f", pid_out);

	//if (abs(cur_err) < Yaw_err_thres) State = 2;
	//else State = 1;

	Pre_yaw_err = cur_err;

	return pid_out;
}
float PID_y2_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_y2_err;

	cur_err = ((backleg_pose[1].x - backleg_pose[0].x) / 3);

	integral = Integral_y1 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_y2 = %f", cur_err);
	ROS_INFO("PID_out_y2 = %f", pid_out);

	//if (abs(cur_err) < Y_err_thres) State = 3;
	//else State = 4;

	Pre_y2_err = cur_err;

	return pid_out;
}

float PID_x2_move()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_x2_err;

	cur_err = backleg_pose[2].x - Complete_dist;

	integral = Integral_x1 + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_x2 = %f", cur_err);
	ROS_INFO("PID_out_x2 = %f", pid_out);


	Pre_x2_err = cur_err;

	if (abs(cur_err) < X_err_thres) State = 5;

	return pid_out;
}

void tableCallback(const visualization_msgs::Marker msg)
{
	// update table position in global variables

	for (int i = 0; i < 7; i++)
	{
		table_pose[i].x = msg.points[i].x;
		table_pose[i].y = msg.points[i].y;
		table_pose[i].z = msg.points[i].z;
	}

	if (State < 1) State = 1;
}

void backlegCallback(const visualization_msgs::Marker msg)
{
	// update table position in global variables

	for (int i = 0; i < 3; i++)
	{
		backleg_pose[i].x = msg.points[i].x;
		backleg_pose[i].y = msg.points[i].y;
		backleg_pose[i].z = msg.points[i].z;
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

	// update odom in global variables
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3(q).getEulerYPR(D_yaw, D_pitch, D_roll);

	D_yaw = (D_yaw * 180 / 3.14159265359);
	F_cur_x = msg->pose.pose.position.x;
	F_cur_y = msg->pose.pose.position.y;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle n;
	Path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 10);
	Cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber sublaser = n.subscribe("table_marker", 2, tableCallback);
	ros::Subscriber subbackleg = n.subscribe("backleg_marker", 2, backlegCallback);
	ros::Subscriber subodom = n.subscribe("odom", 2, odomCallback);

	ros::Rate r(1/ (Time_step/1000));

	int stop_iteration = 0;

	while(ros::ok())
	{
		// wait till table is detected
		if ((State != 0) && (stop_iteration < Stop_iteration))
		{
			geometry_msgs::Twist cmd_vel;



			// adjust yaw, maintain x, adjust y
			if (State == 1)
			{
				cmd_vel.angular.z = PID_yaw_move();
				cmd_vel.linear.x = PID_x_maintain();
				cmd_vel.linear.y = PID_y_cmd();
			}
			

			// adjust yaw
			if ((State >= 2) && (State < 5))
			{
				cmd_vel.angular.z = PID_yaw_move();
				cmd_vel.linear.y = PID_y1_move();
			}

			if (State == 2)
			{
				cmd_vel.linear.x = PID_x_maintain();
			}

			if ((State == 3) && (State < 5))
			{
				cmd_vel.linear.x = PID_x_move();
			}

			


			// publish cmd_vel
			Cmd_vel_pub.publish(cmd_vel);

			ROS_INFO("State = %d", State);

			 if (State == 4) stop_iteration++;
		}

		else if ((State != 0)  && (stop_iteration < Stop_iteration1))
		{
			geometry_msgs::Twist cmd_vel;
			if (State < 6)
			{
				cmd_vel.angular.z = PID_yaw2_move();
				cmd_vel.linear.y = PID_y2_move();
				cmd_vel.linear.x = PID_x2_move();
			}
			Cmd_vel_pub.publish(cmd_vel);
			if (State == 5) stop_iteration++;
			ROS_INFO("stop_iteration = %d", stop_iteration);
		}

		else if (State == 0)
		{
			geometry_msgs::Twist cmd_vel;
			cmd_vel.angular.z = -Vel_ang_thres;
			Cmd_vel_pub.publish(cmd_vel);

		}

		
		r.sleep();	
		ros::spinOnce();	
	}


}
