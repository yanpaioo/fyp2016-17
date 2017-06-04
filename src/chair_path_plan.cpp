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

using namespace std;

ros::Publisher Cmd_vel_pub;

// general variable
float Pi = 3.1416;
float Stop_dist = 0.3;	// stop dist between laser and chair centre
int Stop_iteration = 50;	// stop after this iteration after step 4
// state
uint State = 0;

// odom 
float F_cur_x, F_cur_y;
double D_yaw, D_pitch, D_roll;

// chair position
geometry_msgs::Point chair_pose[6];

// PID variables
float Time_step = 20;	// ms
float Kp_lin = 1;
float Ki_lin = 0.001;
float Kd_lin = 1;
float Kp_ang = 1.0;
float Ki_ang = 0.01;
float Kd_ang = 10.0;

float Cur_x_err = 0;
float Pre_x_err = 0;
float Cur_yaw_err = 0;
float Pre_yaw_err = 0;
float Cur_y_err = 0;
float Pre_y_err = 0;
float X_err_thres = 0.05;	// m
float Y_err_thres = 0.05;	// m
float Yaw_err_thres = 0.08 ;	// rad
float Vel_lin_thres = 0.3;	// m/s
float Vel_ang_thres = 0.5;	// rad/s
float Integral_yaw = 0;
float Integral_y = 0;
float Integral_x = 0;

float PID_yaw_cmd()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_ang;
	ki = Ki_ang;
	kd = Kd_ang;

	pre_err = Pre_yaw_err;
	cur_err = chair_pose[1].y;

	//integral = Integral_yaw + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_ang_thres) pid_out = Vel_ang_thres;
	else if (pid_out < -Vel_ang_thres) pid_out = -Vel_ang_thres;

	ROS_INFO("PID_out_yaw = %f", pid_out);

	Pre_yaw_err = cur_err;
	Integral_yaw = integral;
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

	cur_err = (chair_pose[4].x - chair_pose[1].x)/3;

	integral = Integral_y + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_y = %f", cur_err);
	ROS_INFO("PID_out_y = %f", pid_out);

	Pre_y_err = cur_err;
	return pid_out;
}

float PID_x_cmd()
{
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;
	pre_err = Pre_x_err;

	cur_err = chair_pose[1].x - Stop_dist;

	integral = Integral_x + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_x2 = %f", cur_err);
	ROS_INFO("PID_out_x2 = %f", pid_out);

	Pre_x_err = cur_err;

	return pid_out;

}


void chairCallback(const visualization_msgs::Marker msg)
{
	// update chair position in global variables

	for (int i = 0; i < 6; i++)
	{
		chair_pose[i].x = msg.points[i].x;
		chair_pose[i].y = msg.points[i].y;
		chair_pose[i].z = msg.points[i].z;
	}

	if (State == 0) State = 1;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle n;
	Cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber sublaser = n.subscribe("chair_marker", 2, chairCallback);

	ros::Rate r(1/ (Time_step/1000));

	int stop_iteration = 0;

	while(ros::ok())
	{
		if ((State != 0) && (stop_iteration < Stop_iteration))
		{
			geometry_msgs::Twist cmd_vel;

			if (State >= 1)
			{
				cmd_vel.angular.z = PID_yaw_cmd();
				cmd_vel.linear.y = PID_y_cmd();
				cmd_vel.linear.x = PID_x_cmd();
			}

			Cmd_vel_pub.publish(cmd_vel);
		}
		

		
		r.sleep();	
		ros::spinOnce();	
	}


}