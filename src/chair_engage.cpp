#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <vector>

using namespace std;

ros::Publisher Cmd_vel_pub;

float Stop_dist = 0.32;	// stop dist between laser and table centre
int State = 0;
int Stop_iteration = 150;

geometry_msgs::Pose Chair_pose;

float Time_step = 50;	// ms
float Kp_lin = 1.0;
float Ki_lin = 0.0;
//float Ki_lin = 0.0;
float Kd_lin = 5.0;
float Kp_ang = 0.3;
float Ki_ang = 0.0;
//float Ki_ang = 0.0;
float Kd_ang = 5.0;
float Cur_x_err = 0;
float Pre_x_err = 0;
float Cur_yaw_err = 0;
float Pre_yaw_err = 0;
float Cur_y_err = 0;
float Pre_y_err = 0;
float X_err_thres = 0.01;	// m
float Y_err_thres = 0.05;	// m
float Yaw_err_thres = 0.2 ;	// rad
float Vel_lin_thres = 0.15;	// m/s
float Vel_ang_thres = 0.3;	// rad/s
float Integral_yaw = 0.0;
float Integral_y = 0.0;
float Integral_x = 0.0;

float PID_yaw_move()
{
	float time_step;
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	time_step = Time_step;
	kp = Kp_ang;
	ki = Ki_ang;
	kd = Kd_ang;

	pre_err = Pre_yaw_err;
	cur_err = Chair_pose.position.z - 0.05;

	integral = Integral_yaw + (cur_err * time_step);
	derivative = (cur_err - pre_err) / time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_ang_thres) pid_out = Vel_ang_thres;
	else if (pid_out < -Vel_ang_thres) pid_out = -Vel_ang_thres;

	Pre_yaw_err = cur_err;
	Integral_yaw = integral;

	return pid_out;
}

float PID_y_move()
{
	float time_step;
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	time_step = Time_step;
	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;

	pre_err = Pre_y_err;
	cur_err = Chair_pose.position.y - 0.02;

	integral = Integral_y + (cur_err * time_step);
	derivative = (cur_err - pre_err) / time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	Pre_y_err = cur_err;
	Integral_y = integral;

	return pid_out;
}

float PID_x_move()
{
	float time_step;
	float cur_err, pre_err;
	float integral, derivative;
	float pid_out;
	float kp, ki, kd;

	time_step = Time_step;
	kp = Kp_lin;
	ki = Ki_lin;
	kd = Kd_lin;

	pre_err = Pre_x_err;
	cur_err = Chair_pose.position.x - Stop_dist;

	integral = Integral_x + (cur_err * time_step);
	derivative = (cur_err - pre_err) / time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	Pre_x_err = cur_err;
	Integral_x = integral;

	ROS_INFO("Cur err = %f", cur_err);
	if (cur_err <= X_err_thres) State = 1;

	return pid_out;
}

void chairCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	Chair_pose.position.x = msg->position.x;
	Chair_pose.position.y = msg->position.y;
	Chair_pose.position.z = msg->position.z;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "chair_engage");
	ros::NodeHandle n;
	Cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber sublaser = n.subscribe("chair_pose", 2, chairCallback);

	ros::Rate r(1/ (Time_step/1000));
	int stop_iteration = 0;

	while(ros::ok())
	{
		if ((stop_iteration < Stop_iteration) && (Chair_pose.position.x == Chair_pose.position.y == 0))
		{
			geometry_msgs::Twist cmd_vel;

			ROS_INFO("Iteration = %d", stop_iteration);

			cmd_vel.angular.z = PID_yaw_move();
			cmd_vel.linear.y = PID_y_move();
			cmd_vel.linear.x = PID_x_move();

			Cmd_vel_pub.publish(cmd_vel);

			if (State == 1) stop_iteration++;
			
			if (stop_iteration == Stop_iteration) ROS_INFO("Robot stopped!");

		}
		

		r.sleep();	
		ros::spinOnce();	
	}
}