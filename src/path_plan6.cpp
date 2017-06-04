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
float Stop_dist = 0.35;	// stop dist between laser and table centre
int Stop_iteration = 50;	// stop after this iteration after step 4
// state
uint State = 0;

// odom 
float F_cur_x, F_cur_y;
double D_yaw, D_pitch, D_roll;

// table position
geometry_msgs::Point table_pose[6];

// PID variables
float Time_step = 20;	// ms
float Kp_lin = 1;
float Ki_lin = 0.01;
float Kd_lin = 5;
float Kp_ang = 0.5;
float Ki_ang = 0.01;
float Kd_ang = 5;

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

	float A[2], B[2], C[2];
	float a, b, c;

	B[0] = B[1] = 0;
	A[0] = table_pose[2].x - table_pose[3].x;
	A[1] = table_pose[2].y - table_pose[3].y;
	C[0] = 1; C[1] = 0;

	a = sqrt(pow(B[0]-C[0],2) + pow(B[1]-C[1], 2));
	b = sqrt(pow(A[0]-C[0],2) + pow(A[1]-C[1], 2));
	c = sqrt(pow(B[0]-A[0],2) + pow(B[1]-A[1], 2));

	kp = Kp_ang;
	ki = Ki_ang;
	kd = Kd_ang;

	pre_err = Pre_yaw_err;
	cur_err = acos((a*a + c*c - b*b) / (2*a*c));

	if (A[1] < 0) cur_err = -cur_err;
	ROS_INFO("Cur_yaw_err = %f", cur_err);

	integral = Integral_yaw + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_ang_thres) pid_out = Vel_ang_thres;
	else if (pid_out < -Vel_ang_thres) pid_out = -Vel_ang_thres;

	ROS_INFO("PID_out_yaw = %f", pid_out);

	if (abs(cur_err) < Yaw_err_thres) State = 2;
	else State = 1;

	Pre_yaw_err = cur_err;

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

	cur_err = table_pose[1].y;

	integral = Integral_y + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_y = %f", cur_err);
	ROS_INFO("PID_out_y = %f", pid_out);

	if (abs(cur_err) < Y_err_thres) State = 3;
	else State = 2;

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

	cur_err = table_pose[1].x - Stop_dist;

	integral = Integral_y + (cur_err * Time_step);
	derivative = (cur_err - pre_err) / Time_step;

	pid_out = (kp*cur_err) + (ki*integral) + (kd*derivative);

	if (pid_out > Vel_lin_thres) pid_out = Vel_lin_thres;
	else if (pid_out < -Vel_lin_thres) pid_out = -Vel_lin_thres;

	ROS_INFO("Cur_err_x = %f", cur_err);
	ROS_INFO("PID_out_x = %f", pid_out);


	Pre_x_err = cur_err;

	if (abs(cur_err) < X_err_thres) State = 4;

	return pid_out;
}

void tableCallback(const visualization_msgs::Marker msg)
{
	// update table position in global variables

	for (int i = 0; i < 6; i++)
	{
		table_pose[i].x = msg.points[i].x;
		table_pose[i].y = msg.points[i].y;
		table_pose[i].z = msg.points[i].z;
	}

	State = 1;
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
	ros::Subscriber subodom = n.subscribe("odom", 2, odomCallback);

	ros::Rate r(1/ (Time_step/1000));

	int stop_iteration = 0;

	while(ros::ok())
	{
		// wait till table is detected
		if ((State != 0) && (stop_iteration < Stop_iteration))
		{

			geometry_msgs::Twist cmd_vel;

			// adjust yaw
			if (State >= 1)
			{
				cmd_vel.angular.z = PID_yaw_cmd();
			}

			// adjust axis y
			if (State >= 2)
			{
				cmd_vel.linear.y = PID_y_cmd();
			}

			//adjust axis x
			if (State >= 3)
			{
				cmd_vel.linear.x = PID_x_cmd();
			}

			// publish cmd_vel

			Cmd_vel_pub.publish(cmd_vel);

			if (State == 4) stop_iteration++;


		}

		
		r.sleep();	
		ros::spinOnce();	
	}


}
