#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

#define ROBOT 1

sensor_msgs::LaserScan Msg_laser;
vector<vector<int> > V_combResult;

ros::Publisher Laser_pub;
ros::Publisher Marker_pub;
ros::Publisher Chair_pub;
ros::Publisher Chair_pose_pub;

float F_clus_centre[50][2];     // {x, y}
float F_chair_pose[5][2];       // {center, 2 pts making length, 2 pts making width}

float F_chair_dim[2] = {0.28, 0.38};
float F_chair_dim_thres = 0.05;

bool checkChair(float dist1, float dist2, float dist3, float dist4)
{
  int i_dim_check[2] = {0, 0};
  float f_dist[4];

  f_dist[0] = dist1;
  f_dist[1] = dist2;
  f_dist[2] = dist3;
  f_dist[3] = dist4;

  for (int i = 0; i < 4; i++)
  {
    if ((f_dist[i] > F_chair_dim[0]-F_chair_dim_thres) && (f_dist[i] < F_chair_dim[0]+F_chair_dim_thres))
    {
      i_dim_check[0]++;
    }
    else if ((f_dist[i] > F_chair_dim[1]-F_chair_dim_thres) && (f_dist[i] < F_chair_dim[1]+F_chair_dim_thres))
    {
      i_dim_check[1]++;
    }
  }

  ROS_INFO("Check1 = %d   Check2 = %d", i_dim_check[0], i_dim_check[1]);
  if ((i_dim_check[0] == 1) && (i_dim_check[1]) == 3)
  {
    ROS_INFO("Chair found!");
    return 1;
  }

  else return 0;


}

float computeDist(int pt_index1, int pt_index2)
{
  float x1, x2, y1, y2;

  x1 = F_clus_centre[pt_index1][0];
  x2 = F_clus_centre[pt_index2][0];
  y1 = F_clus_centre[pt_index1][1];
  y2 = F_clus_centre[pt_index2][1];

  return sqrt( ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) );
}

int combination(int n, int k)
{
    std::string bitmask(k, 1); // K leading 1's
    bitmask.resize(n, 0); // N-K trailing 0's

    if (k > n) return 0;
    V_combResult.clear();

    do
    {
      vector<int> v_pts;
      for (int i = 0; i < n; ++i) // [0..N-1] integers
      {
        if (bitmask[i]) v_pts.push_back(i);
      }
      V_combResult.push_back(v_pts);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    return V_combResult.size();
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int i_comb_ct = 0;
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 30;
  float f_clus_sum = 0;
  int i_no_readings = 0;
  float f_range_thres = 2;
  
  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.15;

  Msg_laser.angle_increment = msg->angle_increment;
  Msg_laser.angle_min = msg->angle_min;
  Msg_laser.angle_max = msg->angle_max;
  Msg_laser.time_increment = msg->time_increment;
  Msg_laser.scan_time = msg->scan_time;
  Msg_laser.range_min = msg->range_min;
  Msg_laser.range_max = msg->range_max;
  Msg_laser.header.frame_id = "base_laser_link";

  visualization_msgs::Marker points;
  points.header.frame_id = "base_laser_link";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  i_no_readings = 1 + ((Msg_laser.angle_max - Msg_laser.angle_min)/Msg_laser.angle_increment);
  Msg_laser.ranges.resize(i_no_readings);

  for (int i = 0; i < i_no_readings; ++i)
  {
    Msg_laser.ranges[i] = msg->ranges[i];

    if(isnan(Msg_laser.ranges[i]) || isinf(Msg_laser.ranges[i]))
    {
      Msg_laser.ranges[i] = 6;
    }
  }
  Laser_pub.publish(Msg_laser);

  for (int i = 20; i < i_no_readings - 20; ++i)
  {
    f_pre_pose[0] = Msg_laser.ranges[i-1] * sin((i-1) * Msg_laser.angle_increment);
    f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1] * cos((i-1) * Msg_laser.angle_increment);
    f_cur_pose[0] = Msg_laser.ranges[i] * sin((i) * Msg_laser.angle_increment);
    f_cur_pose[1] = (-1) * Msg_laser.ranges[i] * cos((i) * Msg_laser.angle_increment);

    f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0], 2) + pow(f_cur_pose[1]-f_pre_pose[1], 2));

    while(f_dist_diff < f_diff_thres)
    {
      i_pt_ct++;
      if (i_no_readings < i+i_pt_ct) break;
      f_pre_pose[0] = Msg_laser.ranges[i-1+i_pt_ct] * sin((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1+i_pt_ct] * cos((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[0] = Msg_laser.ranges[i+i_pt_ct] * sin((i+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[1] = (-1) * Msg_laser.ranges[i+i_pt_ct] * cos((i+i_pt_ct) * Msg_laser.angle_increment);
      f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));
      
    }
    //ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    
    if (i_pt_ct == 0)
    {
      f_clus_sum = Msg_laser.ranges[i];
      F_clus_centre[i_clus_ct][0] = Msg_laser.ranges[i-1] * sin((i) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * Msg_laser.ranges[i-1] * cos((i) * Msg_laser.angle_increment);
    } 

    else
    {
      for (int y = 0; y < (i_pt_ct); ++y)
      {
        f_clus_sum += Msg_laser.ranges[i+y];
      }
      float ave;
      ave = f_clus_sum / (i_pt_ct);
      F_clus_centre[i_clus_ct][0] = ave * sin((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * ave * cos((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
    }

    float check_range;
    check_range = sqrt(pow(F_clus_centre[i_clus_ct][0], 2) + pow(F_clus_centre[i_clus_ct][1], 2));
    if ((check_range > f_range_thres) || (i_pt_ct > i_no_threspt))
    {
      F_clus_centre[i_clus_ct][0] = F_clus_centre[i_clus_ct][1] = 0;
    }
    else
    {
      i_clus_ct++;
    }
    
    i = i + i_pt_ct;
    i_pt_ct = 0;
    f_clus_sum = 0;
  }    


  for (int i = 0; i < i_clus_ct; ++i)
  {
    geometry_msgs::Point p;
    p.x = F_clus_centre[i][0];
    p.y = F_clus_centre[i][1];
    points.points.push_back(p);
  }
  
  Marker_pub.publish(points);
  ROS_INFO("Cluster count = %d", i_clus_ct);

  i_comb_ct = combination(i_clus_ct, 4);

  ROS_INFO("No of combination = %d", i_comb_ct);

  for (int i = 0; i < i_comb_ct; i++)
  {
    ROS_INFO("i = %d", i);
    float f_dist1, f_dist2, f_dist3, f_dist4;
    bool b_chair_check = 0;

    ROS_INFO("Combinations - %d %d %d %d", V_combResult[i][0], V_combResult[i][1], V_combResult[i][2], V_combResult[i][3]);
    f_dist1 = computeDist(V_combResult[i][0], V_combResult[i][1]);
    f_dist2 = computeDist(V_combResult[i][1], V_combResult[i][2]);
    f_dist3 = computeDist(V_combResult[i][2], V_combResult[i][3]);
    f_dist4 = computeDist(V_combResult[i][3], V_combResult[i][0]);

    ROS_INFO("%f   %f   %f   %f", f_dist1, f_dist2, f_dist3, f_dist4);

    b_chair_check = checkChair(f_dist1, f_dist2, f_dist3, f_dist4);

    if (b_chair_check == 1)
    {
      int i_pt_index[4];
      visualization_msgs::Marker chair;
      chair.header.frame_id = "base_laser_link";
      chair.header.stamp = ros::Time::now();
      chair.ns = "table";
      chair.action = visualization_msgs::Marker::ADD;
      chair.pose.orientation.w = 1.0;
      chair.id = 2;
      chair.type = visualization_msgs::Marker::POINTS;
      chair.scale.x = 0.2;
      chair.scale.y = 0.2;
      chair.color.g = 0.0;
      chair.color.b = 0.0;
      chair.color.a = 1.0;
      chair.color.r = 1.0;

      
      geometry_msgs::Point p;

      for (int y = 0; y < 4; y++)
      {
        i_pt_index[y] = V_combResult[i][y];
      } 
      p.x = p.y = p.z = 0.0;
      chair.points.push_back(p); //robot position

      for (int y = 0; y < 4; y++)
      {
        p.x += F_clus_centre[i_pt_index[y]][0];
        p.y += F_clus_centre[i_pt_index[y]][1];
      }

      geometry_msgs::Pose chair_pose;
      chair_pose.position.x = p.x = p.x / 4.0;
      chair_pose.position.y = p.y = p.y / 4.0;

      chair.points.push_back(p);



      float x1, x2, y1, y2;

      x1 = (F_clus_centre[V_combResult[i][0]][0] + F_clus_centre[V_combResult[i][3]][0]) / 2 ;
      y1 = (F_clus_centre[V_combResult[i][0]][1] + F_clus_centre[V_combResult[i][3]][1]) / 2 ;

      x2 = (F_clus_centre[V_combResult[i][1]][0] + F_clus_centre[V_combResult[i][2]][0]) / 2 ;
      y2 = (F_clus_centre[V_combResult[i][1]][1] + F_clus_centre[V_combResult[i][2]][1]) / 2 ;

      ROS_INFO("x1=%f  y1=%f  x2=%f  y2=%f", x1, y1, x2, y2);

      float a, b, c;
      c = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
      a = sqrt((y2-y1)*(y2-y1));
      b = sqrt((x2-x1)*(x2-x1));

      chair_pose.position.z = acos((b*b + c*c - a*a)/(2*b*c));
      if (y2 < y1) chair_pose.position.z = -chair_pose.position.z;
    

      ROS_INFO("Centre x= %f y = %f", p.x, p.y);

      p.x = F_clus_centre[V_combResult[i][0]][0];
      p.y = F_clus_centre[V_combResult[i][0]][1];
      chair.points.push_back(p);
      p.x = F_clus_centre[V_combResult[i][1]][0];
      p.y = F_clus_centre[V_combResult[i][1]][1];
      chair.points.push_back(p);
      p.x = F_clus_centre[V_combResult[i][2]][0];
      p.y = F_clus_centre[V_combResult[i][2]][1];
      chair.points.push_back(p);
      p.x = F_clus_centre[V_combResult[i][3]][0];
      p.y = F_clus_centre[V_combResult[i][3]][1];
      chair.points.push_back(p);


      Chair_pub.publish(chair);
      Chair_pose_pub.publish(chair_pose);
      
      break;
    }
  }

}

int main(int argc, char** argv )
{
  ros::init(argc, argv, "chair_detection");
  ros::NodeHandle n;
  Marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  Chair_pub = n.advertise<visualization_msgs::Marker>("chair_marker", 10);
  Chair_pose_pub = n.advertise<geometry_msgs::Pose>("chair_pose", 10);
  ros::Subscriber sublaser = n.subscribe("scan", 10, laserCallback);

  ros::spin();
}