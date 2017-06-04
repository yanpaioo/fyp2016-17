#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>

ros::Publisher Marker_pub;
ros::Publisher Table_pub;
ros::Publisher Laser_pub;

using namespace std;

vector<int> v_pts;
vector<int> v_comb;

sensor_msgs::LaserScan Msg_laser;

float f_clus_centre[50][2]; 


vector<vector<int>> _PermutationResult;
int PermGenerator(int n, int k)
{
    _PermutationResult.clear();
    if(k>n) return 0;

    //cout << "Input : " ;
    std::vector<int> d;
    for(int i=0;i<n;i++){
        d.push_back(i);
        //cout << d[i] << " ";
    }
      
    //cout << "\nThese are the Possible Permutations by k="<< k <<": " << endl;
    do
    {
        vector<int> kpoints;
        for (int i = 0; i < k; i++)
        {
            //cout << d[i] << " ";
            kpoints.push_back(d[i]);
        }
        //cout << endl;
        _PermutationResult.push_back(kpoints);

        std::reverse(d.begin()+k,d.end());
    } while (next_permutation(d.begin(),d.end()));

    return _PermutationResult.size();
}

float ComputeDistance(float x1, float y1, float x2, float y2)
{

  return sqrt( (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) );
}

bool CheckTable(vector<int>kpoints)
{
  float fTableWidth = 1.0;
  float fTableLength = 1.3;
  float f_dist[4];
  int dim_check[2] = {0, 0};

  float x1 = f_clus_centre[kpoints[0]][0];
  float y1 = f_clus_centre[kpoints[0]][1];
  float x2 = f_clus_centre[kpoints[1]][0];
  float y2 = f_clus_centre[kpoints[1]][1];
  float x3 = f_clus_centre[kpoints[2]][0];
  float y3 = f_clus_centre[kpoints[2]][1];
  float x4 = f_clus_centre[kpoints[3]][0];
  float y4 = f_clus_centre[kpoints[3]][1];

  fdist[0] = ComputeDistance(x1, y1, x2, y2);
  fdist[1] = ComputeDistance(x2, y2, x3, y3);
  fdist[2] = ComputeDistance(x3, y3, x4, y4);
  fdist[3] = ComputeDistance(x4, y4, x1, y1);

  if ((fdist[0] < fTableLength-0.2) && (fdist[0] > fTableLength+0.2))
  {

  }
 

}


void go(int offset, int k)
{
  int i_break_comb = 0;
  float f_dist[4];  //distance between two points
  float f_table_dim[3] = {1.0, 1.3, 1.64};
  float f_table_dim_thres = 0.2;

  if (k == 0)
  {
    ROS_INFO("k == 0");
    ROS_INFO("%d %d %d %d", v_comb[0], v_comb[1], v_comb[2], v_comb[3]);
    int i_dim_check[3] = {0, 0, 0};
    f_dist[0] = sqrt(pow(f_clus_centre[v_comb[0]][0]-f_clus_centre[v_comb[1]][0] , 2) + pow(f_clus_centre[v_comb[0]][1]-f_clus_centre[v_comb[1]][1], 2));
    f_dist[1] = sqrt(pow(f_clus_centre[v_comb[1]][0]-f_clus_centre[v_comb[2]][0] , 2) + pow(f_clus_centre[v_comb[1]][1]-f_clus_centre[v_comb[2]][1], 2));
    f_dist[2] = sqrt(pow(f_clus_centre[v_comb[2]][0]-f_clus_centre[v_comb[3]][0] , 2) + pow(f_clus_centre[v_comb[2]][1]-f_clus_centre[v_comb[3]][1], 2));
    f_dist[3] = sqrt(pow(f_clus_centre[v_comb[3]][0]-f_clus_centre[v_comb[0]][0] , 2) + pow(f_clus_centre[v_comb[3]][1]-f_clus_centre[v_comb[0]][1], 2));
  
    //ROS_INFO("dist1 - %f   dist2 - %f   dist3 - %f   dist4 - %f", f_dist[0], f_dist[1], f_dist[2], f_dist[3]);
    
    for (int y = 0; y < 4; ++y)
    {
      if ((f_dist[y] > f_table_dim[0]-f_table_dim_thres) && (f_dist[y] < f_table_dim[0]+f_table_dim_thres)) i_dim_check[0]++;
      else if ((f_dist[y] > f_table_dim[1]-f_table_dim_thres) && (f_dist[y] < f_table_dim[1]+f_table_dim_thres)) i_dim_check[1]++;
    }
    if (i_dim_check[0] == 2) ROS_INFO("Check1!");
    if (i_dim_check[1] == 2) ROS_INFO("Check2!");

    if ((i_dim_check[0] == 2 && i_dim_check[1] == 2))
    {
      visualization_msgs::Marker table;
      table.header.frame_id = "laser";
      table.header.stamp = ros::Time::now();
      table.ns = "table";
      table.action = visualization_msgs::Marker::ADD;
      table.pose.orientation.w = 1.0;
      table.id = 2;
      table.type = visualization_msgs::Marker::POINTS;
      table.scale.x = 0.1;
      table.scale.y = 0.1;
      table.color.g = 0.0;
      table.color.a = 1.0;
      table.color.r = 1.0;

      geometry_msgs::Point p;
      ROS_INFO("Table found!");

      p.x = p.y = p.z = 0.0;
      table.points.push_back(p); //robot position
      //i_break_comb = -1;
    }
    return;
  }

  for (int i = offset; i <= v_pts.size() - k; ++i)
  {
    ROS_INFO("For loop i - %d", i);
    //if (i_break_comb == -1) break;
    v_comb.push_back(v_pts[i]);
    go(i+1, k-1);
    //if (!v_comb.empty()) 
    v_comb.pop_back();
  }
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 50;
  int i_no_readings = 0;
  float f_clus_sum = 0;
  float f_range_thres = 2.5;
  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.25;

  Msg_laser.angle_increment = msg->angle_increment;
  Msg_laser.angle_min = msg->angle_min;
  Msg_laser.angle_max = msg->angle_max;
  Msg_laser.time_increment = msg->time_increment;
  Msg_laser.scan_time = msg->scan_time;
  Msg_laser.range_min = msg->range_min;
  Msg_laser.range_max = msg->range_max;
  Msg_laser.header.frame_id = "laser";

  visualization_msgs::Marker points;
  points.header.frame_id = "laser";
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
    if ((msg->ranges[i] > 0) && (msg->ranges[i] < 10)) Msg_laser.ranges[i] = msg->ranges[i];
  }
  Laser_pub.publish(Msg_laser);

  for (int i = 1; i < i_no_readings; ++i)
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
    ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    
    if (i_pt_ct == 0)
    {
      f_clus_sum = Msg_laser.ranges[i];
      f_clus_centre[i_clus_ct][0] = Msg_laser.ranges[i-1] * sin((i) * Msg_laser.angle_increment);
      f_clus_centre[i_clus_ct][1] = (-1) * Msg_laser.ranges[i-1] * cos((i) * Msg_laser.angle_increment);
    } 

    else
    {
      for (int y = 0; y < (i_pt_ct); ++y)
      {
        f_clus_sum += Msg_laser.ranges[i+y];
      }
      float ave;
      ave = f_clus_sum / (i_pt_ct);
      f_clus_centre[i_clus_ct][0] = ave * sin((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
      f_clus_centre[i_clus_ct][1] = (-1) * ave * cos((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
    }
    
    float check_range;
    check_range = sqrt(pow(f_clus_centre[i_clus_ct][0], 2) + pow(f_clus_centre[i_clus_ct][1], 2));
    if ((check_range > f_range_thres) || (i_pt_ct > i_no_threspt))
    {
      f_clus_centre[i_clus_ct][0] = f_clus_centre[i_clus_ct][1] = 0;
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
    p.x = f_clus_centre[i][0];
    p.y = f_clus_centre[i][1];
    points.points.push_back(p);
    
  }

  
  // v_pts.resize(0);
  // v_comb.resize(0);

  // for(int i = 0; i < (i_clus_ct); ++i)
  // {
  //   v_pts.push_back(i);
  // }

  ROS_INFO("There are %d clusters", i_clus_ct);
  PermGenerator(i_clus_ct, 4);
  //getchar();

  //go(0, 4);
  
  // if (i_clus_ct < 4)
  // {
  //   ROS_INFO("No suspected table leg detected within the range!");
  //   //Marker_pub.publish(points);
  //   return;
  // }
  
  
  Marker_pub.publish(points);
  
}

int main(int argc, char** argv )
{
  
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle n;
  Marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  //Table_pub = n.advertise<visualization_msgs::Marker>("table_marker", 10);
  ros::Subscriber sublaser = n.subscribe("base_scan", 10, laserCallback);

  ros::spin();
}