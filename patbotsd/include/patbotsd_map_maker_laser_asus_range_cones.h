/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Alfredo Chavez Plascencia(plascencia@fit.vutbr.cz)
 * 
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */


/* heder files */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

/*Standard library*/
using namespace std;


/*class definition goes here*/
class Patbot_Map_Maker{
   private:
   ros::NodeHandle nh_;

   // global ROS subscriber handles
   ros::Subscriber laser_scan_sub_;
   ros::Subscriber amcl_pose_;
   ros::Subscriber patbot_pose_sub_;
 
   // global ROS publisher handles
   ros::Publisher  xy_pub_;
   ros::Publisher  range_pub_;
   ros::Publisher  vel_pub_;
   ros::Publisher  pub;
   ros::Publisher  range_front_pub_;

   // Yaw angle
   double yaw_;

   //yaw degree angle
   double yaw_degrees_;

   //convert radians2degrees
   double R2D;
  
   //convert degrees2radians
   double D2R;
 
   //Several variables
   double temp_angle;
   double temp_vel;
   double max_range_element_temp_;
   double max_range_element_left_temp_;
   double max_range_element_right_temp_;
   double max_range_element_x;  
   double max_range_element_y;  
   double max_range_angle;
   double max_angle_index_;
   double patbot_angle_;
   double max_range_element_temp_x_; 
   double max_range_element_temp_y_;  	
   double free_space;
   double angle_index_;
   double angle_index_temp_;
   double patbot_rotation_angle_;
   double displacement_;
   double stop_rotating_flag_;
   double rotate_right_; 
  

   int index_max_range_element,max_index_;
   int var_lr_;
   int wall_count_right_, wall_count_left_;
   int wall_count_front_right_, wall_count_front_left_;
   int wall_count_front_right_right_, wall_count_front_left_left_;

   tf::TransformListener listener;

public:

   Patbot_Map_Maker();
  ~Patbot_Map_Maker();

   /*Function definitions*/
   void Scan_Hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
   void Load_Global_Parameters(void);
   void broadcast_scanner_tf();
   void run();
   
   

  vector<double> laser_range_vector_;  
  vector<double> patbot_front_free_space_;
  vector<double> patbot_right_free_space_;
  vector<double> patbot_left_free_space_;



  vector<double> patbot_front_right_free_space_;
  vector<double> patbot_front_right_right_free_space_;
  vector<double> patbot_front_left_free_space_;
  vector<double> patbot_front_left_left_free_space_;


  
   //Bolean Variables   
   bool max_range_flag_;
   bool forward_flag;
   bool free_space_flag;
   bool displacement_flag_;
   bool obstacle_flag_;
   bool forward_flag_;
   bool forward_right_flag_;
   bool forward_left_flag_;
   bool right_rotation_condition_flag_;
   bool front_right_rotation_condition_flag_;
   bool front_right_right_rotation_condition_flag_;
   bool left_rotation_condition_flag_;
   bool front_left_rotation_condition_flag_;
   bool front_left_left_rotation_condition_flag_;
   bool activation_flag_right_;
   bool left_, right_;
   bool front_left_, front_right_;
   bool front_left_left_, front_right_right_;
   bool wall_flag_;

   struct patbot_direction_{
      string direction_;
   }; 
   patbot_direction_ left_right[4];
  
};
