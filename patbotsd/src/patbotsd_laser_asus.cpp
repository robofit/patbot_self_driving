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

#include <patbotsd_laser_asus.h>

 



//Callback function that makes sensor data readings alignment.
void callback(const sensor_msgs::LaserScanConstPtr& msg_laser, const sensor_msgs::LaserScanConstPtr& msg_asus){

   //ROS INFO LASER_HERE
   double l_length_ = msg_laser->ranges.size();
   //ROS_INFO(" l_length %f ", length_); 
   double l_inc_ = msg_laser->angle_increment;
   //ROS_INFO(" l_inc %f ", inc_); 
   // first pass, see how many values are actually in range
   double l_range_min_ = msg_laser->range_min;
   //ROS_INFO("l_range_min  %f ", l_range_min_);
   double l_range_max_ = msg_laser->range_max;
   //ROS_INFO(" l_range_max %f ", l_range_max_);
   double l_angle_min_ = msg_laser->angle_min;
   //ROS_INFO("l_angle_min %f ", l_angle_min_);
   double l_angle_max_ = msg_laser->angle_max;
   //ROS_INFO(" l_angle_max %f ", l_angle_max_);
   double l_time_inc_ = msg_laser->time_increment;


   //ROS INFO ASUS_HERE
   double a_length_ = msg_asus->ranges.size();
   // ROS_INFO(" a_length %f ", a_length_); 
   double a_inc_ =msg_asus->angle_increment;
   // ROS_INFO(" a_inc %f ", a_inc_); 
   // first pass, see how many values are actually in range
   double a_range_min_ = msg_asus->range_min;
   // ROS_INFO("a_range_min  %f ", a_range_min_);
   double a_range_max_ = msg_asus->range_max;
   //ROS_INFO(" a_range_max %f ", a_range_max_);
   double a_angle_min_ = msg_asus->angle_min;
   //ROS_INFO("a_angle_min %f ", a_angle_min_);
   double a_angle_max_ = msg_asus->angle_max;
   //ROS_INFO(" a_angle_max %f ", a_angle_max_);
   double a_time_inc_ = msg_laser->time_increment;
 


   //Declare and populate laser scan
   sensor_msgs::LaserScan             scan_laser_asus_;
   scan_laser_asus_.header.frame_id = msg_laser->header.frame_id;
   scan_laser_asus_.header.stamp    = msg_laser->header.stamp; 

 
   scan_laser_asus_.angle_min       =  l_angle_min_;
   scan_laser_asus_.angle_max       =  l_angle_max_;
   scan_laser_asus_.angle_increment =  l_inc_;
   scan_laser_asus_.time_increment  =  l_time_inc_;
   scan_laser_asus_.range_min       =  l_range_min_; 
   scan_laser_asus_.range_max       =  l_range_max_;
   scan_laser_asus_.ranges.resize(l_length_);

   //populate the scan_laser_asus with the filtered laser ranges
   for(int i = 0; i < l_length_; i++){
      scan_laser_asus_.ranges[i] = msg_laser->ranges[i];
   }


   //Declare and populate asus scan
   sensor_msgs::LaserScan        asus_scan_;
   asus_scan_.header.frame_id =  msg_laser->header.frame_id;
   asus_scan_.header.stamp    =  msg_laser->header.stamp; 

   asus_scan_.angle_min       =  a_angle_min_;
   asus_scan_.angle_max       =  a_angle_max_;
   asus_scan_.angle_increment =  a_inc_;
   asus_scan_.time_increment  =  a_time_inc_;
   asus_scan_.range_min       =  a_range_min_; 
   asus_scan_.range_max       =  a_range_max_;
   asus_scan_.ranges.resize(a_length_);

   sensor_msgs::PointCloud range_lenght_;
   range_lenght_.points.resize(a_length_);
   range_lenght_.header.frame_id = msg_asus->header.frame_id;
   range_lenght_.header.stamp    = msg_asus->header.stamp; 

   for(int i = 0; i < a_length_; i++){
      range_lenght_.points[i].x     =      msg_asus->ranges[i] * cos(1.5708 - (i*a_inc_)); 
      range_lenght_.points[i].y     =     -msg_asus->ranges[i] * sin(1.5708 - (i*a_inc_));
   }

   //populate the scan_laser_asus with the 2D asus ranges
   int dec_i_ = 719;
   for(int i = 0; i < a_length_; i++){
      if(msg_asus->ranges[i] >= 30.0 ){
         ;
      }
      else{
         scan_laser_asus_.ranges[dec_i_] = msg_asus->ranges[i];
      }
      dec_i_--;
   }

   range_pub_.publish(range_lenght_);  
   laser_asus_pub_.publish(scan_laser_asus_);  

}



int main (int argc, char** argv){
  
    ros::init (argc, argv, "patbotsd_laser_asus");
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_(nh_, "scan_filter", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> xtion_sub_(nh_,  "scan_xtion", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan,sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  laser_sub_, xtion_sub_);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    laser_asus_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_laser_asus", 2);
    range_pub_      = nh_.advertise<sensor_msgs::PointCloud>("/scan_asus", 2);
   

    ros::Rate loop_rate(10);
    while(ros::ok()){	
       ros::spinOnce();
       loop_rate.sleep();
    }

   return 0;
   // clean up subscribers and publishers 
   laser_asus_pub_.shutdown();
   range_pub_.shutdown();
 

 
}
