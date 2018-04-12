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



#include <patbotsd_asus_groundfloor_remove.h>




void  Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg){   
  
   // camera/depth/cloud info
   int width_ = cloud2_msg->width;
   int height_ = cloud2_msg->height;
   int step_ = cloud2_msg->point_step;
   int data_ = cloud2_msg->data.size();



   //declare and set up a point cloud 
   sensor_msgs::PointCloud cloud_msg;
   cloud_msg.header.frame_id = cloud2_msg->header.frame_id;
   cloud_msg.header.stamp = cloud2_msg->header.stamp;

   sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_msg,cloud_msg);
   int cloud_size = cloud_msg.points.size();
 
   //remove the ground floor
   for(int i = 0; i < cloud_size; i++){
      if(cloud_msg.points[i].y >= 0.3)
         cloud_msg.points[i].y = a;

   //cout<<"z:= "<< cloud_msg.points[i].z << endl;

}
   cut_raw_image_pub_.publish(cloud_msg);  



   //declare and set up a point cloud2
   sensor_msgs::PointCloud2 PC2_asus_msg_;
   PC2_asus_msg_.header.frame_id = cloud2_msg->header.frame_id;
   PC2_asus_msg_.header.stamp = cloud2_msg->header.stamp;

   sensor_msgs::convertPointCloudToPointCloud2(cloud_msg,PC2_asus_msg_);
   PC2_asus_pub_.publish(PC2_asus_msg_);
 
}

int main (int argc, char** argv){
   //Initialize ROS
   ros::init (argc, argv, "patbotsd_asus_groundfloor_remove");
   ros::NodeHandle nh_;

   //Create a ROS subscriber for the input point cloud
   ros::Subscriber image_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Cloud_Callback);

   //Create a ROS publisher for the output point cloud
   cut_raw_image_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("/PC_asus_cut_image", 5);
   PC2_asus_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_asus_cut_image", 2);

   // Spin
   ros::Rate loop_rate(10);
   while(ros::ok()){	
      ros::spinOnce();
      loop_rate.sleep();
   }


   // clean up subscribers and publishers
   cut_raw_image_pub_.shutdown();
   image_cloud_sub_.shutdown();
   PC2_asus_pub_.shutdown();

}
