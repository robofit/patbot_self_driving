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
#include "patbotsd_map_maker_laser_asus_rotation.h"




   void Patbot_Map_Maker::run(){

   /*run the global parameters*/
   Load_Global_Parameters();
 
   //set up subscribers
      laser_scan_sub_  = nh_.subscribe<sensor_msgs::LaserScan> ("/scan_laser_asus", 10, &Patbot_Map_Maker::Scan_Hokuyo_callback, this); 


   //set up publishers
   xy_pub_             =  nh_.advertise<visualization_msgs::Marker>( "/xy_marker_", 2 );
   range_pub_          =  nh_.advertise<sensor_msgs::PointCloud>("/laser_scaner_pose", 2);
   range_front_pub_    =  nh_.advertise<sensor_msgs::PointCloud>("/laser_range_front", 2);
   vel_pub_            =  nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
  
}

   /*Run the destructor*/
   Patbot_Map_Maker::~ Patbot_Map_Maker(){

   // clean up subscribers and publishers
   laser_scan_sub_.shutdown();
   xy_pub_.shutdown();
   range_pub_.shutdown();
   vel_pub_.shutdown();
   pub.shutdown();
   amcl_pose_.shutdown();
   patbot_pose_sub_.shutdown();
   range_front_pub_.shutdown();
}

/*call back function*/
void Patbot_Map_Maker::Scan_Hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

   //declare twuist variable for the linear and angular velocities
   geometry_msgs::Twist twist; 

   //transform /base_link to /map
   tf::StampedTransform transform;
      try{
         ros::Time now = ros::Time(0);//msg->header.stamp;
         listener.waitForTransform("/map", "/base_link",now, ros::Duration(3.0));
         listener.lookupTransform( "/map","/base_link", now, transform);
      }
      catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
      }
    //get yaw rotation from -1.57 to 1.57 
    tf::Quaternion q = transform.getRotation();
    yaw_ = tf::getYaw(q);

   //rotation angles in the range of 0 to 360
   yaw_degrees_ = round(yaw_ * 180.0 / M_PI); // conversion to degrees
   if( yaw_degrees_ < 0 ) 
      yaw_degrees_ += 360.0; // convert negative to positive angles
     

   //ROS INFO LASER_HERE
   double length = msg->ranges.size();
   //ROS_INFO(" length %f ", length);    
   double inc =msg->angle_increment;
   //ROS_INFO(" inc %f ", inc); 
   //first pass, see how many values are actually in range
   double range_min = msg->range_min;
   //ROS_INFO("range_min  %f ", range_min);
   double range_max = msg->range_max;
   //ROS_INFO(" range_max %f ", range_max);
   double angle_min = msg->angle_min;
   //ROS_INFO("angle_min %f ", angle_min);  
   double angle_max = msg->angle_max;
   //ROS_INFO(" angle_max %f ", angle_max);
   double alpha =- angle_min;// -120/180*PI;

   //Get valid values within laser min and max values
   int num_valid = 0;
   laser_range_vector_.clear();
   for(int i = 0; i < length; i++){
      //if(msg->ranges[i] >= range_min && msg->ranges[i] <= range_max)
      laser_range_vector_.push_back(double(msg->ranges[i]));
      num_valid++;
   }
   if(num_valid <= 0)
      return;
    
   //Get the max element in the array and the index value    
   int max_range_element = int(*max_element(laser_range_vector_.begin(),laser_range_vector_.end()));
   int index_max_range_element = int(distance(laser_range_vector_.begin(), max_element(laser_range_vector_.begin(),laser_range_vector_.end())));
   
   //Get [x,y,theta] values of the max element of the array  
    max_range_element_x  =  laser_range_vector_[index_max_range_element] *cos(1.5708-(index_max_range_element*inc)); //laser_range_vector_[i] *sin(alpha+theta +robot_pose_->z)*100;
    max_range_element_y  =  laser_range_vector_[index_max_range_element] *sin(1.5708-(index_max_range_element*inc)); //laser_range_vector_[i] *cos(alpha+theta +robot_pose_->z)*100;
    max_range_angle      =  atan((max_range_element_y)/(max_range_element_x));

   //Get max values in front of the robot
   if(max_range_flag_ == true){
      max_range_element_left_temp_   =  laser_range_vector_[index_max_range_element-28];
      max_range_element_temp_        =  laser_range_vector_[index_max_range_element];
      max_range_element_right_temp_  =  laser_range_vector_[index_max_range_element+28];
      max_angle_index_               =  index_max_range_element*inc*R2D;
      max_index_                     =  index_max_range_element;
      max_range_element_temp_x_      =  max_range_element_x; 
      max_range_element_temp_y_      =  max_range_element_y;  
      max_range_flag_         = false;
   }

   
   //Store all ranges in front of the robot in a vector
   patbot_front_frees_pace_.clear();
   patbot_front_frees_pace_.resize(free_space);
   for(int i = 0; i < free_space; i++){
      double vectorin = laser_range_vector_[(359-49)+i];
      patbot_front_frees_pace_[i] = vectorin;
   }


   //Print out the robot angle
   cout<<"yaw_degrees_:= "<< yaw_degrees_ << endl;
      
  
   //Assure to rotate to the right when the robot has rotated to the left
   if((angle_index_ == 70.0) && (yaw_degrees_>= 353.0) && (yaw_degrees_ <= 356.0) && (left_ == true)){
      right_         = true;
      left_          = false;
      temp_angle     = -0.2;
      obstacle_flag_ = true;
      //cout <<"left_:= "<< endl;
      //cout << endl;
   }
   //Assure to rotate to the left when the robot has rotated to the right
   if((angle_index_ == 110.0) && (yaw_degrees_ >= 3.0) && (yaw_degrees_ <= 6.0) && (right_ == true)){
      left_          = true;
      right_         = false;
      temp_angle     = 0.2;
      obstacle_flag_ = true;
      //cout <<"right_:= "<< endl;
      //cout << endl;
  }

   //Get the velocity and rotation angle 
   if(obstacle_flag_ == true){
      for(int i = 0; i < free_space; i++){
         if(patbot_front_frees_pace_[i] < 1.8){
            if(left_ == true){
               angle_index_ = 70.0;  
            }
            if(right_ == true){  
 	       angle_index_ = 110.0; 
            }    
            temp_vel        =  0.0;
            free_space_flag = true;
            obstacle_flag_  = false;
            forward_flag_   = false;
         }   
      }
   }

   //Publish the twist variable values for angular and lineal velocity
   twist.linear.x     = temp_vel;
   twist.angular.z    = temp_angle;
   vel_pub_.publish(twist);

   //Set forward velocity and stop rotating 
   if(forward_flag_==true){
      temp_vel   = 0.2;
      temp_angle = 0.0;
   }     

   
   //Set rotation angle
   if(free_space_flag == true){
      if((angle_index_>=0.0) && (angle_index_<=90.0)){     
         temp_angle      =  0.2;
         patbot_angle_   = -angle_index_+90;
         free_space_flag =  false;
      }

      if((angle_index_>=90.0) && (angle_index_<=180.0)){
         temp_angle = -0.2;
         free_space_flag = false;
         patbot_angle_=((270-360)*(angle_index_-90))/(90)+360 ;
      }

      if(angle_index_>90.0){  
         if(displacement_flag_==true){
            displacement_=abs(360 - patbot_angle_);
            displacement_flag_=false;
         }
         //cout<< endl;
         //cout<<"right: "<<endl;
         patbot_rotation_angle_= yaw_degrees_-displacement_;
         if(patbot_rotation_angle_ < 0)
            patbot_rotation_angle_ = 359 + patbot_rotation_angle_;
           
         left_right[var_lr_].direction_ = "right";  
      }  

      if(angle_index_<90.0){
         if(displacement_flag_==true){
            displacement_=abs(patbot_angle_);
            displacement_flag_=false;
         }
         //cout<< endl;
         //cout<<"left: "<<endl;
         patbot_rotation_angle_= yaw_degrees_ + displacement_;
         if(patbot_rotation_angle_ > 359)
            patbot_rotation_angle_ = patbot_rotation_angle_ - 359;
        
         left_right[var_lr_].direction_ = "left";
      }  
      
      stop_rotating_flag_    = true;
      displacement_flag_     = false;
      forward_space_flag_    = true;
      
      var_lr_++;
   }
 
   //Scape condition 
   if(var_lr_ == 4){
      if((left_right[0].direction_ == "left") && (left_right[1].direction_ == "right") && (left_right[2].direction_ == "left") &&(left_right[3].direction_ == "right")){
         patbot_rotation_angle_= yaw_degrees_ + displacement_ + 10.0;
            if(patbot_rotation_angle_>359)
               patbot_rotation_angle_ = patbot_rotation_angle_ - 359;  
      }
      else if((left_right[0].direction_ == "right") && (left_right[1].direction_ == "left") && (left_right[2].direction_ == "right") &&(left_right[3].direction_ == "left")){
         patbot_rotation_angle_= yaw_degrees_-displacement_ - 10.0;
            if(patbot_rotation_angle_ < 0.0)
               patbot_rotation_angle_ = 359 + patbot_rotation_angle_;
      }
      var_lr_=0;
   }
   //Stop rotation when the robot reach a desired angle 
   if(stop_rotating_flag_==true){
      if((yaw_degrees_>=(patbot_rotation_angle_-2))&&(yaw_degrees_<=(patbot_rotation_angle_+2))){
         temp_angle               = 0.0;
         free_space_flag          = false;
         stop_rotating_flag_      = false;
         obstacle_flag_           = true;
         forward_flag_            = true;
         rotation_condition_flag_ = true;
     }
   }

   //Publish the twist variable values for angular and lineal velocity
   twist.linear.x     = temp_vel;
   twist.angular.z    = temp_angle;
   vel_pub_.publish(twist);


  
   

   //declare point cloud objetcs for the coordenates x y
   sensor_msgs::PointCloud range_lenght_;
   range_lenght_.points.resize(num_valid);
   range_lenght_.header.frame_id = "/laser";


   //Get the coordantes [x,y] from range readings    
   for(int i = 0; i < length; i++){
      range_lenght_.points[i].x  =    laser_range_vector_[i] *cos(1.5708-(i*inc)); 
      range_lenght_.points[i].y  =   -laser_range_vector_[i] *sin(1.5708-(i*inc));    
   }
  range_pub_.publish(range_lenght_); 


   sensor_msgs::PointCloud range_front_;
   range_front_.points.resize(free_space);
   range_front_.header.frame_id = "/laser";


   //Get the coordantes [x,y] from range readings    
   for(int i = 0; i < free_space; i++){
      range_front_.points[i].x  =     laser_range_vector_[((359-49)+i)] *cos(1.5708-(((359-49)+i)*inc)); //laser_range_vector_[i] *sin(alpha+theta +robot_pose_->z)*100;
      range_front_.points[i].y  =    -laser_range_vector_[((359-49)+i)] *sin(1.5708-(((359-49)+i)*inc)); //laser_range_vector_[i] *cos(alpha+theta +robot_pose_->z)*100; 
   }
   range_front_pub_.publish(range_front_); 

  



 
//Plot the middle value of the front of the robot
/////////////////////////
/////////////////////////
/////////////////////////
for(int i = 0; i < num_valid; i++){
int num= i;
visualization_msgs::Marker marker;
marker.header.frame_id = "laser";
marker.header.stamp = ros::Time(0);
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::CUBE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x =        laser_range_vector_[363] *cos(1.5708-(363*inc));
marker.pose.position.y =       -laser_range_vector_[363] *sin(1.5708-(363*inc));
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.3;
marker.scale.y = 0.3;
marker.scale.z = 0.3;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
xy_pub_.publish( marker );
}
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
   
   }


/*Global parameters*/
void Patbot_Map_Maker::Load_Global_Parameters(void){

   
   
   max_range_flag_             =  true;
   forward_flag_               =  true;
   free_space_flag             =  false;
   displacement_flag_          =  true;
   stop_rotating_flag_         =  false;
   obstacle_flag_              =  true;
   forward_space_flag_         =  false;
   rotation_condition_flag_    =  false;
   left_                       =  true;
   right_                      =  false;
   
   yaw_                        =  0.0;
   yaw_degrees_                =  0.0;
   D2R                         =  0.017453;
   R2D                         =  57.296;
   temp_angle                  =  0.0;
   temp_vel                    =  0.0;
   max_range_element_temp_     =  0.0;
   index_max_range_element     =  0;
   max_index_                  =  0;
   max_range_element_x         =  0.0;  
   max_range_element_y         =  0.0;
   max_range_angle             =  0.0;
   max_angle_index_            =  0.0;
   patbot_angle_               =  0.0;
   max_range_element_temp_x_   =  0.0; 
   max_range_element_temp_y_   =  0.0;  
   free_space                  =  98.0; //number of rays in fron of the robot
   angle_index_                =  70.0;
   patbot_rotation_angle_      =  0.0;
   displacement_               =  0.0;
   var_lr_                     =  0;
     
}










int main(int argc, char** argv){
   ros::init(argc, argv, "patbotsd_map_maker_laser_asus_rotation");
   Patbot_Map_Maker patbot_map_maker;

   ros::Rate loop_rate(5);
   while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }

  return (0);
}
