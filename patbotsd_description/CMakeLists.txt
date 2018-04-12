cmake_minimum_required(VERSION 2.8.3)
project(patbotsd_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  urdf
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


