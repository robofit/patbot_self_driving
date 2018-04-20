cmake_minimum_required(VERSION 2.8.3)
project(patbotsd)

find_package(catkin REQUIRED COMPONENTS
  amcl
  cv_bridge
  geometry_msgs
  gmapping
  hokuyo_node
  image_transport
  laser_geometry
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)



catkin_package( )



include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable

 add_executable(patbotsd_filter_hokuyo                     src/patbotsd_filter_hokuyo.cpp)
 add_executable(patbotsd_laser_asus                        src/patbotsd_laser_asus.cpp)
 add_executable(patbotsd_map_maker_laser_asus_rotation     src/patbotsd_map_maker_laser_asus_rotation.cpp)
 add_executable(patbotsd_map_maker_laser_asus_directions   src/patbotsd_map_maker_laser_asus_directions.cpp)
 add_executable(patbotsd_map_maker_laser_asus_range_cones  src/patbotsd_map_maker_laser_asus_range_cones.cpp)
 add_executable(patbotsd_asus_groundfloor_remove           src/patbotsd_asus_groundfloor_remove.cpp)

 
## Add cmake target dependencies of the executable/library
add_dependencies(patbotsd_filter_hokuyo                     patbotsd_filter_hokuyo_generate_messages_cpp)
add_dependencies(patbotsd_laser_asus                        patbotsd_laser_asus_generate_messages_cpp)
add_dependencies(patbotsd_map_maker_laser_asus_rotation     patbotsd_map_maker_laser_asus_rotation_generate_messages_cpp)
add_dependencies(patbotsd_map_maker_laser_asus_directions   patbotsd_map_maker_laser_asus_directions_generate_messages_cpp)
add_dependencies(patbotsd_map_maker_laser_asus_range_cones  patbotsd_map_maker_laser_asus_range_cones_generate_messages_cpp)
add_dependencies(patbotsd_asus_groundfloor_remove           patbotsd_asus_groundfloor_remove_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(patbotsd_filter_hokuyo
   ${catkin_LIBRARIES}
)


target_link_libraries(patbotsd_laser_asus
   ${catkin_LIBRARIES}
 )


target_link_libraries(patbotsd_map_maker_laser_asus_rotation
   ${catkin_LIBRARIES}
)

target_link_libraries(patbotsd_map_maker_laser_asus_directions
   ${catkin_LIBRARIES}
)

target_link_libraries(patbotsd_map_maker_laser_asus_range_cones
   ${catkin_LIBRARIES}
)


target_link_libraries(patbotsd_asus_groundfloor_remove
   ${catkin_LIBRARIES}
)


