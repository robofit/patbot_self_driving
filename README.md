# Package description:





The issue about the software is tackled directly by explaining the launch files that runs the application in the PatrolBot. They  are called **patbotsd_navigation.launch** and **patbotsd_directions_navigation.launch**. These launch files are found under **patbot_self_driving/patbotsd/launch**. And, in order to launch the application, one must type the following commands in a terminator terminal   **roslaunch patbotsd patbotsd_navigation.launch** or **roslaunch patbotsd patbotsd_directions_navigation.launch**. These launch files  run the following packages and launch files: 


* **patbotsd_slam_gmapping.launch** a launch file that  runs the gmapping package which in turn launches the laser based on SLAM package  that creates a 2D occupancy grid map, [9].
* **patbotsd_asus.launch**, a launch file that launches the **openni.launch** file that runs the asus driver. Also, it launches the **pointcloud_to_laserscan** file that converts the **point_cloud** asus readings to **laserscan** data, [10,11].     
* **rosaria**, a package that runs the **RosAria** node that provides a ROS interface for PatrolBot, [12].   
* **hokuyo_node**, a package that runs the **hokuyo_node**  that provides a ROS interface for the hokuyo laser sensor, [13].
* **patbotsd**, a package that runs the following nodes:
     * **patbotsd_filter_hokuyo** node that has to do with the filtering off some laser readings that are produced by some obstacles in the robot. 
     * **patbotsd**, a package that runs the  **patbotsd_laser_asus** node that has to do with the combination of 2D laser and 2D depth asus readings.
     * **patbotsd_map_maker_laser_asus_rotation** node that has to do with the self driving of the PatBot in the empty space of the indoor environment while making a 2D map based on the  laser and the camera\depth\points readings. The main idea of this node is to let the PatrolBot to navigate forward till an obstacle boundary is reached, then the MR turns to the left with a small angle till it finds space to move forward. It keeps rotating and moving forward till a full rotation takes place. After that, it starts turning to the right and moving forward till a full rotation also takes place. This process repeats constantly and by doing so, the MR covers all the empty space of the area.
     * **patbotsd_map_maker_laser_asus_directions** node that has to do also with the safe driving of the robot on the empty space of the area. The main idea is to place empty cones  in front of the robot and to find whether they are empty to allow the robot to navigate. These cones are separated about 45 degrees.  
     * **patbotsd**, a package that runs the  **patbotsd_asus_groundfloor_remove** node that has to do with the removing of the ground_floor of the 3D "/camera/depth/points" PointCloud2 asus topic. 
      
  
Once the launch file **roslaunch patbotsd patbotsd_navigation.launch** is launched, it runs two packages **joint_state_publisher** and  **robot_state_publisher** which in turn runs the **joint_state_publisher** and  **robot_state_publisher** nodes respectively. These nodes  have to do with  the publishing of all the join states and their transform tree (tf) of the states of the robot. Then, it establishes communication between the laptop-PatrolBot, laptop-laser and laptop-asus by running the **RosAria**, **hokuyo_node** nodes and the **openni.launch** launch file. Moreover, the **patbotsd_filter_hokuyo** node takes as an input  the **\scan** topic from the hokuyo laser and gives as an output the **scan\filter** topic which contain the filtered \scan ranges.  Furthermore, the **patbotsd_asus_groundfloor_remove** node takes as an input the the **\camera\points** topic that contains the PointCloud2 from the asus sensor and gives as output the **\PC2_asus_cut_image** topic which contains the PointCloud2 with the ground floor removed. Then, the **pointcloud_to_laserscan** node takes as an input the **\PC2_asus_cut_image** topic and delivers the   **\scan_xtion** topic which contain a LaserScan message. 


The **patbotsd_laser_asus** node has as an input the **\scan_filter** and the **\scan_xtion** topics which are synchronized and combined in a single LaserScan message and published in the **\scan_laser_asus** topic. Then, the **patbotsd_map_maker** node takes as an input the **\scan_laser_asus** topic and drives the PatrolBot in the empty space of the indoor environment while the **gmapping_node** is making a  map on a 2D grid. 
      



## Instalation instructions:
<ul>
  <li> cd ~/ros/catkin_ws/src </li>
  <li> git clone https://github.com/robofit/but_sensor_fusion.git </li>
  <li> cd ../ </li>
  <li> catkin_make </li>
</ul>

## Run simulation:
<ul>
  <li> roslaunch but_laser_rgbd_fusion laser_rgbd.launch
</ul>


# Bibliogrhapy



 [9]  ROS. gmapping. [Online]. Available: http://wiki.ros.org/gmapping.
 
[10] ——. openni launch. [Online]. Available: http://wiki.ros.org/openni launch.

[11] ——. pointcloud to laserscan. [Online]. Available: http://wiki.ros.org/pointcloud to laserscan.

[12] ——. RosAria node. [Online]. Available: http://wiki.ros.org/ROSARIA.

[13] ——. hokuyo node. [Online]. Available: http://wiki.ros.org/hokuyo node.