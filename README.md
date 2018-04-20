# PatrolBot Self Driving main repository.

Autonomous driving for map making of a clutter environments is a complex task due to the robot has to deal with precise measurements of the environment, in other words the robot must have a good field of view of the environment in turn. We mainly present the functioning and description of the ROS package called patbotsd that has to do with the navigation of a PatrolBot in the empty area of a indoor environment while making a 2D map by means of a laser hokuyo and a 3D depth asus sensors.

## Package description:








* **patbotsd_slam_gmapping.launch** a launch file that  runs the gmapping package which in turn launches the laser based on SLAM package  that creates a 2D occupancy grid map, [1].
* **patbotsd_asus.launch**, a launch file that launches the **openni.launch** file that runs the asus driver. Also, it launches the **pointcloud_to_laserscan** file that converts the **point_cloud** asus readings to **laserscan** data, [2,3].     
* **rosaria**, a package that runs the **RosAria** node that provides a ROS interface for PatrolBot, [4].   
* **hokuyo_node**, a package that runs the **hokuyo_node**  that provides a ROS interface for the hokuyo laser sensor, [5].
* **patbotsd**, a package that runs the following nodes:
     * **patbotsd_filter_hokuyo** node that has to do with the filtering off some laser readings that are produced by some obstacles in the robot. 
     * **patbotsd_laser_asus** node that has to do with the combination of 2D laser and 2D depth asus readings.
     * **patbotsd_map_maker_laser_asus_rotation** node that has to do with the self driving of the PatBot in the empty space of the indoor environment while making a 2D map based on the  laser and the camera/depth/points readings. 
     * **patbotsd_map_maker_laser_asus_directions** node that has to do also with the safe driving of the robot on the empty space of the area. 
     * **patbotsd_map_maker_laser_asus_range_cones** node that has to do also with the safe driving of the robot on the empty space of the area.
     * **patbotsd_asus_groundfloor_remove** node that has to do with the removing of the ground_floor of the 3D "/camera/depth/points" PointCloud2 asus topic. 
      
  

      






## Functionality

The system operates in two different modes: 

**Mode 1:**  The main idea  is to let the PatrolBot to navigate forward till an obstacle boundary is reached, then the mobile robot (MR) turns to the left with a small angle till it finds space to move forward. It keeps rotating and moving forward till a full rotation takes place. After that, it starts turning to the right and moving forward till a full rotation also takes place. This process repeats constantly and by doing so, the MR covers all the empty space of the area. The **mode 1** can be launched by the file  **roslaunch patbotsd patbotsd_navigation.launch**. This mode is suitable for a open area where the robot can move freely in the middle. 




 



<a href="http://www.youtube.com/watch?feature=player_embedded&v=y8OI2HpYXLQ&feature=youtu.be" target="_blank"><img src="http://i3.ytimg.com/vi/y8OI2HpYXLQ/hqdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>


**Mode 2:** The main idea is to place range cones along the range laser which scans the front part of the robot by 180 degrees. To this end,  a total of 5 range cones were placed and distributed in front of the robot. Then the aim is to find whether they are empty to allow the robot to navigate. The **mode 2** is launched by the  file **roslaunch patbotsd patbotsd_directions_navigation.launch**. This mode was tested in a bit cluttered corridor and with some people around.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=fUunIjcg0NE&feature=youtu.be" target="_blank"><img src="http://i3.ytimg.com/vi/fUunIjcg0NE/hqdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>

The **mode 2** was modified by adding two more range cones along the range laser making a total of 7 ones. The source code can be found in the node **patbotsd_map_maker_laser_asus_range_cones**.
<a href="https://www.youtube.com/watch?v=HpYqldQJJ2Q&feature=youtu.be" target="_blank"><img src="http://i3.ytimg.com/vi/HpYqldQJJ2Q/hqdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>

In order to extend the functionality of the application,  a mapping bagpack solution was mounted on the robot. This  solution is able to make a 3D map of the environment by means of two  Lidars which are placed on the top with a certain inclination to cover the whole area.  For more information about the 3D bagpack refer to [GitHub Pages](https://github.com/robofit/but_velodyne_lib).

<a href="https://www.youtube.com/watch?v=udxwwbGjEiU&feature=youtu.be" target="_blank"><img src="http://i3.ytimg.com/vi/udxwwbGjEiU/hqdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>



## Instalation instructions:

*  cd ~/ros/catkin_ws/src 
*  git clone https://github.com/robofit/patbot_self_driving.git 
*  cd ../ 
*  catkin_make 


## Run simulation:

*  roslaunch patbotsd patbotsd_navigation.launch
*  roslaunch patbotsd patbotsd_directions_navigation.launch


# Bibliogrhapy



 [1]  ROS. gmapping. [Online]. Available: http://wiki.ros.org/gmapping.
 
[2] ——. openni launch. [Online]. Available: http://wiki.ros.org/openni_launch.

[3] ——. pointcloud to laserscan. [Online]. Available: http://wiki.ros.org/pointcloud_to_laserscan.

[4] ——. RosAria node. [Online]. Available: http://wiki.ros.org/ROSARIA.

[5] ——. hokuyo node. [Online]. Available: http://wiki.ros.org/hokuyo_node.




<!--
Once the launch the file  **roslaunch patbotsd patbotsd_navigation.launch** or **roslaunch patbotsd patbotsd_directions_navigation.launch** are launched, the following process takes place:  It runs the packages **joint_state_publisher** and  **robot_state_publisher** which in turn runs the **joint_state_publisher** and  **robot_state_publisher** nodes respectively. These nodes  have to do with  the publishing of all the join states and their transform tree (tf) of the states of the robot. Then, it establishes communication between the laptop-PatrolBot, laptop-laser and laptop-asus by running the **RosAria**, **hokuyo_node** nodes and the **openni.launch** launch file. Moreover, the **patbotsd_filter_hokuyo** node takes as an input  the **\scan** topic from the hokuyo laser and gives as an output the **scan_filter** topic which contain the filtered **\scan** ranges.  Furthermore, the **patbotsd_asus_groundfloor_remove** node takes as an input the the **/camera/points** topic that contains the PointCloud2 from the asus sensor and gives as output the **\PC2_asus_cut_image** topic which contains the PointCloud2 with the ground floor removed. Then, the **pointcloud_to_laserscan** node takes as an input the **\PC2_asus_cut_image** topic and delivers the   **\scan_xtion** topic which contain a LaserScan message. 


The **patbotsd_laser_asus** node has as an input the **\scan_filter** and the **\scan_xtion** topics which are synchronized and combined in a single LaserScan message and published in the **\scan_laser_asus** topic. Then, either the  **patbotsd_map_maker_laser_asus_rotation** or **patbotsd_map_maker_laser_asus_directions** nodes take as an input the **\scan_laser_asus** topic and drives the PatrolBot in the empty space of the indoor environment while the **gmapping_node** is making a  map on a 2D grid. -->