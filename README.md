# patbot_self_driving
##Package description:


The issue about the software is tackled directly by explaining the launch files that runs the application in the PatrolBot. They  are called **patbotsd_navigation.launch** and **patbotsd_directions_navigation.launch**. These launch files are found under **patbot_self_driving/patbotsd/launch**. And, in order to launch the application, one must type the following commands in a terminator terminal   **roslaunch patbotsd patbotsd_navigation.launch** or **roslaunch patbotsd patbotsd_directions_navigation.launch**. These launch files  run the following packages and launch files: 

*  The but_laser_rgbd_fusion package description is: 
      * **patbotsd_slam_gmapping.launch** a launch file that  runs the gmapping package which in turn launches the laser based on SLAM package  that creates a 2D occupancy grid map.
      
      
<ul>
  <li> The but_laser_rgbd_fusion package description is: </li>
    <ul>
      <li>  <span style="font-weight: bold;">patbotsd_slam_gmapping.launch</span>, a launch file that  runs the gmapping package which in turn launches the laser based on SLAM package  that creates a 2D occupancy grid map.  <li>
      <li> **filter_hokuyo** filters the sonar readings to a specified min and max ranges. </li>
      <li> laser_rgbd_pc2 converts the data readings to pc and pc2 formats. </li>
      <li> laser_rgbd_registration makes sensor data readings alignment. </li>
      <li> laser_rgbd_fusion fuses both sensor data readings. </li>
      <li style="font-weight: bold;">
    <p><span style="font-weight: bold;">Text</span></p>
  </li>
    </ul>
  </li>
  <li> The package fuses data from the environment which is obtained  by a Hokuyo laser and a RGB-D camera. </li>
  <li> It has been tested with an UTM-30LX  and an Asus Live Xtion Pro RGB-D camera. </li>
  <li> The fused readings can be depicted under RVIZ in	 "Global Options/FixedFrame/laser" option, whrere the fused data readings topic is; "\laser_xtion_sensor" </li>
</ul>


##Package parameters:
<ul>
  <li> The main parameters used in the package are listed below, the package has not been used with different parameters. </li>
  <li> filter_hokuyo </li>
     <ul>
        <li> minimum_range_ specifies the minimun desired laser range, the used value is: 0.25 [m] </li>
        <li> maximum_range_ specifies the maximum desired laser range, the used value is: 10.0 [m] </li>
     </ul>
  <li> laser_rgbd_fusion </li>
     <ul>
        <li> map_resolution_ specifies the resolution of the map, the used value is: 0.05 [m/cell] </li>
        <li>map_x_max_ defines the height of the map, the used value is: 300 [cells] </li>
         <li>  map_y_max_ defines the witdth of the map, the used value is: 300 [cells] </li>
         <li> emp specifies prior probabilistic value of a cell being empty, the used value is: [0.4]  </li>
         <li> occ  specifies prior probabilistic value of a cell being occupied, the used value is: [0.65] </li>
     </ul>
</ul>


##Instalation instructions:
<ul>
  <li> cd ~/ros/catkin_ws/src </li>
  <li> git clone https://github.com/robofit/but_sensor_fusion.git </li>
  <li> cd ../ </li>
  <li> catkin_make </li>
</ul>

##Run simulation:
<ul>
  <li> roslaunch but_laser_rgbd_fusion laser_rgbd.launch
</ul>
