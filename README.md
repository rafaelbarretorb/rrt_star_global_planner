This is a Global Planner Plugin that implements the RRT* (Rapidly-exploring Random Tree STAR) Path Planning Algorithm.

I followed the Tutorial [Writing A Global Path Planner As Plugin in ROS] (http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)


If you decided test this plugin using Husky Robot Simulator you can install it with following instructions:

 cd ~/catkin_ws/src   
***%  comment your catkin workspace name here is mine is (catkin_ws)*** 
$ git clone https://github.com/husky/husky.git
$ cd ..
$ catkin_make
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
$ sudo apt-get install ros-kinetic-multimaster-launch
$ sudo apt-get install ros-kinetic-lms1xx
$ rosdep install --from-path src --ignore-src  
$ catkin_make 
$ source devel/setup.bash

In the launch file responsible for run the move_base node and related configuration files, to load this plugin after all steps you have to insert the value of the parameter "base_global_planner" to "RRTstar_planner/RRTstarPlannerROS".


  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">

    <param name="base_global_planner" value="RRTstar_planner/RRTstarPlannerROS"/>
    <param name="base_local_planner" value="$(arg base_local_planner)"/>
    <rosparam file="$(find husky_navigation)/config/planner.yaml" command="load"/> 

In the case of the Husky packages, this launch file is "move_base.launch" and is located in "~/catkin_ws/src/husky/husky_navigation/launch/move_base.launch" of the package husky_navigation.
