This is a ROS Global Planner Plugin that implements the RRT* (Rapidly-exploring Random Tree Star) Path Planning Algorithm. I tried to keep the code simple to help everyone that is learning ROS and the Navigation Stack. I had several difficulties learning how to write my first Global Planner Plugin and I hope this package helps you to learn it faster.

For better performance, fork this repository and improve your own global planner plugin implementing newer variations of the RRT* algorithm as, for example, the RRT*-Smart or Dual-Tree RRT*-Smart. I implemented these algorithms in this repository [Path-Planning-Pygame](https://github.com/rafaelbarretorb/Path-Planning-Pygame) using Python and they reach low-cost paths much faster.

I followed the Tutorial [Writing A Global Path Planner As Plugin in ROS](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS) and study the source code of the global_planner package of the [Navigation Stack](https://github.com/ros-planning/navigation) and a few other plugin packages available at github to develop this package.

I tested in Kinetic and Melodic Distros. If you decided to test this plugin using Husky Robot Simulator you can install it with the following instructions:

```
$ cd ~/catkin_ws/src   
$ git clone --branch melodic https://github.com/husky/husky.git
$ cd ..
$ catkin_make
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
$ sudo apt-get install ros-melodic-multimaster-launch
$ sudo apt-get install ros-melodic-lms1xx
$ rosdep install --from-path src --ignore-src  
$ catkin_make 
$ source devel/setup.bash

```
After following all steps of the sessions 1 and 2  of the Tutorial:

**1. Writing the Path Planner Class**

**2. Writing your Plugin**

You have to run the plugin.

**3. Running the Plugin on the Husky Robot using Gazebo Simulator**

In the launch file responsible for run the move_base node and related configuration files, to load this plugin after all steps of the Tutorial you have to insert the value of the parameter "base_global_planner" to "RRTstar_planner/RRTStarPlanner". In the case of the Husky packages, this launch file is "move_base.launch" and is located at my catkin workspace called catkin_ws at "~/catkin_ws/src/husky/husky_navigation/launch/move_base.launch" inside of the package husky_navigation.

```
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_global_planner" value="RRTstar_planner/RRTStarPlanner"/>
    <param name="base_local_planner" value="$(arg base_local_planner)"/>
    <rosparam file="$(find husky_navigation)/config/planner.yaml" command="load"/> 
```
**4. Testing the planner with GAZEBO Simulator and RVIZ**

In three separate terminals, execute these three launch commands:

```
roslaunch husky_gazebo husky_playpen.launch
roslaunch husky_navigation amcl_demo.launch
roslaunch husky_viz view_robot.launch
```
The amcl_demo.launch file launchs the move_base.launch edited above and the all navigation packages. The view_robot.launch run the RViz with the proper topics configuration. 

Lastly, on the left side at "Display" part, change the name of the topic related to Global Planner to visualize it. If you using the DWA Local Planner the topic name is "/move_base/DWAPlannerROS/global_plan".

To test the plugin just click on "2D Nav Goal" button (at the top) and choose a goal location. You can now see your robot moving to its goal
if everything is fine.
