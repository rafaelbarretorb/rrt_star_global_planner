cmake_minimum_required(VERSION 2.8.3)
project(rrt_star_global_planner)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(global_planner_lib src/RRTstar_ros.cpp)

add_library(${PROJECT_NAME}
   src/RRTstar_ros.cpp
)


