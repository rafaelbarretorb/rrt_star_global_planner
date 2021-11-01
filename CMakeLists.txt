cmake_minimum_required(VERSION 3.1.3)
project(rrt_star_global_planner)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
  geometry_msgs
  angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    pluginlib
    nav_core
    base_local_planner
    geometry_msgs
    angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/rrt_star_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  FILES rrt_star_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
endif()
