cmake_minimum_required(VERSION 3.2)
project(rrt_star_global_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
  geometry_msgs
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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rrt_star_planner.cpp
  src/rrt_star.cpp
  src/collision_detector.cpp
  src/random_double_generator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  FILES rrt_star_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(cmake_modules)
  enable_testing()
  find_package(rostest REQUIRED)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_rostest_gtest(test_rrt_star
    test/rrt_star.test
    test/src/test_rrt_star.cpp
  )

  target_link_libraries(test_rrt_star
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    gtest
  )

endif()
