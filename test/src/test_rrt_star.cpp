#include <gtest/gtest.h>
#include <ros/ros.h>


#include "rrt_star_global_planner/rrt_star_planner.hpp"

using rrt_star_global_planner::RRTStarPlanner;


int g_argc;
char** g_argv;

class RRTStarPlannerTest : public ::testing::Test {
 public:
  RRTStarPlannerTest() {}

 protected:
  RRTStarPlanner global_planner_;
};

// just test the test, remove it
TEST_F(RRTStarPlannerTest, getRandomDoubles) {
  ASSERT_NE(1, 1);  // false
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  ros::init(g_argc, g_argv, "test");
  ros::Time::init();
  return RUN_ALL_TESTS();
}