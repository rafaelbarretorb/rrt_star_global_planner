#include <gtest/gtest.h>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>

#include <utility>

#include "rrt_star_global_planner/rrt_star.hpp"

using rrt_star_global_planner::RRTStar;


int g_argc;
char** g_argv;

class RRTStarTest : public ::testing::Test {
 public:
  RRTStarTest() {}
  virtual ~RRTStarTest() {}

  void SetUp() override {
    std::pair<float, float> start = {0., 0.};
    std::pair<float, float> goal = {5., 5.};
    rrt_star = new RRTStar(start, goal, nullptr);
  }

  void TearDown() override {
    delete rrt_star;
    rrt_star = nullptr;
  }

  RRTStar *rrt_star{nullptr};
};

// just test the test, remove it
TEST_F(RRTStarTest, getRandomDoubles) {
  ASSERT_NE(1, 2);  // false
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  ros::init(g_argc, g_argv, "test");
  ros::Time::init();
  return RUN_ALL_TESTS();
}
