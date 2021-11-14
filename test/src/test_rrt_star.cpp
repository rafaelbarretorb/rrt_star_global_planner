/*
  Copyright 2021 - Rafael Barreto
*/

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>

#include <utility>

#include "rrt_star_global_planner/rrt_star.hpp"
#include "rrt_star_global_planner/node.hpp"

using rrt_star_global_planner::RRTStar;
using rrt_star_global_planner::Node;

int g_argc;
char** g_argv;

class RRTStarTest : public ::testing::Test {
 public:
  RRTStarTest() {}

  virtual ~RRTStarTest() {}

  void SetUp() override {
    std::pair<float, float> start = {0., 0.};
    std::pair<float, float> goal = {5., 5.};
    rrt_star = new RRTStar(start,
                           goal,
                           nullptr,
                           0.2,
                           1.3,
                           0.2,
                           10000,
                           1000,
                           10.0,
                           10.0);

    rrt_star->createNewNode(0.5, 0.5, -1);  // 0
    rrt_star->createNewNode(-0.5, 1.5, 0);  // 1
    rrt_star->createNewNode(1.0, 3.0, 1);  // 2
    rrt_star->createNewNode(3.0, 3.8, 2);  // 3 - nearest of new node
    rrt_star->createNewNode(3.5, 7.0, 3);  // 4
    rrt_star->createNewNode(2.2, 2.0, 0);  // 5 - low cost near of new node
    rrt_star->createNewNode(3.0, 3.0, 3);  // 6 - new node
  }

  void TearDown() override {
    delete rrt_star;
    rrt_star = nullptr;
  }

  RRTStar *rrt_star{nullptr};
};

TEST_F(RRTStarTest, ChooseTheBestParentForTheNewNode) {
  const std::vector<Node> &nodes = rrt_star->getNodes();
  EXPECT_EQ(nodes.back().parent_id, 5);
}

TEST_F(RRTStarTest, RewireNodesAroundNewNode) {
  const std::vector<Node> &nodes = rrt_star->getNodes();
  EXPECT_EQ(nodes[3].parent_id, 6);
}

TEST_F(RRTStarTest, NewNodeFinalCost) {
  std::vector<Node> nodes = rrt_star->getNodes();
  EXPECT_NEAR(nodes.back().cost, 3.5478, 0.001);
}

TEST_F(RRTStarTest, NearestNodeFinalCost) {
  const std::vector<Node> &nodes = rrt_star->getNodes();
  EXPECT_NEAR(nodes[3].cost, 4.3478, 0.001);
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  ros::init(g_argc, g_argv, "test");
  ros::Time::init();
  return RUN_ALL_TESTS();
}
