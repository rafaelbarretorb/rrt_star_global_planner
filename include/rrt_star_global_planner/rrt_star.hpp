/*

*/
#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

/** standard libraries **/
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>  // std::pair

#include "rrt_star_global_planner/random_double_generator.hpp"
#include "rrt_star_global_planner/node.hpp"

namespace rrt_star_global_planner {

inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

class RRTStar {
 public:
  RRTStar(const std::pair<float, float> &start_point,
          const std::pair<float, float> &goal_point,
          costmap_2d::Costmap2D* costmap);

  std::pair<float, float> sampleFree();

  bool collision(float wx, float wy);

  bool obstacleFree(const Node &node, float px, float py);

  bool obstacleFree(const Node &node1, const Node &node2);

  void createNewNode(float x, float y, int node_nearest_id);

  void chooseParent(int node_nearest_id);

  void rewire();

  const std::vector<Node> &getNodes();

  void setRadius(double radius);


  void computeFinalPath();

  const std::list<std::pair<float, float>> &pathPlanning();

 private:
  std::pair<float, float> start_point_;
  std::pair<float, float> goal_point_;
  std::vector<Node> nodes_;
  RandomDoubleGenerator random_double_;
  int node_count_{0};
  float map_width_;
  float map_height_;
  double radius_{1.3};
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
