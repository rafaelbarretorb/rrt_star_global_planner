

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

/** standard libraries **/
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>  // std::pair

#include "rrt_star_global_planner/random_double_generator.hpp"
#include "rrt_star_global_planner/node.hpp"

namespace rrt_star_global_planner {

class RRTStar {
 public:
  RRTStar(const std::pair<float, float> &start_point,
          const std::pair<float, float> &goal_point,
          costmap_2d::Costmap2D* costmap) {
    // Set range
    random_double_.setRange(-map_width_, map_width_);
  }

  std::pair<float, float> sampleFree() {
    std::pair<float, float> random_point;
    random_point.first = random_double_.generate();
    random_point.second = random_double_.generate();

    return random_point;
  }

  bool collision(float wx, float wy) {
    return false;
  }

  bool obstacleFree(const Node &node, float px, float py) {
    return true;
  }

  bool obstacleFree(const Node &node1, const Node &node2) {
    return true;
  }

 private:
  std::pair<float, float> start_point_;
  std::pair<float, float> goal_point_;
  std::vector<Node> nodes_;
  RandomDoubleGenerator random_double_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
