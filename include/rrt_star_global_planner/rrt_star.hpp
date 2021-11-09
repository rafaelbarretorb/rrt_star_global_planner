

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

#include <utility>
#include <vector>

#include "rrt_star_global_planner/node.hpp"

namespace rrt_star_global_planner {

class RRTStar {
 public:
  RRTStar(const std::pair<float, float> &start_point,
          const std::pair<float, float> &goal_point,
          costmap_2d::Costmap2D* costmap) {}
  
 private:
  std::pair<float, float> start_point_;
  std::pair<float, float> goal_point_;
  std::vector<Node> nodes_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
