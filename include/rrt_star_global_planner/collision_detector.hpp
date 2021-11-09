#ifndef RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_

#include <costmap_2d/costmap_2d.h>

#include "rrt_star_global_planner/node.hpp"

namespace rrt_star_global_planner {

class CollisionDetector {
 public:
  explicit CollisionDetector(costmap_2d::Costmap2D* costmap);

  bool isThisPointCollides(int px, int py);

  // line between node and point
  bool isThisLineObstacleFree(const Node &node, float px, float py);

  // line between two nodes
  bool isThisLineObstacleFree(const Node &node1, const Node &node2);
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_
