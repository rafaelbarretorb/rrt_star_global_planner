/*

*/
#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_


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
          costmap_2d::Costmap2D* costmap,
          double goal_tolerance,
          double radius,
          double epsilon,
          unsigned int max_num_nodes,
          unsigned int min_num_nodes,
          float map_width,
          float map_height
          );

  std::pair<float, float> sampleFree();

  int getNearestNodeId(const std::pair<float, float> &p_rand);

  bool collision(float wx, float wy);

  bool obstacleFree(const Node &node, float px, float py);

  bool obstacleFree(const Node &node1, const Node &node2);

  void createNewNode(float x, float y, int node_nearest_id);

  void chooseParent(int node_nearest_id);

  void rewire();

  // TODO change parameters name
  std::pair<float, float> steer(float x1, float y1, float x2, float y2);

  const std::vector<Node> &getNodes();

  void setRadius(double radius);

  void computeFinalPath();

  const std::list<std::pair<float, float>> &pathPlanning();

  bool isGoalReached(const std::pair<float, float> &p_new);

  void worldToMap(float wx, float wy, int& mx, int& my);

 private:
  std::pair<float, float> start_point_;
  std::pair<float, float> goal_point_;
  costmap_2d::Costmap2D* costmap_{nullptr};
  std::vector<Node> nodes_;
  RandomDoubleGenerator random_double_;
  int node_count_{0};
  float map_width_;
  float map_height_;
  double radius_{1.3};  // TODO remove this initialization
  unsigned int max_num_nodes_;
  unsigned int min_num_nodes_;
  double goal_tolerance_;
  double epsilon_;
  float resolution_;
  float origin_x_;
  float origin_y_;
  std::list<std::pair<float, float>> path_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_
