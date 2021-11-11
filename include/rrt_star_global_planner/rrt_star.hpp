

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

inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

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

  void createNewNode(float x, float y, int node_nearest_id) {
    Node new_node(x, y, node_count_, node_nearest_id);
    nodes_.push_back(new_node);

    if(node_nearest_id != -1) {
      // Optimize
      chooseParent(node_nearest_id);  // Select the best parent
      rewire();  // rewire
    }

    node_count_++;
  }

  void chooseParent(int node_nearest_id) {
    float cost_new_node;
    float cost_other_parent;
    float nodes_dist;

    Node parent_node = nodes_[node_nearest_id];

    Node &new_node = nodes_.back();

    for(const auto &node : nodes_) {
      if (node.node_id == new_node.node_id) break;
      // distance between node and new_node
      nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

      if (nodes_dist < radius_) {
        // current cost of new_node
        cost_new_node = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
        
        // cost if the parent is node
        cost_other_parent = node.cost + nodes_dist;

        if (cost_other_parent < cost_new_node) {
          if (obstacleFree(node, new_node.x, new_node.y)) {
            parent_node = node;
          }
        }
      }
    }

    // Update new_node cost and its new parent
    new_node.cost = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
    new_node.parent_id = parent_node.node_id;
  }

  void rewire() {
    float nodes_dist;
    float cost_node;

    Node new_node = nodes_.back();

    for(auto &node : nodes_) {
      // distance between node and new_node
      nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

      // check if node is already the parent and if node is near for optimization
      if(node != nodes_[new_node.parent_id] && nodes_dist < radius_) {
        // cost if the parent of node is new_node
        cost_node = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

        if(cost_node < node.cost && obstacleFree(node, new_node.x, new_node.y)) {
          // update the new parent of node and its new cost
          node.parent_id = new_node.node_id;
          node.cost = cost_node;
        }
      }
    }
  }

  const std::vector<Node> &getNodes() {
    return nodes_;
  }

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
