#ifndef RRT_STAR_GLOBAL_PLANNER_NODE_HPP_
#define RRT_STAR_GLOBAL_PLANNER_NODE_HPP_

namespace rrt_star_global_planner {

struct Node {
	float x;
  float y;
  int node_id;
	int parent_id;
  float cost{0.0};

  Node() {}

  Node(float px, float py, int node_index, int parent_index) : x(px),
                                                       y(py),
                                                       node_id(node_index),
                                                       parent_id(parent_index) {}
   
  bool operator ==(const Node& node) { return node_id == node.node_id; }

  bool operator !=(const Node& node) { return !(node_id == node.node_id); }
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_NODE_HPP_
