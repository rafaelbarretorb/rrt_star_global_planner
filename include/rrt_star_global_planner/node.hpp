#ifndef NODE_HPP_
#define NODE_HPP_

namespace rrt_star_global_planner {

struct Node {
	float x;
  float y;
  int node_id;
	int parent_id;
  float cost;
  bool operator ==(const Node& node) 
  {
	  return (x == node.x) && (y == node.y) && (node_id == node.node_id) && (parent_id == node.parent_id) && (cost == node.cost) ;
  }

  bool operator !=(const Node& node) 
  {
    if((x != node.x) || (y != node.y) || (node_id != node.node_id) || (parent_id != node.parent_id) || (cost != node.cost))
      return true;
    else
      return false;
  }
};

}  // namespace rrt_star_global_planner

#endif  // NODE_HPP_
