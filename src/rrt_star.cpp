

#include "rrt_star_global_planner/rrt_star.hpp"

namespace rrt_star_global_planner {

RRTStar::RRTStar(const std::pair<float, float> &start_point,
                 const std::pair<float, float> &goal_point,
                 costmap_2d::Costmap2D* costmap) {
  // Set range
  random_double_.setRange(-map_width_, map_width_);
}

std::pair<float, float> RRTStar::sampleFree() {
  std::pair<float, float> random_point;
  random_point.first = random_double_.generate();
  random_point.second = random_double_.generate();

  return random_point;
}

bool RRTStar::collision(float wx, float wy) {
  return false;
}

bool RRTStar::obstacleFree(const Node &node, float px, float py) {
  return true;
}

bool RRTStar::obstacleFree(const Node &node1, const Node &node2) {
  return true;
}

void RRTStar::createNewNode(float x, float y, int node_nearest_id) {
  Node new_node(x, y, node_count_, node_nearest_id);
  nodes_.push_back(new_node);

  if(node_nearest_id != -1) {
    // Optimize
    chooseParent(node_nearest_id);  // Select the best parent
    rewire();  // rewire
  }

  node_count_++;
}

void RRTStar::chooseParent(int node_nearest_id) {
  float cost_new_node;
  float cost_other_parent;
  float nodes_dist;

  Node parent_node = nodes_[node_nearest_id];

  Node &new_node = nodes_.back();

  for(const auto &node : nodes_) {
    if (node.node_id == new_node.node_id) break;  // TODO
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

void RRTStar::rewire() {
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

const std::vector<Node> &RRTStar::getNodes() {
  return nodes_;
}

void RRTStar::setRadius(double radius) {
  radius_ = radius;
}

const std::list<std::pair<float, float>> &RRTStar::pathPlanning() {
  // Start Node
  createNewNode(start.pose.position.x, start.pose.position.x, -1);

  std::list<std::pair<float, float>> path; // remove

  // Add the initial Pose
  plan.push_back(start);
  
  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

  while(nodes_.size() < max_num_nodes_) {
    bool found_next = false;
    while (found_next == false) {
      p_rand = sampleFree(); // random point in the free space
      node_nearest = getNearest(p_rand); // The nearest node of the random point
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
      if (obstacleFree(node_nearest, p_new.first, p_new.second)) {
        found_next = true;
        createNewNode(p_new.first, p_new.second, node_nearest.node_id);
      }
    }

    // Check if the distance between the goal and the new node is less than the goal tolerance
    if(isGoalReached(p_new) && nodes_.size() > min_num_nodes_) {
      ROS_INFO("RRT* Global Planner: Path found!!!!");
      computeFinalPath(plan);

      return true;
    }
  }
}

}  // namespace rrt_star_global_planner
