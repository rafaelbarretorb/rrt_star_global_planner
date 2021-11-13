

#include "rrt_star_global_planner/rrt_star.hpp"

namespace rrt_star_global_planner {

RRTStar::RRTStar(const std::pair<float, float> &start_point,
                 const std::pair<float, float> &goal_point,
                 costmap_2d::Costmap2D* costmap,
                 double goal_tolerance,
                 double radius,
                 double epsilon,
                 unsigned int max_num_nodes,
                 unsigned int min_num_nodes) : start_point_(start_point),
                                               goal_point_(goal_point),
                                               costmap_(costmap),
                                               goal_tolerance_(goal_tolerance),
                                               radius_(radius),
                                               epsilon_(epsilon),
                                               max_num_nodes_(max_num_nodes),
                                               min_num_nodes_(min_num_nodes) {
  // Set range
  random_double_.setRange(-map_width_, map_width_);
}

std::pair<float, float> RRTStar::sampleFree() {
  std::pair<float, float> random_point;
  random_point.first = random_double_.generate();
  random_point.second = random_double_.generate();

  return random_point;
}

int RRTStar::getNearestNodeId(const std::pair<float, float> &p_rand) {
  
  float dist_nearest, dist;
  Node node_nearest = nodes_[0];

  // TODO range loop
  for (int i = 1; i < nodes_.size(); ++i) {
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, p_rand.first, p_rand.second);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

  return node_nearest.node_id;
}

bool RRTStar::collision(float wx, float wy) {
  return false;
}

bool RRTStar::obstacleFree(const Node &node_nearest, float px, float py) {
  int n = 1;
  float theta;

  std::pair<float, float> p_n;
  p_n.first = 0.0;
  p_n.second = 0.0;

  float dist = euclideanDistance2D(node_nearest.x, node_nearest.y, px, py);
  if (dist < resolution_) {
    if (collision(px, py))
      return false;
    else
      return true;
  } else {
    int value = int(floor(dist/resolution_));
    float theta;
    for (int i = 0; i < value; i++) {
      theta = atan2(node_nearest.y - py, node_nearest.x - px);
      p_n.first = node_nearest.x + n*resolution_*cos(theta);
      p_n.second = node_nearest.y + n*resolution_*sin(theta);
      if (collision(p_n.first, p_n.second))
        return false;
      
      n++;
    }
    return true;
  }
}

bool RRTStar::obstacleFree(const Node &node1, const Node &node2) {
  obstacleFree(node1, node2.x, node2.y);
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

// TODO change parameters name
std::pair<float, float> RRTStar::steer(float x1, float y1, float x2, float y2) {
  std::pair<float, float> p_new;
  float dist = euclideanDistance2D(x1, y1, x2, y2);
  if (dist < epsilon_) {
    p_new.first = x1;
    p_new.second = y1;
    return p_new;
  } else {
    float theta = atan2(y2-y1, x2-x1);
    p_new.first = x1 + epsilon_*cos(theta);
    p_new.second = y1 + epsilon_*sin(theta);
    return p_new;
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
  createNewNode(start_point_.first, start_point_.second, -1);

  std::list<std::pair<float, float>> path;

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

  while(nodes_.size() < max_num_nodes_) {
    bool found_next = false;
    while (found_next == false) {
      p_rand = sampleFree(); // random point in the free space
      node_nearest = nodes_[getNearestNodeId(p_rand)]; // The nearest node of the random point
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
      if (obstacleFree(node_nearest, p_new.first, p_new.second)) {
        found_next = true;
        createNewNode(p_new.first, p_new.second, node_nearest.node_id);
      }
    }

    // Check if the distance between the goal and the new node is less than the goal tolerance
    if(isGoalReached(p_new) && nodes_.size() > min_num_nodes_) {
      computeFinalPath(path);

      return path;
    }
  }
}

void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path) {
  path.clear();

  // New goal inside of the goal tolerance
  Node current_node = nodes_.back();

  // Final Path
  std::pair<float, float> point;

  do {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);

    // update the current node
    current_node = nodes_[current_node.parent_id];
  } while (current_node.parent_id != -1);
}

bool RRTStar::isGoalReached(const std::pair<float, float> &p_new) {
  return (euclideanDistance2D(p_new.first,
                              p_new.second,
                              goal_point_.first,
                              goal_point_.second) < goal_tolerance_) ? true : false;
}

}  // namespace rrt_star_global_planner