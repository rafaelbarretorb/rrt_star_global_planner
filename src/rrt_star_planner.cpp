/*
  Rafael Barreto, 2021
  rrt_star_planner.cpp

*/


#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"


// TODO set size of vector nodes_(max_num_nodes_)


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner {

RRTStarPlanner::RRTStarPlanner() 
  : costmap_(NULL), initialized_(false){}

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_(NULL), initialized_(false) {
    //initialize the planner
    initialize(name, costmap_ros);
}

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
  : costmap_(NULL), initialized_(false) {
    //initialize the planner
    initialize(name, costmap, global_frame);
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
  if (!initialized_) {
    // Initialize map
    costmap_ = costmap;
    //costmap_ = costmap_ros->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.5);
    private_nh.param("radius", radius_, 1.0);
    private_nh.param("epsilon", epsilon_, 0.2);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_num_nodes", min_num_nodes_, 500);

    // TODO check this
    //world_model_.reset(new base_local_planner::CostmapModel(*costmap_));

    // TODO
    // hard coding for while
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();
    map_width_ = 10.0;
    map_height_ = 10.0;

    // Random
    random_double_.setRange(-map_width_, map_width_);

    ROS_INFO("RRT* Global Planner initialized successfully.");
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {
  //clear the plan, just in case
  plan.clear();
  node_count_ = 0;

  // Initialize the goal node
  goal_node_.x = goal.pose.position.x;
  goal_node_.y = goal.pose.position.y;

  ROS_INFO("RRT* Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

  // TODO remove this
  std::string global_frame = frame_id_;

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

  ROS_WARN("The planner failed to find a path, choose other goal position");
  return false;
}

std::pair<float, float> RRTStarPlanner::sampleFree() {
  std::pair<float, float> random_point;
  for (int i = 0; i < 10000; i++) {
    // generate random x and y coords within map bounds

    random_point.first = random_double_.generate();
    random_point.second = random_double_.generate();

    if (!collision(random_point.first, random_point.second))
      return random_point;
  }
  // TODO
  // ROS_ERROR() not point found

  return random_point;
}

bool RRTStarPlanner::collision(float wx, float wy) {
  // TODO check this method
  int mx, my;
  worldToMap(wx, wy, mx, my);

  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;

  // TODO static_cast? check this 
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));
  if (cost > 0)
    return true;
  
  return false;
}

Node RRTStarPlanner::getNearest(const std::pair<float, float> &p_rand) {
  Node node_nearest = nodes_[0];

  float dist_nearest, dist;
  for (int i = 1; i < nodes_.size(); ++i) {
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, p_rand.first, p_rand.second);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

  return node_nearest;
}

void RRTStarPlanner::chooseParent(int node_nearest_id) {
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

void RRTStarPlanner::rewire() {
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
std::pair<float, float> RRTStarPlanner::steer(float x1, float y1, float x2, float y2) {
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

bool RRTStarPlanner::obstacleFree(const Node &node_nearest, float px, float py) {
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

void RRTStarPlanner::worldToMap(float wx, float wy, int& mx, int& my) {
  mx = (wx - origin_x_) / resolution_;
  my = (wy - origin_y_) / resolution_;
}

bool RRTStarPlanner::isGoalReached(const std::pair<float, float> &p_new) {
  return (euclideanDistance2D(p_new.first,
                              p_new.second,
                              goal_node_.x,
                              goal_node_.y) < goal_tolerance_) ? true : false;
}

void RRTStarPlanner::createNewNode(float x, float y, int node_nearest_id) {
  Node new_node(x, y, node_count_, node_nearest_id);
  nodes_.push_back(new_node);

  if(node_nearest_id != -1) {
    // Optimize
    chooseParent(node_nearest_id);  // Select the best parent
    rewire();  // rewire
  }

  node_count_++;
}

void RRTStarPlanner::computeFinalPath(std::vector<geometry_msgs::PoseStamped>& plan) {
  std::list<std::pair<float, float>> path;

  // New goal inside of the goal tolerance
  goal_node_ = nodes_.back();
  Node current_node = goal_node_;

  // Final Path
  std::pair<float, float> point;
  while(current_node.parent_id != -1) {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);

    // update the current node
    current_node = nodes_[current_node.parent_id];
  }
  
  //if the global planner find a path
  if(path.size() > 0) {
    ros::Time plan_time = ros::Time::now();
    // convert points to poses
    for(auto p : path) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = "map";  // TODO remove hard coding
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
  }
}
}  // RRTstar_planner namespace
