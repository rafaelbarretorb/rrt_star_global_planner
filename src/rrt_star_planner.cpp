/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner {

RRTStarPlanner::RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap_ros);
}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2D* costmap,
                               std::string global_frame) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap, global_frame);
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
  if (!initialized_) {
    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.5);
    private_nh.param("radius", radius_, 1.0);
    private_nh.param("epsilon", epsilon_, 0.2);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_num_nodes", min_num_nodes_, 500);

    // TODO(Rafael) remove hard coding
    if (search_specific_area_) {
      map_width_ = 10.0;
      map_height_ = 10.0;
    } else {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    ROS_INFO("RRT* Global Planner initialized successfully.");
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {
  // clear the plan, just in case
  plan.clear();

  ROS_INFO("RRT* Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

  std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
  std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};

  planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                  goal_point,
                                                  costmap_,
                                                  goal_tolerance_,
                                                  radius_,
                                                  epsilon_,
                                                  max_num_nodes_,
                                                  min_num_nodes_,
                                                  map_width_,
                                                  map_height_));

  std::list<std::pair<float, float>> path;

  if (planner_->pathPlanning(path)) {
    ROS_INFO("RRT* Global Planner: Path found!!!!");
    computeFinalPlan(plan, path);
    return true;
  } else {
    ROS_WARN("The planner failed to find a path, choose other goal position");
    return false;
  }
}

void  RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path) {
  // clean plan
  plan.clear();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
}

}  // namespace rrt_star_global_planner
