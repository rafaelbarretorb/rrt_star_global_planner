/** 
 * Rafael Barreto, 2021
 * rrt_star_planner.hpp
 *
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_

#include <ros/ros.h>

/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>  // std::pair

#include "rrt_star_global_planner/random_double_generator.hpp"
#include "rrt_star_global_planner/node.hpp"


namespace rrt_star_global_planner {

/**
 * @brief  Constructor for the RRTStarPlanner object
 * @param  name The name of this planner
 * @param  costmap A pointer to the ROS wrapper of the costmap to use
 */
inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

/**
 * @class RRTStarPlanner
 * @brief Provides a ROS rrt* planner planner
 */
class RRTStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief  Default constructor for the RRTStarPlanner object
   */
  RRTStarPlanner();

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  global_frame The global frame of the costmap
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param  global_frame The global frame of the costmap
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

 
  std::pair<float, float> sampleFree();
protected:
  bool collision(float wx, float wy);

  Node getNearest(const std::pair<float, float> &p_rand);

  void chooseParent(int node_nearest_id);

  void rewire();

  // TODO change parameters name
  std::pair<float, float> steer(float x1, float y1, float x2, float y2);

  bool obstacleFree(const Node &node_nearest, float px, float py);

  void worldToMap(float wx, float wy, int& mx, int& my);

  bool isGoalReached(const std::pair<float, float> &p_new);

  void createNewNode(float x, float y, int node_nearest_id);

  void computeFinalPath(std::vector<geometry_msgs::PoseStamped>& plan);

  costmap_2d::Costmap2D* costmap_{NULL};
  costmap_2d::Costmap2DROS* costmap_ros_{NULL};  // TODO chech to remove
  std::string frame_id_;
  ros::Publisher plan_pub_;


 private:
  float origin_x_;
  float origin_y_;
  float resolution_;

  bool initialized_{false};
  int max_num_nodes_;
  int min_num_nodes_;
  double epsilon_;
  float map_width_;
  float map_height_;
  double radius_;

  std::vector<Node> nodes_;
  Node goal_node_;
  double goal_tolerance_;
  RandomDoubleGenerator random_double_;
  int node_count_{0};

  // TODO
  //bool allow_unknown_{false};
  //boost::shared_ptr<NavFn> planner_;
};
}  // rrt_star_global_planner namespace
#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_
