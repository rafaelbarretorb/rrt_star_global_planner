/** 
 * Rafael Barreto, 2021
 * rrt_star_planner.hpp
 *
*/

#ifndef RRT_STAR_PLANNER_HPP_
#define RRT_STAR_PLANNER_HPP_




/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>


float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

 

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

class RRTStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTStarPlanner();

  RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  std::pair<float, float> sampleFree();

  bool collision(float wx, float wy);

  Node getNearest(std::pair<float, float> p_rand);

  Node chooseParent(Node nn, Node newnode);

  void rewire(Node newnode);

  std::pair<float, float> steer(float x1, float y1, float x2, float y2);

  bool obstacleFree(Node node_nearest, float px, float py);
  
 private:
  void worldToMap(float wx, float wy, int& mx, int& my);

  bool isGoalReached(const std::pair<float, float> &p_new);

  costmap_2d::Costmap2D* costmap_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::string frame_id_;
  ros::Publisher plan_pub_;

  float origin_x_;
  float origin_y_;
  float resolution_;

  bool initialized_{false};
  int width_;
  int height_;
  int max_number_nodes_;
  int min_number_nodes_;
  float epsilon_;
  float map_width_;
  float map_height_;
  float radius_;

  std::vector<Node> nodes_;
  Node goal_node_;
  float goal_tolerance_;

};
} // rrt_star_global_planner namespace
#endif  // RRT_STAR_PLANNER_HPP_
