/** 
 * Rafael Barreto, 2021
 * rrt_star_planner.hpp
 *
*/

#ifndef RRT_STAR_PLANNER_HPP_
#define RRT_STAR_PLANNER_HPP_

/** include ROS libraries **/
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>


/**
 * @brief Node struct
 * 
*/
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


namespace rrt_star_global_planner {

class RRTStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
  * @brief Default constructor of the plugin
  */
  RRTStarPlanner();

  RRTStarPlanner(std::string name,
                    costmap_2d::Costmap2DROS* costmap_ros);

  /**
  * @brief  Initialization function for the PlannerCore object
  * @param  name The name of this planner
  * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
  */
  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros);

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
    
  /*
  * @brief Compute the euclidean distance (straight-line distance) between two points
  * @param px1 point 1 x
  * @param py1 point 1 y
  * @param px2 point 2 x
  * @param py2 point 2 y
  * @return the distance computed
  */
  float distance(float px1, float py1, float px2, float py2);

  /**
   * @brief it randomly samples a point in the free space of the plan
   * @return the a random point in the free space of the cartesian plane considered 
  */
  std::pair<float, float> sampleFree();

  /**
   * @brief Check if there is a collision at the world point (wx, wy)
   * @param wx world x coordinate (cartesian system)
   * @param wy world y coordinate (cartesian system)
   * @return True is the point collides and false otherwise
  */
  bool collision(float wx, float wy);

  /**
   * @brief Given the nodes set and an point the function returns the closest node of the node
   * @param nodes the set of nodes
   * @param p_rand the random point (x,y) in the plane
   * return the closest node
  */
  Node getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand);

  /**
   * @brief Select the best parent. Check if there is any node around the newnode with cost less than its parent node cost. 
   * If yes choose this less cost node as the new parent of the newnode.
   * @param nn the parent of the newnode
   * @param newnode the node that will checked if there is a better parent for it
   * @param nodes the set of nodes
   * @return the same newnode with the best parent node
   * 
  */
  Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes);

  /*
  * @brief The function checks if the cost of the parents of all nodes around is still less than the newnode. 
  * If there is a node with parent with higher cost the new parent of this node is the newnode now.
  * 
  * @param nodes the set of nodes
  * @param newnode the newnode
  * @return the nodes set rewired
  */
  std::vector<Node> rewire(std::vector<Node> nodes, Node newnode);

  /*
    * @brief The function generate the new point between the epsilon_min and epsilon_max along the line p_rand and nearest node. 
    *        This new point is a node candidate. It will a node if there is no obstacles between its nearest node and itself.
    * @param px1 point 1 x
    * @param py1 point 1 y
    * @param px2 point 2 x
    * @param py2 point 2 y
  * @return the new point
  */
  std::pair<float, float> steer(float x1, float y1, float x2, float y2);

  bool obstacleFree(Node node_nearest, float px, float py);

  /**
   * @brief Check if the distance between the goal and the newnode is less than the GOAL_RADIUS. If yes the newnode is the goal.
   * @param px1 point 1 x
   * @param py1 point 1 y
   * @param px2 point 2 x
   * @param py2 point 2 y
   * *@return True if distance is less than the xy tolerance (GOAL_RADIUS), False otherwise
  */
  bool pointCircleCollision(float x1, float y1, float x2, float y2, float radius);

    float XDIM;
    float YDIM;
    int MAX_NUM_NODES;
    float RADIUS;
    float GOAL_RADIUS;
    float epsilon_min;
    float epsilon_max;
  protected:

    /**
    * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
    */
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::string frame_id_;
    ros::Publisher plan_pub_;
    
    // TODO
    //allow_unknown_;

 private:
  /** 
   * @brief Convert from Map (matrix type) coordinates (mx = column and my = row) to World
   */
  void mapToWorld(int mx, int my, float& wx, float& wy);

  /** 
   * @brief Convert from Map (matrix type) coordinates (mx = column and my = row) to World
   */
  void worldToMap(float wx, float wy, int& mx, int& my);

  float originX;
  float originY;
  float resolution;
  //double step_size_, min_dist_from_robot_;
  //base_local_planner::WorldModel* world_model_;
  bool initialized_;
  int width;
  int height;

  std::vector<Node> nodes_;

};
}; // RRTstar_planner namespace
#endif