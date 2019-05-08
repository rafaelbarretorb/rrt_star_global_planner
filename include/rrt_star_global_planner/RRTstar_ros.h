/** include the libraries you need in your planner here */
/* for global path planner interface */



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
#include <boost/random.hpp>

using std::string;

#ifndef RRTSTAR_ROS_CPP
#define RRTSTAR_ROS_CPP

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


namespace RRTstar_planner 
{

  class RRTstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      RRTstarPlannerROS();
      RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /** overridden classes from interface nav_core::BaseGlobalPlanner */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
      
      float distance(float px1, float py1, float px2, float py2);
      std::pair<float, float> sampleFree();
      bool collision(float wx, float wy);
      Node getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand);
      Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes);

      /*
      * 
      */
      std::vector<Node> rewire(std::vector<Node> nodes, Node newnode);
      std::pair<float, float> steer(float x1, float y1, float x2, float y2);
      bool obstacleFree(Node node_nearest, float px, float py);
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
      //bool initialized_, allow_unknown_;

    private:
      void mapToWorld(int mx, int my, float& wx, float& wy);
      void worldToMap(float wx, float wy, int& mx, int& my);

      float originX;
      float originY;
      float resolution;
      //double step_size_, min_dist_from_robot_;
      //base_local_planner::WorldModel* world_model_;
      bool initialized_;
      int width;
      int height;

  };
} // RRTstar_planner
#endif