/** include the libraries you need in your planner here */
/* for global path planner interface */

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <cmath>
#include <math.h>
#include <tuple>

using std::string;

#ifndef RRTSTAR_ROS_CPP
#define RRTSTAR_ROS_CPP

struct Node {
	float x;
  float y;
  int node_id;
	int parent_id;
  float cost;
}; 

namespace RRTstar_planner 
{

  class RRTstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {
    public:
      float XDIM;
      float YDIM;
      int MAX_NUM_NODES;

      RRTstarPlannerROS();
      RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS∗ costmap_ros);
      /** overridden classes from interface nav_core::BaseGlobalPlanner */
      void initialize(std::string name, costmap_2d::Costmap2DROS∗ costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
      
      float distance(float px1, float py1, float px2, float py2);
      std::pair<float, float> sampleFree(float x_dim, float y_dim);
      bool collision(float wx, float wy);
      Node getNearest(std::vector<Node> nodes, std::vector<float> x_rand);
      Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes);

      /*
      * 
      */
      std::vector<Node> rewire(std::vector<Node> nodes, Node newnode);
      std::pair<float, float> steer(float x1, float y1, float x2, float y2);
      bool obstacleFree(Node node_nearest, std::vector<float> x_rand);
      bool pointCircleCollision(float x1, float y1, float x2, float y2, float radius);

    protected:

      /**
      * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
      */
      costmap_2d::Costmap2D* costmap_;
      std::string frame_id_;
      ros::Publisher plan_pub_;
      bool initialized_, allow_unknown_;

    private:
      void mapToWorld(double mx, double my, double& wx, double& wy);
      bool worldToMap(double wx, double wy, double& mx, double& my);

  };
};
#endif