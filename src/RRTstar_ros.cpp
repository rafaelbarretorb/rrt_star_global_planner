/*
  planner_core.h
  RRTstar_ros.cpp

*/

#include <rrt_star_global_planner/RRTstar_ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

std::random_device rd;
static std::default_random_engine generator ( rd() );

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRTstar_planner::RRTstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace RRTstar_planner
{

  RRTstarPlannerROS::RRTstarPlannerROS() :
          costmap_(NULL), initialized_(false), allow_unknown_(true) {
  }

  RRTstarPlannerROS::RRTstarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
          costmap_(NULL), initialized_(false), allow_unknown_(true) 
  {
      //initialize the planner
      initialize(name, costmap, frame_id);
  }

  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
  {
      initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS∗ costmap_ros)
  {

    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
  
      ros::NodeHandle private_nh("~/" + name);
  
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();
	    width = costmap_->getSizeInCellsX();
	    height = costmap_->getSizeInCellsY();
	    resolution = costmap_->getResolution();
	    mapSize = width*height;
	    tBreak = 1+1/(mapSize); // ????
	    value = 0;

      RADIUS = 1.0;
      GOAL_RADIUS = 0.5;
      epsilon_min = 0.05;
      epsilon_max = 0.1;


      ROS_INFO("RRT* planner initialized successfully");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    std::vector<Node> nodes;
    //float x_dim = this->XDIM;
    //float y_dim = this->YDIM;

    float x_dim = 20.0;
    float y_dim = 20.0;

    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;

    nodes.push_back(start_node);
    
    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;

    Node node_nearest;
    while (nodes.size() < MAX_NUM_NODES)
    {
      bool found_next = false;
      while (found_next == false)
      {
        p_rand = sampleFree(x_dim, y_dim); // random point in the free space
        node_nearest = getNearest(nodes, x_rand); // The nearest node of the random point
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
        if (obstacleFree(node_nearest, p_new))
        {
          Node newnode;
          newnode.x = x_new.first;
          newnode.y = x_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimize
          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent
          nodes.push_back(newnode);
          nodes = rewire(nodes, newnode); 
          found_next = true;
        }
      }
      // Check if the distance between the goal and the new node is less than
      // the GOAL_RADIUS
      if (pointCircleCollision(p_new.first, p_new.second, goal.pose.position.x , goal.pose.position.y, GOAL_RADIUS) && nodes.size() > 5000)
      {
        std::vector<std::pair<float, float>> path;
        std::pair<float, float> point;
        
        // New goal inside of the goal tolerance
        Node new_goal_node = nodes[nodes.size() - 1];
        Node current_node = new_goal_node;

        current_node = new_goal_node;
        // Final Path
        while (current_node.parent_id != -1)
        {
          point.first = current_node.x;
          point.second = current_node.y;
          path.insert(path.begin(), point); 
      
          current_node = nodes[current_node.parent_id];
        }

      }
    }

    //if the global planner find a path
    if (path.size() > 0)
    {
      ros::Time plan_time = ros::Time::now();
      // convert the points to poses
      for (int i = 0; i < path.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = path[i].first;
        pose.pose.position.y = path[i].second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
      }
    }
    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }
  }

  float RRTstarPlannerROS::distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
    return dist;
  }

  std::pair<float, float> RRTstarPlannerROS::sampleFree(float x_dim, float y_dim)
  {
    float MAX_RANDOM_NUMBER = 1.0;

    float x = (distribution(generator) - MAX_RANDOM_NUMBER/2)*x_dim;
    float y = (distribution(generator) - MAX_RANDOM_NUMBER/2)*y_dim;
    std::pair<float, float> p_rand;
    p_rand.first = x;
    p_rand.second = y;
    return p_rand; 
  }

  void RRTstarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) 
  {
      wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
      wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
  }

bool RRTstarPlannerROS::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

  //check if point collides with the obstacle
  bool RRTstarPlannerROS::collision(float wx, float wy)
  {
    int mx, my;
    worldToMap(wx, wy, mx, my);

    if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
      return true;

    // grid[row][column] = vector[row*WIDTH + column]
    if (costmap_[my*this->width + mx] > 0)
      return true;

    return false;
  }
  
  Node RRTstarPlannerROS::getNearest(std::vector<Node> nodes, std::vector<float> x_rand)
  {
    Node node = nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
      if (this->distance(nodes[i].x, nodes[i].y, x_rand[0], x_rand[1]) < this->distance(node.x, node.y, x_rand[0], x_rand[1]))
        node = nodes[i];
    }
  
    return node;
  }

  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {
    for (int i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < RADIUS &&
         nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y) &&
         obstacleFree(nodes[i], nn.x, nn.y))
      {
        nn = nodes[i];
      }
      newnode.cost = nn.cost + this->distance(nn.x, nn.y, newnode.x, newnode.y);
      newnode.parent_id = nn.node_id;
    }
  
    return newnode;
  }

  std::vector<Node> RRTstarPlannerROS::rewire(std::vector<Node> nodes, Node newnode)
  {
    Node node;
    for (int i = 0; i < nodes.size(); i++)
    {
      node = nodes[i]
      if (node != this->getParent(newnode) &&
          this->distance(node.x, node.y, newnode.x, newnode.y) < RADIUS &&
          newnode.cost + this->distance(node.x, node.y, newnode.x, newnode.y) < node.cost)
      {
        node.parent = newnode.node_id;
        node.cost = newnode.cost + this->distance(node.x, node.y, newnode.x, newnode.y);
      }
    }
    return nodes;
  }

  std::pair<float, float> RRTstarPlannerROS::steer(float x1, float y1, float x2, float y2)
  {
    std::pair<float, float> p_new;
    float dist = this->distance(x1, y1, x2, y2);
    if (dist < this->epsilon_max && dist > this->epsilon_min)
    {
      p_new.first = x1;
      p_new.second = y1;
      return p_new;
    }
    else
    {
      float theta = atan2(y2-y1, x2-x1);
      p_new.first = x1 + this->epsilon_max*cos(theta);
      p_new.second = y1 + this->epsilon_max*sin(theta);
      return p_new;
    }
  }

  bool RRTstarPlannerROS::obstacleFree(Node node_nearest, std::vector<float> x_rand)
  {
    int n = 1;
    float = theta;
    std::vector<float> x_n = {0.0, 0.0};
    float dist = this->distance(node_nearest.x, node_nearest.y, x_rand[0], x_rand[1]);
    if (dist < this->obs_resolution)
    {
      if (collision(x_rand))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist/this->obs_resolution));
      float theta;
      for (int i = 0;i < value; i++)
      {
        theta = atan2(node_nearest.y - x_rand[1], node_nearest.x - x_rand[0]);
        x_n[0] = node_nearest.x + n*this->obs_resolution*cos(theta);
        x_n[1] = node_nearest.y + n*this->obs_resolution*sin(theta);
        if (this->collision(x_n))
          return false;
        
        n++;
      }
      return true;
    }
  }
} // RRTstar_planner namespace