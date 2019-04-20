

#include <rrt_star_global_planner/RRTstar_ros.h>

std::random_device rd;
static std::default_random_engine generator ( rd() );

namespace RRTstar_planner
{

  RRTstarPlannerROS::RRTstarPlannerROS()
  {

  }

  void RRTstarPlannerROS::initialize()
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
	    tBreak = 1+1/(mapSize); 
	    value = 0;


	    OGM = new bool [mapSize]; 
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
      {
        for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
          //cout<<cost;
          if (cost == 0)
            OGM[iy*width+ix]=true;
          else
            OGM[iy*width+ix]=false;
        }
      }

      ROS_INFO("RAstar planner initialized successfully");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTstarPlannerROS::makePlan()
  {
    float x_dim = this -> XDIM;
    float y_dim = this -> YDIM;
    while (nodes.size() < this -> max_num_nodes)
    {
      bool found_next = false;
      while (found_next == false)
      {
        std::vector<float> x_rand = this -> sampleFree(x_dim, y_dim);
        Node x_nearest = this -> getNearest(nodes, x_rand);
        x_new = this -> steer(x_nearest.point, x_rand)
      }
    }







    //if the global planner find a path
    if ( bestPath.size()>0)
    {
    
      // convert the path
    
      for (int i = 0; i < bestPath.size(); i++)
      {
      
        float x = 0.0;
        float y = 0.0;
      
        int index = bestPath[i];
      
        convertToCoordinate(index, x, y);
      
        geometry_msgs::PoseStamped pose = goal;
      
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
      
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
      
        plan.push_back(pose);
      }
    }



  }


  std::vector<float> RRTstarPlannerROS::sampleFree(float x_dim, float y_dim)
  {
    float MAX_RANDOM_NUMBER = 1.0;

    float x = (distribution(generator) - MAX_RANDOM_NUMBER/2)*x_dim;
    float y = (distribution(generator) - MAX_RANDOM_NUMBER/2)*y_dim;
    std::vector<float> x_rand = {x,y};
    return x_rand; 
  }