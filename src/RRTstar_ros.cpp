/*
  planner_core.h

*/

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

      ROS_INFO("RRT* planner initialized successfully");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTstarPlannerROS::makePlan()
  {
    float x_dim = this -> XDIM;
    float y_dim = this -> YDIM;

    std::vector<Node> nodes;
    std::vector<float> x_new(2);
    Node x_nearest;
    Node x_new;
    while (nodes.size() < this -> max_num_nodes)
    {
      bool found_next = false;
      while (found_next == false)
      {
        x_rand = this -> sampleFree(x_dim, y_dim);

        GetNearestListIndex
        // TODO
        x_nearest_id = this -> getNearest(nodes, x_rand);

        
        x_new = this -> steer(x_nearest.point, x_rand);
        if (this->obstacle_free(x_nearest, x_new))
        {
          
          new
          parent_node = x_nearest;

          
          Node newnode = {x_new.x, x_new.y, x_nearest_id, 0.0};

          nodes.push_back(newnode);

        // Python
                    parent_node = x_nearest

                    newnode = Node(x_new, parent_node)

                    [newnode, x_nearest] = self.choose_parent(parent_node, newnode, nodes)
                    nodes.append(newnode)
                    nodes = self.rewire(nodes,newnode)

                    nodes.append(newnode)
             
                    foundNext = True
        }

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

  float RRTstarPlannerROS::distance(px1, py1, px2, py2)
  {
    float dist = sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
    return dist;
  }

  std::vector<float> RRTstarPlannerROS::sampleFree(float x_dim, float y_dim)
  {
    float MAX_RANDOM_NUMBER = 1.0;

    float x = (distribution(generator) - MAX_RANDOM_NUMBER/2)*x_dim;
    float y = (distribution(generator) - MAX_RANDOM_NUMBER/2)*y_dim;
    std::vector<float> x_rand = {x,y};
    return x_rand; 
  }

  std::vector<int> worldToMap()
  {
    int cell_column = int((x - originX) / resolution);
    return 
  }


void RRTstarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) {
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

      # get the cell coord of the center point of the robot
    def world_to_map(self, x, y):
        cell_column = int((x - self.MAP.info.origin.position.x) / self.MAP.info.resolution)
cell_row = int((y - self.MAP.info.origin.position.y) / self.MAP.info.resolution)


  //check if point collides with the obstacle
  bool RRTstarPlannerROS::collision(wx, wy)
  {
    int mx, my;
    worldToMap(double wx, double wy, double& mx, double& my);
    

  }
      

        # TODO: get a point and transform into cell grid

        # TODO: verify if thi

        cell_row, cell_column = self.world_to_map(p[0], p[1])

        if cell_column < 0:
            return True
        elif cell_row < 0:
            return True
        elif cell_row - 1 < 0:
            return True
        elif cell_column -1 < 0:
            return True
        elif cell_column >= self.grid.shape[1]:
            return True
        elif cell_row >= self.grid.shape[0]:
            return True
        elif cell_column+1 >= self.grid.shape[1]:
            return True
        elif cell_row +1>= self.grid.shape[0]:
            return True
        elif self.grid[cell_row][cell_column] == 1:
            return True
        elif self.grid[cell_row+1][cell_column+1] == 1:
            return True
        elif self.grid[cell_row][cell_column+1] == 1:
            return True
        elif self.grid[cell_row+1][cell_column] == 1:
            return True
        elif self.grid[cell_row-1][cell_column] == 1:
            return True  
        elif self.grid[cell_row][cell_column-1] == 1:
            return True   
        elif self.grid[cell_row-1][cell_column-1] == 1:
            return True
        else:
            return False