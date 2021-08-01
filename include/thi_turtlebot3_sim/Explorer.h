
/* Copyright (C) 2020 Prof. Dr. Christian Pfitzner - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license, which unfortunately won't be
 * written for another century.
 */

#if !defined(EXPLORER)
#define EXPLORER


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


class Frontier
{

};



// const values for occupancy grid

namespace thi
{
  constexpr auto FREE     =  0;
  constexpr auto UNKNOWN  = -1;  
  constexpr auto OCCUPIED =  100;

  constexpr auto UNVALID     =  0; 
  constexpr auto FRONTIER    =  100; 
  constexpr auto NO_FRONTIER =  100; 



/**
 * @class     Explorer
 * @author    Prof. Dr. Christian Pfitzner
 * 
 */
class Explorer
{
  public:
  /**
   * @brief Construct a new Explorer object
   * @param nh      Node Handle of the ros node 
   */
    Explorer(ros::NodeHandle nh);

    /**
     * @brief Destroy the Explorer object
     */
    ~Explorer(void);

    /**
     * @brief Set the Map object
     * @param map 
     */
    void setMap(const nav_msgs::OccupancyGrid::ConstPtr map)
    {
      _map = *map; 
    }


    /**
     * @brief function to process exploration
     */
    void process(void)
    {
      // // resize map to speed up exploration
      // const nav_msgs::OccupancyGrid resizedMap      = this->resizeMap(_map, 0.1);

      // find frontiers
      const nav_msgs::OccupancyGrid frontier_map    = this->findFrontiers(_map);

      // // group frontiers 
      // const std::vector<Frontier> frontier_grouped  = this->groupFrontiers(frontier_map);

      // // weight frontiers      
      // const std::vector<Frontier> frontier_weighted = this->weightFrontiers(frontier_grouped);
    }


private: 

    /**
     * @brief Function to resize the occupancy grid
     * 
     * @param input_map           input map 
     * @param output_size         size of the new grid in meter
     * @return nav_msgs::OccupancyGrid 
     */
    nav_msgs::OccupancyGrid resizeMap(const nav_msgs::OccupancyGrid input_map, const double output_size) const
    {

      // generate output occupancy grid
      nav_msgs::OccupancyGrid output; 







      if(_resize_map_pub.getNumSubscribers() > 0)
      {
        _resize_map_pub.publish(output); 
      }

      return output; 
    }


    /**
     * @brief Function to check wheater a cell is valid or not
     * 
     * @param idx 
     * @param map 
     * @return true 
     * @return false 
     */
    bool idxInRange(const unsigned int idx, const nav_msgs::OccupancyGrid map) const
    {
    
      const auto map_size = map.info.width*map.info.height; 

      if(idx < 0)        return false; 

      if(idx > map_size) return false; 

      return true; 
    }


    /**
     * @brief Get the Neighbor Indices object
     * 
     * @param idx 
     * @param map 
     * @return std::vector<int> 
     */
    std::vector<int> getNeighborIndices(const unsigned int idx, const nav_msgs::OccupancyGrid map) const
    {
      std::vector<int> neighbors; 

      const auto width  = map.info.width; 
      const auto height = map.info.height; 


      // top
      const auto top_idx     = idx - width; 
      if(this->idxInRange(top_idx, map))     neighbors.push_back(top_idx); 
      else                                   neighbors.push_back(UNVALID); 

      // bottom
      const auto bottom_idx  = idx + width; 
      if(this->idxInRange(bottom_idx, map))  neighbors.push_back(bottom_idx); 
      else                                   neighbors.push_back(UNVALID); 

      // left
      const auto left_idx    = idx - 1; 
      if(this->idxInRange(left_idx, map))    neighbors.push_back(left_idx); 
      else                                   neighbors.push_back(UNVALID); 

      // right
      const auto right_idx   = idx + 1; 
      if(this->idxInRange(right_idx, map))   neighbors.push_back(right_idx); 
      else                                   neighbors.push_back(UNVALID);


      return neighbors; 
    }

    /**
     * @brief 
     * 
     * @param neighbors 
     * @param map 
     * @return 
     */
    bool neighborIsUnknown(const std::vector<int> neighbors, const nav_msgs::OccupancyGrid map) const
    {
      for (size_t i = 0; i < neighbors.size() ; i++)
      {
        if(neighbors[i] == UNKNOWN)   return true; 
      }

      return false; 
    }



    nav_msgs::OccupancyGrid copy(const nav_msgs::OccupancyGrid input) const 
    {
      nav_msgs::OccupancyGrid output; 
      output.header.frame_id = input.header.frame_id; 
      output.header.seq      = input.header.seq; 
      output.header.stamp    = input.header.stamp; 
      output.info.height     = input.info.height; 
      output.info.width      = input.info.width;  
      output.info.height     = input.info.height; 
      output.info.resolution = input.info.resolution;
     
      return output; 
    }


    /**
     * @brief 
     * 
     * @param map 
     * @return nav_msgs::OccupancyGrid 
     */
    nav_msgs::OccupancyGrid findFrontiers(const nav_msgs::OccupancyGrid map) const
    {

      // todo make this a function to copy the map information
      nav_msgs::OccupancyGrid frontier_map = copy(map);  
      const auto map_size          = frontier_map.info.width * frontier_map.info.height;  
      frontier_map.data.resize(map_size); 



      ROS_ERROR_STREAM("Size: " << frontier_map.data.size());

      // iterate over all frontier cells 
      const auto nr_of_cells = map.info.height*map.info.width; 

      for (size_t i = 0; i < nr_of_cells; i++)
      {
        const auto current_cell = map.data[i];



        if(current_cell == UNKNOWN)
        {
          const std::vector<int> neighbors = this->getNeighborIndices(i, map); 
          ROS_ERROR_STREAM("free"); 

          if( this->neighborIsUnknown(neighbors, map))
          {
            frontier_map.data[i] = FRONTIER; 
            ROS_ERROR_STREAM("1"); 
          }
          else 
          {
            frontier_map.data[i] = NO_FRONTIER;
          }
        }
        else
        {
          frontier_map.data[i] = NO_FRONTIER; 
        }
      }
      

      ROS_ERROR_STREAM(__PRETTY_FUNCTION__); 

      // publish debug message if anyone is interested
      if(_frontier_map_pub.getNumSubscribers() > 0)
      {
        _frontier_map_pub.publish(frontier_map); 
      }


      return frontier_map; 
    }


    /**
     * @brief 
     * 
     * @param map 
     * @return std::vector<Frontier> 
     */
    std::vector<Frontier> groupFrontiers(const nav_msgs::OccupancyGrid map) const
    {


    }



    /**
     * @brief 
     * 
     * @param frontiers 
     * @return std::vector<Frontier> 
     */
    std::vector<Frontier> weightFrontiers(std::vector<Frontier> frontiers) const
    {
      std::vector<Frontier> weighted_frontiers; 





      return weighted_frontiers; 
    }





  private:
    ros::NodeHandle         _nh;      //!< node handle of ros node
    nav_msgs::OccupancyGrid _map;     //!< input of occupancy grid

    ros::Publisher          _resize_map_pub; 
    ros::Publisher          _frontier_map_pub; 
};






Explorer::Explorer(ros::NodeHandle nh)
{
  _nh = nh; 

  constexpr auto queue_size = 1; 

  _resize_map_pub   = _nh.advertise<nav_msgs::OccupancyGrid>("debug/resized_map",  queue_size); 
  _frontier_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("debug/frontier_map", queue_size); 



}



Explorer::~Explorer(void)
{

}

};


#endif // EXPLORER
