
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



constexpr auto FREE     =  0;
constexpr auto UNKNOWN  = -1;  
constexpr auto OCCUPIED =  1; 
constexpr auto UNVALID  = -200; 
constexpr auto FRONTIER = 100; 


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



    void process(void)
    {
      // resize map to speed up exploration
      nav_msgs::OccupancyGrid resizedMap      = this->resizeMap(_map);

      // find frontiers
      nav_msgs::OccupancyGrid frontier_map    = this->findFrontiers(resizedMap);

      // group frontiers 
      std::vector<Frontier> frontier_grouped  = this->groupFrontiers(frontier_map);

      // weight frontiers      
      std::vector<Frontier> frontier_weighted = this->weightFrontiers(frontier_grouped);
    }


    nav_msgs::OccupancyGrid resizeMap(const nav_msgs::OccupancyGrid input_map)
    {
      nav_msgs::OccupancyGrid output; 





      return output; 
    }


    bool idxInRange(const unsigned int idx, const nav_msgs::OccupancyGrid map) const
    {
      const auto map_size = map.info.width*map.info.height; 

      if(idx < 0)        return false; 

      if(idx > map_size) return false; 


      return true; 
    }


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


    bool neighborIsUnknown(const std::vector<int> neighbors, const nav_msgs::OccupancyGrid map) const
    {
      for (size_t i = 0; i < neighbors.size() ; i++)
      {
        if(neighbors[i] == UNKNOWN)   return true; 
      }

      return false; 
    }


    nav_msgs::OccupancyGrid findFrontiers(const nav_msgs::OccupancyGrid map) const
    {

      // todo make this a function to copy the map information
      nav_msgs::OccupancyGrid frontier_map; 
      frontier_map.header.frame_id = map.header.frame_id; 
      frontier_map.header.seq      = map.header.seq; 
      frontier_map.header.stamp    = map.header.stamp; 
      frontier_map.info.height     = map.info.height; 
      frontier_map.info.width      = map.info.width;  
      frontier_map.info.height     = map.info.height; 
      frontier_map.info.resolution = map.info.resolution;
      const auto map_size          = frontier_map.info.width * frontier_map.info.height;  
      frontier_map.data.resize(map_size); 


      // iterate over all frontier cells 
      const auto nr_of_cells = map.info.height*map.info.width; 

      for (size_t i = 0; i < nr_of_cells; i++)
      {
        const auto current_cell = map.data[i];
        if(current_cell == FREE)
        {
          const std::vector<int> neighbors = this->getNeighborIndices(i, map); 

          if( this->neighborIsUnknown(neighbors, map))
          {
            frontier_map.data[i] = FRONTIER; 
          }
        }
      }
      

    }

    std::vector<Frontier> groupFrontiers(const nav_msgs::OccupancyGrid map) const
    {


    }


    std::vector<Frontier> weightFrontiers(std::vector<Frontier> frontiers) const
    {
      std::vector<Frontier> weighted_frontiers; 

      return weighted_frontiers; 
    }





  private:
    ros::NodeHandle         _nh;
    nav_msgs::OccupancyGrid _map;  
};



Explorer::Explorer(ros::NodeHandle nh)
{
  _nh = nh; 
}

Explorer::~Explorer(void)
{

}



#endif // EXPLORER
