

#include <ros/ros.h>


#include <sensor_msgs/LaserScan.h>

#include <thi_turtlebot3_sim/Explorer.h>
#include <memory>



nav_msgs::OccupancyGrid::ConstPtr _map; 



std::unique_ptr<Explorer> e; 



void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr map)
{

   e->setMap(map);

   e->process(); 
}




int main(int argc, char **argv)
{
   ros::init(argc, argv, "thi_explore_node");
   ros::NodeHandle n;


   ros::Subscriber map_sub = n.subscribe("map", 1, mapCallback);


   e = std::make_unique<Explorer>(n); 


   ros::Rate loop_rate(100);
   while(ros::ok())
   {
      ros::spin(); 
      loop_rate.sleep();  
   }

   return 0;
}