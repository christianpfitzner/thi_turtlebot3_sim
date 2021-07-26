

#include <ros/ros.h>


#include <sensor_msgs/LaserScan.h>


#include <thi_turtlebot3_sim/Explorer.h>




nav_msgs::OccupancyGrid::ConstPtr _map; 


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr map)
{
  _map = map; 
}




int main(int argc, char **argv)
{
   ros::init(argc, argv, "thi_explore_node");
   ros::NodeHandle n;


   ros::Subscriber map_sub = n.subscribe("map", 1, mapCallback);





   ros::Rate loop_rate(100);
   while(ros::ok())
   {
      Explorer e(n); 
      e.setMap(_map); 

      ros::spin(); 
      loop_rate.sleep();  
   }

   return 0;
}