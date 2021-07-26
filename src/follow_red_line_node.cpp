

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// local includes
#include <thi_turtlebot3_sim/LineFollower.h>

#include <image_transport/image_transport.h>

LineFollower lf; 
ros::Publisher vel_pub ; 


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
  lf.setInputImage(msg); 
  geometry_msgs::Twist twist = lf.process(); 
  vel_pub.publish(twist); 
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_red_line_node");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  vel_pub    = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

  ros::Rate loop_rate(30);
  while(ros::ok())
  {
      ros::spinOnce(); 
      loop_rate.sleep();    

  }

  return 0;
}