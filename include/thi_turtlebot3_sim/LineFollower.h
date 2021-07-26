/* Copyright (C) 2020 Prof. Dr. Christian Pfitzner - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license, which unfortunately won't be
 * written for another century.
 */


#if !defined(LINEFOLLOWER)
#define LINEFOLLOWER

// ros includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>



bool _debug = false; 


class LineFollower
{
public:
  LineFollower(/* args */)
  {
    _error_sum = 0.0; 
  
  }

  ~LineFollower();

  geometry_msgs::Twist setInputImage(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      _input = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    } 


    // get a single line from the image
    cv::Mat line = _input->image.row(_input->image.rows * 2 / 3); 

    // the center of the line is found by adding all indices above a provided threshold
    const auto threshold = 100; 
    unsigned int pixel_index_sum = 0; 
    unsigned int nr_of_pixel_over_threshold = 0; 

    for(unsigned int i=0 ; i<_input->image.cols ; i++)
    {
      const unsigned int pixel_value = (unsigned int)(line.at<unsigned char>(0,3*i)); 

      if(pixel_value > threshold)
      {
        pixel_index_sum += i; 
        nr_of_pixel_over_threshold++; 
      }
    }

    // in case of no red line found -> drive straight
    if(nr_of_pixel_over_threshold == 0)
    {
      geometry_msgs::Twist twist_drive_straight;
      twist_drive_straight.linear.x = 0.3; 
      return twist_drive_straight; 
    }

    const double line_row = pixel_index_sum / nr_of_pixel_over_threshold; 
    const double error    = -(line_row - 160.0); 

    // update error sum for i controller 
    _error_sum += error; 

    // anti windup for i controller
    const auto anti_wu_th = 200; 
    if(_error_sum < -anti_wu_th) _error_sum = -anti_wu_th;
    if(_error_sum >  anti_wu_th) _error_sum =  anti_wu_th;  

    const auto p_portion           = error / 1000.0; 
    const auto i_portion           = _error_sum * (1 / 3000.0); 
    const auto d_portion           = 0.0; 
    const auto steering_velocity  = p_portion + i_portion + d_portion; 



    // generate a twist message based on steering angle
    geometry_msgs::Twist twist; 
    twist.linear.x  = 0.3 - abs(steering_velocity);       // drive slow if the steering angle is high in value
    twist.angular.z = steering_velocity;                  // controll steering angle

    if(_debug)
    {
      ROS_ERROR_STREAM("error:     " << error); 
      ROS_ERROR_STREAM("error_sum: " << _error_sum); 
      ROS_ERROR_STREAM("line pos:  " << line_row); 
      ROS_ERROR_STREAM("middle:    " << _input->image.cols/2); 
      ROS_ERROR_STREAM("steering:  " << steering_velocity); 
    }


    return twist; 
  }

private:
  cv_bridge::CvImagePtr _input;
  double                _error_sum; 

};

LineFollower::~LineFollower()
{

}



#endif // LINEFOLLOWER
