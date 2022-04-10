#include "ros/ros.h"
#include "Robotics/Reset.h"
#include <nav_msgs/Odometry.h>
   

bool Turtle::teleportAbsoluteCallback(turtlesim::TeleportAbsolute::Request& req, turtlesim::TeleportAbsolute::Response&)
{
  teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
  return true;
}
bool reset_callback(pub_sub::Reset::Request &req, pub_sub::Reset::Response &res) 
{
   //todo implementing the settage of the new parameter
   
      float x = req.new_x;
      float y = req.new_y;
      float th = req.new_theta;

    return true;
}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "reset_position_server");
    ros::NodeHandle n;
 
    ros::ServiceServer service = n.advertiseService<pub_sub::Reset::Request, pub_sub::Reset::Response>("reset",boost::bind(&reset_callback, _1, _2));
    ROS_INFO("Ready to reset_pos.");
    ros::spin();
  
    return 0;
}