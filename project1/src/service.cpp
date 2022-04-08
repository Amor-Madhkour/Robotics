#include "ros/ros.h"
#include "Robotics/Reset.h"
   
bool reset_callback(int *count, pub_sub::Reset::Request &req, pub_sub::Reset::Response &res) 
{
   //todo implementing the settage of the new parameter

    return true;
}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "reset_position_server");
    ros::NodeHandle n;
 
    ros::ServiceServer service = n.advertiseService<pub_sub::Reset::Request, pub_sub::Reset::Response>("reset",boost::bind(&reset_callback, &count, _1, _2));
    ROS_INFO("Ready to reset_pos.");
    ros::spin();
  
    return 0;
}