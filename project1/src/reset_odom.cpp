#include "ros/ros.h"
#include "Robotics/Reset.h"
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "reset_position_service");
    if(argc != 4)
    {
        ROS_INFO("Usage: rosrun project set X Y THETA");
        return 1;
    }

    ros::NodeHandle n;
    ros::service::waitForService("reset_odom");
    ros::ServiceClient set_pos = n.serviceClient<project1::Reset>("reset_odom");
    project1 ::Reset srv;
    srv.request.x = atoll(argv[1]);
    srv.request.y = atoll(argv[2]);
    srv.request.theta = atoll(argv[3]);

    if (set_pos.call(srv))
    {
        ROS_INFO("Odometry has been set");
    }
    else
    {
        ROS_ERROR("Failed to call service set_odom");
        return 1;
    }
    return 0;
}