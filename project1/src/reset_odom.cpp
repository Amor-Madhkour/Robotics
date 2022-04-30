#include "ros/ros.h"
#include "project1/reset_odom.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "reset_odom_service");
    if(argc != 4)
    {
        ROS_INFO("The correct syntax is rosrun project1 reset_odom X Y THETA");
        return 1;
    }

    ros::NodeHandle n;
    ros::service::waitForService("reset_odom");
    ros::ServiceClient set_pos = n.serviceClient<project1::reset_odom>("reset_odom");
    project1 ::reset_odom srv;
    srv.request.x = atoll(argv[1]);
    srv.request.y = atoll(argv[2]);
    srv.request.theta = atoll(argv[3]);

    if (set_pos.call(srv))
    {
        ROS_INFO("Odometry has been reset");
    }
    else
    {
        ROS_ERROR("Failed to call service reset_odom");
        return 1;
    }
    return 0;
}