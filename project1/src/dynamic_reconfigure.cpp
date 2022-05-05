#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "dynamic_reconfigure");

   

    ros::spin();
}
