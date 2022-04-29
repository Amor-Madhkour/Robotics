#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>


void DynamicReconfigureCallback(project1::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d", config.integration_mode);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "integration_mode");

    //Set dynamic reconfigure
    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    ros::spin();
}
    