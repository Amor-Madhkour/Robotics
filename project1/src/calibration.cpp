#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Time.h>

#include "vel_pub.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_velocity");
  ros::NodeHandle nh;

  ros::Subscriber sub_wheels = nh.subscribe("/wheel_states", 1, &CalcluateVelocityCallback);
  
  pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

  //TODO REMOVE
  //pub_w = nh.advertise<geometry_msgs::TwistStamped>("/wheel_vel_test", 1);
  //TODO REMOVE

  //ros::Rate rate(100);

  ROS_INFO("VELOCITY NODE");
  while(ros::ok()){

    ros::spinOnce();

  }
  
}