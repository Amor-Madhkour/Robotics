#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


float r = 0.7;
float l = 0.2;
float w = 0.169;

// Wheel wlb; //left bottom wheel
// Wheel wlt; //left top wheel
// Wheel wrb; //right bottom wheel
// Wheel wrt; //right top wheel

ros::Publisher pub_vel;

void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

  geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();
  msg_out.header.stamp = ros::Time::now();

  //msg_out.twist.linear.x
  //msg_out.twist.angular.x

  //TODO ADD FORMULA 

  pub_vel.publish(msg_out);

  ROS_INFO("Ciao Velocity");
}


int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_velocity");
  ros::NodeHandle nh;

  ros::Subscriber sub_wheels = nh.subscribe("/wheel_states", 10, &CalcluateVelocityCallback);
  
  pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);
  //ros::Rate rate(10);

  ros::spin();
}

//TODO CAPIRE PERCHE' NON VA BENE TWISTSTAMPED