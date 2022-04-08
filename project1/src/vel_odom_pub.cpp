#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


float r = 0.7;
float l = 0.2;
float w = 0.169;

// struct Wheel {
//   float tick;
//   float angVel;
// }

// Wheel wlb; //left bottom wheel
// Wheel wlt; //left top wheel
// Wheel wrb; //right bottom wheel
// Wheel wrt; //right top wheel

ros::Publisher pub_odom;
ros::Publisher pub_vel;

void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

  geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();
  msg_out.header.stamp = ros::Time(0);

  //msg_out.twist.linear.x
  //msg_out.twist.angular.x

  //TODO ADD FORMULA 

  //pub_vel.publish(msg_out);

  ROS_INFO("Ciao Velocity");
}

void CalcluateOdometryCallback(const geometry_msgs::PoseStamped& msg_in){

  nav_msgs::Odometry msg_out = nav_msgs::Odometry();

  //msg_out.twist.linear.x
  //msg_out.twist.angular.x

  //TODO ADD FORMULA 

  //pub_vel.publish(msg_out);

  ROS_INFO("Ciao Odometry");
}

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_velocity");
  ros::NodeHandle nh;

  ros::Subscriber sub_wheels = nh.subscribe("/wheel_states", 10, &CalcluateVelocityCallback);
  ros::Subscriber sub_pose = nh.subscribe("/robot/pose", 10, &CalcluateOdometryCallback);
  
  pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  //ros::Rate rate(10);

  ros::spin();
}