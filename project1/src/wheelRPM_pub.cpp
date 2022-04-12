#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <project1/StampedWheelRPM.h>


float r = 0.7;
float l = 0.2;
float w = 0.169;

// Wheel wlb; //left bottom wheel
// Wheel wlt; //left top wheel
// Wheel wrb; //right bottom wheel
// Wheel wrt; //right top wheel

ros::Publisher pub_wheelRPM;

void CalcluateWheelsRPM(const geometry_msgs::TwistStamped& msg_in){

  project1::StampedWheelRPM msg_out = project1::StampedWheelRPM();

  //msg_out.rpm_fl = ...

  //TODO ADD FORMULA 

  //pub_wheelRPM.publish(msg_out);

  //BroadcastTF(msg_out.twist.linear.x, msg_out.twist.linear.y, msg_out.twist.angular.z);

  ROS_INFO("Ciao wheels_rpm");
}


int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_wheels_rpm");
  ros::NodeHandle nh;

  ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, &CalcluateWheelsRPM);
  
  pub_wheelRPM = nh.advertise<project1::StampedWheelRPM>("/wheels_rpm", 10);
  //ros::Rate rate(10);

  ros::spin();
}