#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
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

ros::Publisher pub;

void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

  geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();
  msg_out.header.stamp = ros::Time(0);

  //msg_out.twist.linear.x
  //msg_out.twist.angular.x

  //TODO ADD FORMULA 

  //pub.publish(msg_out);

  ROS_INFO("Ciao");
}

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_velocity");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/wheel_states", 1000, &CalcluateVelocityCallback);
  //pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  //ros::Rate rate(10);

  ros::spin();
}