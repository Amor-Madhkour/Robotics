#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <project1/StampedWheelRPM.h>

#include <math.h> 

#define PI 3.1415

const float r = 0.07;
const float l = 0.2;
const float w = 0.169;
const int T = 5;
const int N = 42;
const int N_WHEELS = 4;

ros::Publisher pub_wheelRPM;

void CalcluateWheelsRPM(const geometry_msgs::TwistStamped& msg_in){

  project1::StampedWheelRPM msg_out = project1::StampedWheelRPM();

  msg_out.header.stamp = msg_in.header.stamp;
  msg_out.header.frame_id = msg_in.header.frame_id;
  
  // VERIFICA SE VA DIVISO PER 2PI
  msg_out.rpm_fl = (msg_in.twist.linear.x - msg_in.twist.linear.y + (-l-w) * msg_in.twist.angular.z) / r * 60;
  msg_out.rpm_fr = (msg_in.twist.linear.x + msg_in.twist.linear.y + (l+w) * msg_in.twist.angular.z) / r * 60;
  msg_out.rpm_rl = (msg_in.twist.linear.x + msg_in.twist.linear.y + (-l-w) * msg_in.twist.angular.z) / r * 60;
  msg_out.rpm_rr = (msg_in.twist.linear.x - msg_in.twist.linear.y + (l+w) * msg_in.twist.angular.z) / r * 60;

  pub_wheelRPM.publish(msg_out);

  //ROS_INFO("fl: %f, fr: %f, rr: %f, rl: %f", msg_out.rpm_fl, msg_out.rpm_fr, msg_out.rpm_rr, msg_out.rpm_rl);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_wheels_rpm");
  ros::NodeHandle nh;

  ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, &CalcluateWheelsRPM);
  
  pub_wheelRPM = nh.advertise<project1::StampedWheelRPM>("/wheels_rpm", 10);
  //ros::Rate rate(10);

  ROS_INFO("WHEELS RPM NODE");
  ros::spin();
}