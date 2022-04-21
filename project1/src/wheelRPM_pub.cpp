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


float h[4][3];
float wheels_offset[4][2] = {{l,w}, {l,-w}, {-l,-w}, {-l,w}};
float gammas[4] = {-PI/4, PI/4, -PI/4, PI/4};

ros::Publisher pub_wheelRPM;

void CalcluateWheelsRPM(const geometry_msgs::TwistStamped& msg_in){

  project1::StampedWheelRPM msg_out = project1::StampedWheelRPM();

  msg_out.header.stamp = msg_in.header.stamp;
  msg_out.header.frame_id = msg_in.header.frame_id;
  //TODO VERIFICA FORMULA E Unità misura
  msg_out.rpm_fl = (h[0][0] * msg_in.twist.linear.x + h[0][1] * msg_in.twist.linear.y + h[0][2] * msg_in.twist.angular.z) / r * 60 /(2 * PI);
  msg_out.rpm_fr = (h[1][0] * msg_in.twist.linear.x + h[1][1] * msg_in.twist.linear.y + h[1][2] * msg_in.twist.angular.z) / r * 60 /(2 * PI);
  msg_out.rpm_rr = (h[2][0] * msg_in.twist.linear.x + h[2][1] * msg_in.twist.linear.y + h[2][2] * msg_in.twist.angular.z) / r * 60 /(2 * PI);
  msg_out.rpm_rl = (h[3][0] * msg_in.twist.linear.x + h[3][1] * msg_in.twist.linear.y + h[3][2] * msg_in.twist.angular.z) / r * 60 /(2 * PI);

  pub_wheelRPM.publish(msg_out);

  ROS_INFO("fl: %f, fr: %f, rr: %f, rl: %f", msg_out.rpm_fl, msg_out.rpm_fr, msg_out.rpm_rr, msg_out.rpm_rl);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_wheels_rpm");
  ros::NodeHandle nh;

  //wheels_offset = {{l,w}, {l,-w}, {-l,-w}, {-l,w}};
  //gammas = {-PI/4, PI/4, -PI/4, PI/4};
  for(int i = 0; i < 4; ++i){
    h[i][0] = 1;
    h[i][1] = tan(gammas[i]);
    h[i][2] = wheels_offset[i][0] * tan(gammas[i]) + wheels_offset[i][1];
  }

  ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, &CalcluateWheelsRPM);
  
  pub_wheelRPM = nh.advertise<project1::StampedWheelRPM>("/wheels_rpm", 10);
  //ros::Rate rate(10);

  ROS_INFO("WHEELS RPM NODE");
  ros::spin();
}