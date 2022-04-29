#include <ros/ros.h>
#include "data.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <project1/StampedWheelRPM.h>

#include <math.h> 

class odom_pub {

// ================ ATTRIBUTES ================
private:
  //ROS
  ros::NodeHandle nh;
  ros::Publisher pub_wheelRPM;
  ros::Subscriber sub_vel;

// =============== CONSTRUCTOR ===============
public:
  odom_pub()
  {
    sub_vel = nh.subscribe("/cmd_vel", 1, &odom_pub::CalcluateWheelsRPM, this);
    pub_wheelRPM = nh.advertise<project1::StampedWheelRPM>("/wheels_rpm", 1);
  }

  void CalcluateWheelsRPM(const geometry_msgs::TwistStamped& msg_in){

    project1::StampedWheelRPM msg_out = project1::StampedWheelRPM();

    msg_out.header.stamp = msg_in.header.stamp;
    msg_out.header.frame_id = msg_in.header.frame_id;
    
    //TODO VERIFICA SE VA DIVISO PER 2PI
    msg_out.rpm_fl = (msg_in.twist.linear.x - msg_in.twist.linear.y + (-L_LENGTH-W_LENGTH) * msg_in.twist.angular.z) / WHEEL_RADIUS * 60;
    msg_out.rpm_fr = (msg_in.twist.linear.x + msg_in.twist.linear.y + (L_LENGTH+W_LENGTH) * msg_in.twist.angular.z) / WHEEL_RADIUS * 60;
    msg_out.rpm_rl = (msg_in.twist.linear.x + msg_in.twist.linear.y + (-L_LENGTH-W_LENGTH) * msg_in.twist.angular.z) / WHEEL_RADIUS * 60;
    msg_out.rpm_rr = (msg_in.twist.linear.x - msg_in.twist.linear.y + (L_LENGTH+W_LENGTH) * msg_in.twist.angular.z) / WHEEL_RADIUS * 60;

    //Publish WheelRPM
    pub_wheelRPM.publish(msg_out);

    ROS_INFO("fl: %f, fr: %f, rl: %f, rr: %f", msg_out.rpm_fl, msg_out.rpm_fr, msg_out.rpm_rl, msg_out.rpm_rr);
  }
};

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_wheels_rpm");
  odom_pub my_odom_pub;
  ROS_INFO("WHEELS RPM NODE");
  ros::spin();
}