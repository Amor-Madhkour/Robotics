#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


float r = 0.7;
float l = 0.2;
float w = 0.169;

ros::Publisher pub_odom;


void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

  nav_msgs::Odometry msg_out = nav_msgs::Odometry();

  //msg_out.twist.linear.x
  //msg_out.twist.angular.z

  //TODO ADD FORMULA 

  //pub_odom.publish(msg_out);

  //BroadcastTF(msg_out.twist.linear.x, msg_out.twist.linear.y, msg_out.twist.angular.z);

  ROS_INFO("Ciao Odometry");
}


void BroadcastTF(double x, double y, double th)
{
  //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

  tf::TransformBroadcaster odom_broadcaster;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_odometry");
  ros::NodeHandle nh;

  ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, &CalcluateOdometryCallback);
  
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  //ros::Rate rate(10);

  ros::spin();
}