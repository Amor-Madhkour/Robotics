#include <ros/ros.h>
#include "data.h"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 
#include <project1/reset_odom.h>


struct pose {
  float x;
  float y;
  float theta;
};

class odom_pub {

// ================ ATTRIBUTES ================
private:

  //ROS
  ros::NodeHandle nh;
  ros::Publisher pub_odom;
  ros::ServiceServer set_srv;
  ros::Subscriber sub_vel;

  tf::TransformBroadcaster br;
  tf::Transform odom_transform;
 /* tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  */
  //Calculations
  double last_time; //Time initialized to infinity
  pose odom_frame_pose; //odom frame pose
  pose pose_world; //robot odometry referred to world frame
  pose pose_odom; //robot odometry referred to odom frame


  enum integration_mode {EULER, RK};
  int current_integration;



// =============== CONSTRUCTOR ===============
public:

  odom_pub(){

    last_time = 0;

    nh.getParam("/initial_x", odom_frame_pose.x);
    nh.getParam("/initial_y", odom_frame_pose.y);
    nh.getParam("/initial_theta", odom_frame_pose.theta);

    //In questo modo la pose del base_link rispetto a odom è (0,0,0) nell'istante 0
    pose_world.x = odom_frame_pose.x;
    pose_world.y = odom_frame_pose.y;
    pose_world.theta = odom_frame_pose.theta;

    pose_odom.x = 0;
    pose_odom.y = 0;
    pose_odom.theta = 0;

    odom_transform.setOrigin( tf::Vector3(odom_frame_pose.x, odom_frame_pose.y, 0) );
    odom_transform.setRotation( tf::Quaternion(0, 0, odom_frame_pose.theta) );
    br.sendTransform(tf::StampedTransform(odom_transform, ros::Time::now(), "odom", "base_link"));

    //Subscribe to cmd_vel
    sub_vel = nh.subscribe("/cmd_vel", 1, &odom_pub::CalcluateOdometryCallback, this);
    
    //Setup publisher odom
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);


    //Amor try
    //pub_odom = nh.advertise<project1::Odom>("my_odom", 1);

    //Set service
    set_srv = nh.advertiseService("reset_odom", &odom_pub::ResetOdom, this);
  }


  //REMOVE?
  void ResetPosition(){
    pose_world.x = odom_frame_pose.x;
    pose_world.y = odom_frame_pose.y;
    pose_world.theta = odom_frame_pose.theta;
  }

  void UpdatePoseWorldFromDeltas(pose deltas_b)
  {
    pose_world.x += deltas_b.x;
    pose_world.y += deltas_b.y;
    pose_world.theta += deltas_b.theta;
  }

  void UpdatePoseOdom()
  {
    pose_odom.x = pose_world.x - odom_frame_pose.x;
    pose_odom.y = pose_world.y - odom_frame_pose.y;
    pose_odom.theta = pose_world.theta - odom_frame_pose.theta;
  }

  void Euler(float vx, float vy, float omega, double dt)
  {
    pose deltas_b;

    //Calculate deltas referred to odom with EULER
    deltas_b.x = (vx * cos(pose_world.theta) - vy * sin(pose_world.theta)) * dt;
    deltas_b.y = (vx * sin(pose_world.theta) + vy * cos(pose_world.theta)) * dt;
    deltas_b.theta = omega * dt;

    //Update the pose of the robot in the world given the deltas
    UpdatePoseWorldFromDeltas(deltas_b);

    //Update the pose of the robot in the world - Notice: it cannot be updated by hand, but it is calculated relatively to the frame odom position
    UpdatePoseOdom();

  }

  void RungeKutta(float vx, float vy, float omega, double dt)
  {
    pose deltas_b;

    //Calculate deltas referred to odom with RK
    float offset = omega * dt / 2;
    deltas_b.x = (vx * cos(pose_world.theta + offset) - vy * sin(pose_world.theta + offset)) * dt;
    deltas_b.y = (vx * sin(pose_world.theta + offset) + vy * cos(pose_world.theta + offset)) * dt;
    deltas_b.theta = omega * dt;

    //Update the pose of the robot in the world given the deltas
    UpdatePoseWorldFromDeltas(deltas_b);

    //Update the pose of the robot in the world - Notice: it cannot be updated by hand, but it is calculated relatively to the frame odom position
    UpdatePoseOdom();
  }

  void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

    nav_msgs::Odometry msg_out = nav_msgs::Odometry();

    if(last_time == 0)
    {
      last_time = msg_in.header.stamp.toSec();
      return;
    }

    //TODO CHECK IF IS IT OK
    nh.getParam("/dynamic_reconfigure/integration_mode", current_integration);

    //Integrate in the modality specified by the parameter 
    if(current_integration == integration_mode::EULER)
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    else
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);

    last_time = msg_in.header.stamp.toSec();

    //Setup message out with the pose referred to frame odom
    msg_out.header.seq = msg_in.header.seq;
    msg_out.header.stamp = msg_in.header.stamp;
    msg_out.header.frame_id = "base_link";
    msg_out.pose.pose.position.x = pose_odom.x;
    msg_out.pose.pose.position.y = pose_odom.y;
    
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,last_odom.theta);
    msg_out.pose.pose.orientation.x = myQuat.x();
    msg_out.pose.pose.orientation.y= myQuat.y();
    msg_out.pose.pose.orientation.z = myQuat.z();
    msg_out.pose.pose.orientation.w = myQuat.w();

    //Publish odom
    pub_odom.publish(msg_out);

    BroadcastTF(msg_out.pose.pose.position.x, msg_out.pose.pose.position.y,msg_out.pose.pose.orientation.w, msg_in.header.stamp);
    
    if(current_integration==integration_mode::EULER)
      ROS_INFO("Integration_mode: Euler");
    else
      ROS_INFO("Integration_mode: RK");

    ROS_INFO("WORLD: x: %f, y: %f, theta: %f", pose_world.x, pose_world.y, pose_world.theta);
    ROS_INFO("ODOM: x: %f, y: %f, theta: %f", pose_odom.x, pose_odom.y, pose_odom.theta);
  }

    //TODO
  /*void CalcluateOdometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg_in){

    nav_msgs::Odometry msg_out = nav_msgs::Odometry();

    //Reset position if the bag restarts
    if(last_time > msg_in.header.stamp.toSec())
    {
      last_time = msg_in.header.stamp.toSec();
      SetInitialPositions(); //TO BE REMOVED
      return;
    }

    nh.getParam("/integration_mode/integration_mode", current_integration);
    
    //Integrate in the modality specified by the parameter 
    if(current_integration == integration_mode::EULER)
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    else
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    
    //Roba New
    last_time = msg_in.header.stamp.toSec();
    publish_msg(msg_in);
    }

    void publish_msg(const geometry_msgs::TwistStamped::ConstPtr& msg_in) 
    {
        msg_out.odom.header.stamp = msg_in->header.stamp;
        msg_out.odom.header.seq = msg_in->header.seq;
        msg_out.odom.header.frame_id = "odom";
        msg_out.odom.child_frame_id = "base_link";
   
        msg_out.pose.pose.position.x = pose_world.x;
        msg_out.pose.pose.position.y = pose_world.y;
        msg_out.pose.pose.position.z = pose_world.y;
        msg_out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_world.theta);

        pub.publish(msg);
 
    } 
  }

  */

  void BroadcastTF(const geometry_msgs::TwistStamped& msg_in)
  {
    //publish the transform over tf
    ROS_INFO("fuck fuck fuck fuck");
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    myQuat.setRPY(0,0,pose_world.theta);
    odom_trans.transform.translation.x = pose_world.x;
    odom_trans.transform.translation.y = pose_world.y;
    odom_trans.transform.translation.z = 0.0;
    
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,last_odom.theta);
    odom_trans.transform.rotation.x = myQuat.x();
    odom_trans.transform.rotation.y = myQuat.y();
    odom_trans.transform.rotation.z = myQuat.z();
    odom_trans.transform.rotation.w = myQuat.w();

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

  }

  /*
  void BroadcastTF(float x, float y, float th,  ros::Time timeStamp)
  {
    //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

    //publish the transform over tf
    
    odom_trans.header.stamp = timeStamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }*/

  
  //======================= SERVICE ====================
  bool ResetOdom(project1::reset_odom::Request &req, project1::reset_odom::Response &res)
  {
      //Recalculate position of odom relative to the desired base_link new pose
      odom_frame_pose.x = pose_odom.x - req.new_x;
      odom_frame_pose.y = pose_odom.y - req.new_y;
      odom_frame_pose.theta = pose_odom.theta - req.new_theta;

      odom_transform.setOrigin( tf::Vector3(odom_frame_pose.x, odom_frame_pose.y, 0) );
      odom_transform.setRotation( tf::Quaternion(0, 0, odom_frame_pose.theta) );
      br.sendTransform(tf::StampedTransform(odom_transform, ros::Time(last_time), "odom", "base_link"));

      ROS_INFO("Odom frame has been set to ([%f],[%f],[%f])", odom_frame_pose.x, odom_frame_pose.y, odom_frame_pose.theta);
      ROS_INFO("Robot new pose([%f],[%f],[%f])", req.new_x, req.new_y, req.new_theta);

      return true;
  }

  /* DEPRECATED
  bool ResetOdom(project1::reset_odom::Request &req, project1::reset_odom::Response &res)
  {
    pose_world.x = req.new_x;
    pose_world.y = req.new_y;
    pose_world.theta = req.new_theta;
  }
  */
  //=====================================================
};

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_odometry");
  odom_pub my_odom_pub;
  ROS_INFO("ODOMETRY NODE");
  ros::spin();
}