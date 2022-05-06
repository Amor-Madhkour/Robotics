#include <ros/ros.h>
#include "data.h"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 
#include <std_msgs/Int8.h>
#include <project1/reset_odom.h>

#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>


struct pose {
  float x;
  float y;
  float theta;
};

class odom_pub {

// =========================== ATTRIBUTES =============================
private:

  //ROS
  ros::NodeHandle nh;
  ros::Publisher pub_odom;
  ros::ServiceServer set_srv;
  ros::Subscriber sub_vel;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped odom_transform;

  //Calculations
  double last_time; //Time initialized to infinity
  pose odom_frame_pose; //odom frame pose
  pose pose_odom; //pose of base_link referred to odom frame

  enum integration_mode {EULER, RK};
  int current_integration;


// =========================== CONSTRUCTOR =============================
public:

  odom_pub(){

    last_time = 0;

    //Setup odom transform
    odom_transform.header.frame_id = "world";
    odom_transform.child_frame_id = "odom";

    nh.getParam("/initial_x", odom_frame_pose.x);
    nh.getParam("/initial_y", odom_frame_pose.y);
    nh.getParam("/initial_theta", odom_frame_pose.theta);

    // In questo modo la pose del base_link rispetto a odom è (0,0,0) nell'istante 0
    // Avremmo potuto settare il frame odom in (0,0,0) e base link a (initial_x, initial_y, initial_theta)
    // per risultare allineati a /robot/pose, tuttavia secondo la nostra interpretazione non è questa la richiesta del progetto
    pose_odom.x = 0;
    pose_odom.y = 0;
    pose_odom.theta = 0;

    //Subscribe to cmd_vel
    sub_vel = nh.subscribe("/cmd_vel", 1, &odom_pub::CalcluateOdometryCallback, this);
    
    //Setup publisher odom
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    //Set service
    set_srv = nh.advertiseService("reset_odom", &odom_pub::ResetOdom, this);
  }

  void UpdatePoseOdom(pose deltas_b)
  {
    pose_odom.x += deltas_b.x;
    pose_odom.y += deltas_b.y;
    pose_odom.theta += deltas_b.theta;  
  }

  void Euler(float vx, float vy, float omega, double dt)
  {
    pose deltas_b;

    //Calculate deltas referred to odom with EULER
    deltas_b.x = (vx * cos(pose_odom.theta) - vy * sin(pose_odom.theta)) * dt;
    deltas_b.y = (vx * sin(pose_odom.theta) + vy * cos(pose_odom.theta)) * dt;
    deltas_b.theta = omega * dt;

    //Update the pose of the robot in the world - Notice: it cannot be updated by hand, but it is calculated relatively to the frame odom position
    UpdatePoseOdom(deltas_b);

  }

  void RungeKutta(float vx, float vy, float omega, double dt)
  {
    pose deltas_b;

    //Calculate deltas referred to odom with RK
    float offset = omega * dt / 2;
    deltas_b.x = (vx * cos(pose_odom.theta + offset) - vy * sin(pose_odom.theta + offset)) * dt;
    deltas_b.y = (vx * sin(pose_odom.theta + offset) + vy * cos(pose_odom.theta + offset)) * dt;
    deltas_b.theta = omega * dt;

    //Update the pose of the robot in the world - Notice: it cannot be updated by hand, but it is calculated relatively to the frame odom position
    UpdatePoseOdom(deltas_b);
  }

// ============================ CALLBACKS ===========================

  void DynamicReconfigureCallback(project1::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d", config.integration_mode);
    
    current_integration = config.integration_mode;

    if(current_integration==integration_mode::EULER)
      ROS_INFO("Integration_mode: Euler");
    else
      ROS_INFO("Integration_mode: RK");
  }

  void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

    if(last_time == 0)
    {
      last_time = msg_in.header.stamp.toSec();
      return;
    }

    //Integrate in the modality specified by the parameter 
    if(current_integration == integration_mode::EULER)
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    else
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);

    last_time = msg_in.header.stamp.toSec();

    //Setup message out with the pose referred to frame odom
    nav_msgs::Odometry msg_out = nav_msgs::Odometry();
    msg_out.header.seq = msg_in.header.seq;
    msg_out.header.stamp = msg_in.header.stamp;
    msg_out.header.frame_id = "odom";
    msg_out.pose.pose.position.x = pose_odom.x; 
    msg_out.pose.pose.position.y = pose_odom.y; 
    
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,pose_odom.theta); 
    msg_out.pose.pose.orientation.x = myQuat.x();
    msg_out.pose.pose.orientation.y = myQuat.y();
    msg_out.pose.pose.orientation.z = myQuat.z();
    msg_out.pose.pose.orientation.w = myQuat.w();
 
    //Publish odom
    pub_odom.publish(msg_out);
    

    //Broadcast transform
    BroadcastTF(pose_odom.x, pose_odom.y, pose_odom.theta, msg_in.header.stamp);

    ROS_INFO("ODOM: x: %f, y: %f, theta: %f", pose_odom.x, pose_odom.y, pose_odom.theta);
  }

// ==============================================================
  
// ======================== BROADCASTING ========================
  void BroadcastTF(float x, float y, float th,  ros::Time timeStamp)
  {
    geometry_msgs::TransformStamped odom_trans;

    //publish the transform over tf
    odom_trans.header.stamp = timeStamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,pose_odom.theta);
    odom_trans.transform.rotation.x = myQuat.x();
    odom_trans.transform.rotation.y = myQuat.y();
    odom_trans.transform.rotation.z = myQuat.z();
    odom_trans.transform.rotation.w = myQuat.w();

    //send the transform
    br.sendTransform(odom_trans);
  }
// ==============================================================
  
  //========================== SERVICE ==========================
  bool ResetOdom(project1::reset_odom::Request &req, project1::reset_odom::Response &res)
  {
      //Recalculate position of odom relative to the desired base_link new pose
      odom_frame_pose.x = req.new_x - pose_odom.x;
      odom_frame_pose.y = req.new_y - pose_odom.y;
      odom_frame_pose.theta = req.new_theta - pose_odom.theta;

      //Publish new odom frame position
      odom_transform.header.stamp = ros::Time(last_time);

      odom_transform.transform.translation.x = odom_frame_pose.x;
      odom_transform.transform.translation.y = odom_frame_pose.y;

      tf2::Quaternion myQuat;
      myQuat.setRPY(0,0,pose_odom.theta);
      odom_transform.transform.rotation.x = myQuat.x();
      odom_transform.transform.rotation.y = myQuat.y();
      odom_transform.transform.rotation.z = myQuat.z();
      odom_transform.transform.rotation.w = myQuat.w();

      //send the transform
      br.sendTransform(odom_transform);

      ROS_INFO("Odom frame has been set to: ([%f],[%f],[%f])", odom_frame_pose.x, odom_frame_pose.y, odom_frame_pose.theta);
      ROS_INFO("Robot new pose in the world: ([%f],[%f],[%f])", req.new_x, req.new_y, req.new_theta);

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
  //===============================================================
};

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_odometry");
  odom_pub my_odom_pub;
  ROS_INFO("ODOMETRY NODE");

  //Set dynamic reconfigure
  dynamic_reconfigure::Server<project1::parametersConfig> server;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
  f = boost::bind(&odom_pub::DynamicReconfigureCallback, &my_odom_pub, _1, _2);
  server.setCallback(f);

  ros::spin();
}

