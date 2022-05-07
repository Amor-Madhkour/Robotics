#include "data.h"
#include <project1/reset_odom.h>
#include <project1/parametersConfig.h>

#include <math.h> 

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


//We could have used the geometry_msgs/Pose, but this allows more compactness and readability of the code
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

  //Calculations
  double last_time; //Time initialized to infinity
  pose odom_frame_pose; //odom frame pose
  pose pose_odom; //pose of base_link referred to odom frame

  enum integration_mode {EULER, RK};
  int current_integration;


// ======================== CONSTRUCTOR ===========================
public:

  odom_pub(){

    last_time = 0;

    //Initialize the position of the frame odom with respect to the world
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

// =================================================================

// =========================== METHODS =============================

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

// ==================================================================

// ======================= DYNAMIC RECONFIGURE ======================

  void DynamicReconfigureCallback(project1::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d", config.integration_mode);
    
    current_integration = config.integration_mode;

    if(current_integration==integration_mode::EULER)
      ROS_INFO("Integration_mode: Euler");
    else
      ROS_INFO("Integration_mode: RK");
  }
// ==================================================================

// ============================ CALLBACKS ===========================

  void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

    //Executed only at the beginning of the bag
    if(last_time == 0)
    {
      last_time = ros::Time::now().toSec();
      return;
    }

    //Integrate in the modality specified by the parameter 
    if(current_integration == integration_mode::EULER)
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, ros::Time::now().toSec() - last_time);
    else
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, ros::Time::now().toSec() - last_time);

    last_time = ros::Time::now().toSec();

    //Setup message out with the pose referred to frame odom
    nav_msgs::Odometry msg_out = nav_msgs::Odometry();
    msg_out.header.seq = msg_in.header.seq;
    msg_out.header.stamp = ros::Time::now();
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
    

    //Broadcast transform odom->base_link
    BroadcastTF("odom", "base_link", pose_odom, ros::Time::now());

    ROS_INFO("ODOM: x: %f, y: %f, theta: %f", pose_odom.x, pose_odom.y, pose_odom.theta);
  }

// ==============================================================
  
// ======================== BROADCASTING ========================
  void BroadcastTF(const std::string& frame_id, const std::string& child_id, pose new_pose, ros::Time timeStamp)
  {
    geometry_msgs::TransformStamped tr;

    //publish the transform over tf
    tr.header.stamp = timeStamp;
    tr.header.frame_id = frame_id;
    tr.child_frame_id = child_id;

    tr.transform.translation.x = new_pose.x;
    tr.transform.translation.y = new_pose.y;
    
    tf2::Quaternion myQuat;
    myQuat.setRPY(0,0,new_pose.theta);
    tr.transform.rotation.x = myQuat.x();
    tr.transform.rotation.y = myQuat.y();
    tr.transform.rotation.z = myQuat.z();
    tr.transform.rotation.w = myQuat.w();

    //send the transform
    br.sendTransform(tr);
  }
// ==============================================================
  
  //========================== SERVICE ==========================
  bool ResetOdom(project1::reset_odom::Request &req, project1::reset_odom::Response &res)
  {
      //Recalculate position of odom relative to the desired base_link new pose
      odom_frame_pose.theta = req.new_theta - pose_odom.theta;
      odom_frame_pose.x = req.new_x - pose_odom.x * cos(odom_frame_pose.theta) - pose_odom.y * cos(odom_frame_pose.theta + M_PI/2);
      odom_frame_pose.y = req.new_y - pose_odom.x * sin(odom_frame_pose.theta) - pose_odom.y * sin(odom_frame_pose.theta + M_PI/2);

      //Publish new odom frame position
      BroadcastTF("world", "odom", odom_frame_pose, ros::Time(last_time));

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

