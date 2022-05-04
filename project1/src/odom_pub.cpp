#include <ros/ros.h>
#include "data.h"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 
#include <project1/reset_odom.h>


struct odom_struct {
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
  tf2_ros::TransformBroadcaster odom_broadcaster;
  
  tf::TransformBroadcaster br;
  tf::Transform odom_transform;
  tf2::Quaternion myQuat;
 
  //Calculations
  double last_time = 1.7976931348623157E+308; //Time initialized to infinity
  odom_struct initial_odom, last_odom;


  enum integration_mode {EULER, RK};
  int current_integration;



// =============== CONSTRUCTOR ===============
public:

  odom_pub(){
    nh.getParam("/initial_x", initial_odom.x);
    nh.getParam("/initial_y", initial_odom.y);
    nh.getParam("/initial_theta", initial_odom.theta);

    odom_transform.setOrigin( tf::Vector3(initial_odom.x, initial_odom.y, 0) );
    odom_transform.setRotation( tf::Quaternion(0, 0, initial_odom.theta) );
    br.sendTransform(tf::StampedTransform(odom_transform, ros::Time::now(), "odom", "base_link"));

    //Subscribe to cmd_vel
    sub_vel = nh.subscribe("/cmd_vel", 1, &odom_pub::CalcluateOdometryCallback, this);
   
    //Setup publisher odom
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);


    //Set service
    set_srv = nh.advertiseService("reset_odom", &odom_pub::ResetOdom, this);
  }


  //REMOVE?
  void ResetToInitialPositions(){
    last_odom.x = initial_odom.x;
    last_odom.y = initial_odom.y;
    last_odom.theta = initial_odom.theta;
  }

  /*REMOVE
  odom_struct CalculateDeltas(float vx, float vy, float omega, double dt)
  {
    odom_struct deltas_b; //x, y, theta - referred to robot frame

    //Calculate deltas referred to robot frame
    if(omega != 0){
      deltas_b.x = (vx * sin(omega) + vy * (cos(omega) - 1)) / omega * dt;
      deltas_b.y = (vy * sin(omega) + vx * (1 - cos(omega))) / omega * dt;
      deltas_b.theta = omega * dt;
    }
    else {
      deltas_b.x = vx * dt;
      deltas_b.y = vy * dt;
      deltas_b.theta = 0;
    }
    
    return deltas_b;
  }
  */

  void Euler(float vx, float vy, float omega, double dt)
  {
    odom_struct deltas_b;

    //Calculate deltas referred to odom with EULER
    /*
    last_odom.x += deltas_b.x * cos(last_odom.theta) - deltas_b.y * sin(last_odom.theta);
    last_odom.y += deltas_b.x * sin(last_odom.theta) + deltas_b.y * cos(last_odom.theta);
    last_odom.theta += deltas_b.theta;
    */

    deltas_b.x = (vx * cos(last_odom.theta) - vy * sin(last_odom.theta)) * dt;
    deltas_b.y = (vx * sin(last_odom.theta) - vy * cos(last_odom.theta)) * dt;
    deltas_b.theta = omega * dt;

    last_odom.x += deltas_b.x;
    last_odom.y += deltas_b.y;
    last_odom.theta += deltas_b.theta;


  }

  void RungeKutta(float vx, float vy, float omega, double dt)
  {
    odom_struct deltas_b;

    //Calculate deltas referred to odom with RK
    float offset = omega * dt / 2;

    /*
    last_odom.x += deltas_b.x * cos(last_odom.theta + offset) - deltas_b.y * sin(last_odom.theta + offset);
    last_odom.y += deltas_b.x * sin(last_odom.theta + offset) + deltas_b.y * cos(last_odom.theta + offset);
    last_odom.theta += deltas_b.theta;
    */

    deltas_b.x = (vx * cos(last_odom.theta + offset) - vy * sin(last_odom.theta + offset)) * dt;
    deltas_b.y = (vx * sin(last_odom.theta + offset) - vy * cos(last_odom.theta + offset)) * dt;
    deltas_b.theta = omega * dt;

    last_odom.x += deltas_b.x;
    last_odom.y += deltas_b.y;
    last_odom.theta += deltas_b.theta;
  }

  void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

    nav_msgs::Odometry msg_out = nav_msgs::Odometry();

    //Reset position if the bag restarts
    if(last_time > msg_in.header.stamp.toSec())
    {
      last_time = msg_in.header.stamp.toSec();
      ResetToInitialPositions(); //TO BE REMOVED
      return;
    }

    nh.getParam("/dynamic_reconfigure/integration_mode", current_integration);

    //Integrate in the modality specified by the parameter 
    if(current_integration == integration_mode::EULER)
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    else
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);

    last_time = msg_in.header.stamp.toSec();

    //Setup message out
    msg_out.header.seq = msg_in.header.seq;
    msg_out.header.stamp = msg_in.header.stamp;
    msg_out.header.frame_id = "base_link";
    msg_out.pose.pose.position.x = last_odom.x;
    msg_out.pose.pose.position.y = last_odom.y;
    msg_out.pose.pose.position.z = 0.0;
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
    ROS_INFO("x: %f, y: %f, theta: %f", last_odom.x, last_odom.y, last_odom.theta);
  }

  void BroadcastTF(float x, float y, float th,  ros::Time timeStamp)
  {
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = timeStamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    
    odom_trans.transform.rotation.x = myQuat.x();
    odom_trans.transform.rotation.y = myQuat.y();
    odom_trans.transform.rotation.z = myQuat.z();
    odom_trans.transform.rotation.w = myQuat.w();
  
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }

  
  //======================= SERVICE ====================
  bool ResetOdom(project1::reset_odom::Request &req, project1::reset_odom::Response &res)
  {

      //Recalculate position of odom relative to the desired base_link new pose
      odom_struct new_transform;
      new_transform.x = last_odom.x - req.new_x;
      new_transform.y = last_odom.y - req.new_y;
      new_transform.theta = last_odom.theta - req.new_theta;
     
      last_odom.x = req.new_x;
      last_odom.y = req.new_y;
      last_odom.theta = req.new_theta;
      
      odom_transform.setOrigin( tf::Vector3(new_transform.x, new_transform.y, 0) );
      odom_transform.setRotation( tf::Quaternion(0, 0, new_transform.theta) );
      br.sendTransform(tf::StampedTransform(odom_transform, ros::Time::now(), "odom", "base_link"));

      ROS_INFO("Odom has been set to ([%f],[%f],[%f])", last_odom.x, last_odom.y, last_odom.theta);
      return true;
  }
  //=====================================================
};

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_odometry");
  odom_pub my_odom_pub;
  ROS_INFO("ODOMETRY NODE");
  ros::spin();
}