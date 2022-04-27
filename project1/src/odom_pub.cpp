#include <ros/ros.h>
//#include <data/data.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>

#include <math.h> 

#include <project1/Reset.h>

// ============ DATA ===========
#define PI 3.14159265359

#define r 0.07
#define l 0.2
#define w 0.169
#define N 42
#define T 5
#define N_WHEELS 4
// =============================

const float r = 0.07;
const float l = 0.2;
const float w = 0.169;

double last_time = 1.7976931348623157E+308; //Time initialized to infinity
float last_x, last_y, last_theta;
float initial_x, initial_y, initial_theta;
float delta_x_b, delta_y_b, delta_theta_b; //Referred to robot frame

//odom frame position
double x, y, theta;
int integration_mode; //If 0 Euler, if 1 Runge-kutta

ros::Publisher pub_odom;
ros::ServiceServer set_srv;


//REMOVE
void SetInitialPositions(){
  last_x = initial_x;
  last_y = initial_y;
  last_theta = initial_theta;
}

void DynamicReconfigureCallback(project1::parametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", 
            config.integration_mode);
  integration_mode = config.integration_mode;
}

void CalculateDeltas(float vx, float vy, float omega, double dt)
{
  //Calculate deltas referred to robot frame
  if(w != 0){
    delta_x_b = (vx * sin(omega) + vy * (cos(omega) - 1)) / omega * dt;
    delta_y_b = (vy * sin(omega) + vx * (1 - cos(omega))) / omega * dt;
    delta_theta_b = omega * dt;
  }
  else {
    delta_x_b = vx * dt;
    delta_y_b = vy * dt;
    delta_theta_b = 0;
  }
}

void Euler(float vx, float vy, float omega, double dt)
{
  CalculateDeltas(vx, vy, omega, dt);

  //Calculate deltas referred to odom with EULER
  last_x += delta_x_b * cos(last_theta) - delta_y_b * sin(last_theta);
  last_y += delta_x_b * sin(last_theta) + delta_y_b * cos(last_theta);
  last_theta += delta_theta_b;
}

void RungeKutta(float vx, float vy, float omega, double dt)
{
  CalculateDeltas(vx, vy, omega, dt);

  //Calculate deltas referred to odom with RK
  float offset = omega * dt / 2;
  last_x += delta_x_b * cos(last_theta + offset) - delta_y_b * sin(last_theta + offset);
  last_y += delta_x_b * sin(last_theta + offset) + delta_y_b * cos(last_theta + offset);
  last_theta += delta_theta_b;
}

void CalcluateOdometryCallback(const geometry_msgs::TwistStamped& msg_in){

  nav_msgs::Odometry msg_out = nav_msgs::Odometry();

  //Reset position if the bag restarts
  if(last_time > msg_in.header.stamp.toSec())
  {
    last_time = msg_in.header.stamp.toSec();
    SetInitialPositions(); //TO BE REMOVED
    return;
  }

  //Integrate in the modality specified by the parameter 
  switch (integration_mode)
  {
    case 0:
      Euler(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    break;
    case 1:
      RungeKutta(msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.angular.z, msg_in.header.stamp.toSec() - last_time);
    break;
    default:
      ROS_INFO("integration mode not implemented");
    break;
  }

  last_time = msg_in.header.stamp.toSec();

  //Setup message out
  msg_out.header.seq = msg_in.header.seq;
  msg_out.header.stamp = msg_in.header.stamp;
  msg_out.header.frame_id = "odom";
  msg_out.pose.pose.position.x = last_x;
  msg_out.pose.pose.position.y = last_y;
  msg_out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_theta);

  //Publish odom
  pub_odom.publish(msg_out);

  //BroadcastTF(msg_out.twist.linear.x, msg_out.twist.linear.y, msg_out.twist.angular.z, msg_in.header.stamp);

  ROS_INFO("x: %f, y: %f, theta: %f", last_x, last_y, last_theta);
}


void BroadcastTF(float x, float y, float th,  ros::Time timeStamp)
{
  //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

  tf::TransformBroadcaster odom_broadcaster;

  //publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = timeStamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
}
bool reset(project1::Reset::Request &req, project1::Reset::Response &res)
{
    x = req.new_x;
    y = req.new_y;
    theta = req.new_theta;
    ROS_INFO("Odom has been set to ([%f],[%f],[%f])", req.new_x, req.new_y, req.new_theta);
    return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_odometry");

  ros::NodeHandle nh;

  nh.getParam("/initial_x", initial_x);
  nh.getParam("/initial_y", initial_y);
  nh.getParam("/initial_theta", initial_theta);

  //Subscribe to cmd_vel
  ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 1, &CalcluateOdometryCallback);

  //Setup publisher odom
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  set_srv = nh.advertiseService("set_odom", &reset);
  ros::Rate rate(1000);

  //Dynamic Reconfigure
  dynamic_reconfigure::Server<project1::parametersConfig> server;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

  f = boost::bind(&DynamicReconfigureCallback, _1, _2);
  server.setCallback(f);


  ROS_INFO("ODOMETRY NODE");
  ros::spin();
}