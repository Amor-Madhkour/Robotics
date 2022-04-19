#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Time.h>

#define PI 3.1415

const float r = 0.7;
const float l = 0.2;
const float w = 0.169;
const int T = 5;
const int N = 42;
const int N_WHEELS = 4;

double last_time = 1.7976931348623157E+308; //Time initialized to infinity
float last_ticks[4];

ros::Publisher pub_vel;



void UpdateLastValues(const sensor_msgs::JointState& msg_in){

  last_time = msg_in.header.stamp.toSec();

  for (int i = 0; i < N_WHEELS; i++) {
    last_ticks[i] = msg_in.position[i];
  }
  
}

float CalculateVelocityFromTickDelta(float deltaTick, float deltaTime)
{
  return (2 * PI * (deltaTick / deltaTime) / N) / T;
}


void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

  geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();

  //Initial setup - executed only at the beginning of the bag
  if(last_time > msg_in.header.stamp.toSec())
  {
    UpdateLastValues(msg_in);
  }
  else
  {
    //Wheels order: fl, fr, bl, br

    //Calculate wheels angular velocities
    float wheels_velocity[4];
    for (int i = 0; i < N_WHEELS; i++) {
      wheels_velocity[i] = CalculateVelocityFromTickDelta(msg_in.position[i] - last_ticks[i], msg_in.header.stamp.toSec() - last_time);
    }

    float vx = r * (wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] + wheels_velocity[3]) / 4;

    float vy = r * (- wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] - wheels_velocity[3]) / 4;

    float K = l + w;
    float omega = r * (- wheels_velocity[0] + wheels_velocity[1] - wheels_velocity[2] + wheels_velocity[3]) / (4 * K);

    //Setup message out
    msg_out.header.stamp = msg_in.header.stamp;
    msg_out.twist.linear.x = vx;
    msg_out.twist.linear.y = vy;
    msg_out.twist.angular.z = omega;

    //Publish message out
    pub_vel.publish(msg_out);

    ROS_INFO("Vx: %f, Vy: %f, omega: %f", vx, vy, omega);
    //ROS_INFO("T. prev: %f, tick prev: %f", last_time, last_ticks_fl);

    UpdateLastValues(msg_in);
  }
}



int main(int argc, char **argv){

  ros::init(argc, argv, "calculate_velocity");
  ros::NodeHandle nh;

  ros::Subscriber sub_wheels = nh.subscribe("/wheel_states", 1000, &CalcluateVelocityCallback);
  
  pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
  ros::Rate rate(1);


  ROS_INFO("VELOCITY NODE");
  while(ros::ok()){

    ros::spinOnce();

  }
  
}