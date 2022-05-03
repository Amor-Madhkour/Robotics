#include <ros/ros.h>
#include "data.h"
#include "project1/Odom.h"
#include <geometry_msgs/TwistStamped.h>
//#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Time.h>

//  //======== PARAMETERS CALIBRATION =========
//#include <dynamic_reconfigure/server.h>
//#include <project1/parametersConfig.h>
//  //=========================================


class vel_pub 
{
  //float r, l, w, N; //REMOVE

// ================ ATTRIBUTES ================
private:

  //ROS
  ros::NodeHandle nh;
  ros::Publisher pub_vel;
  ros::Subscriber sub_wheels;
  //TODO REMOVE
  //ros::Publisher pub_w;
  //TODO REMOVE

  //Calculations
  double last_time = 1.7976931348623157E+308; //Time initialized to infinity: this logic allows to play a new bag without restarting this node
  float last_ticks[4] = {0, 0, 0, 0};

  //Utility
  const int noise_remover = 3;
  int skip_counter = 0;
  float ticks_avg[4];


// =============== CONSTRUCTOR ===============
public:

  vel_pub(){
    sub_wheels = nh.subscribe("/wheel_states", 1, &vel_pub::CalcluateVelocityCallback, this);
    pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

    //TODO REMOVE
    //pub_w = nh.advertise<geometry_msgs::TwistStamped>("/wheel_vel_test", 1);
    //TODO REMOVE
  }


// ================ METHODS ==================
  void ResetTickAvg(){
    for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
      ticks_avg[i] = 0;
  }

  void UpdateLastValues(const sensor_msgs::JointState& msg_in){

    last_time = msg_in.header.stamp.toSec();

    for (int i = 0; i < N_WHEELS; i++) {
      last_ticks[i] = msg_in.position[i];
    }
    
  }

  float CalculateVelocityFromTickDelta(float deltaTick, double deltaTime){
    return (2 * PI * (deltaTick / deltaTime) / N_ENCODER) / GEAR_RATIO;
  }


  void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

    // ============= PARAMETERS CALIBRATION =============
    //nh.getParam("/integration_mode/r", r);
    //nh.getParam("/integration_mode/l", l);
    //nh.getParam("/integration_mode/w", w);
    //nh.getParam("/integration_mode/N", N);
    //ROS_INFO("%f, %f, %f, %f", r, l, w, N);
    // ==================================================

    //============== Decrease Noise ==============
    if(skip_counter < noise_remover)
    {
      for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
        ticks_avg[i] = (ticks_avg[i] * skip_counter + msg_in.position[i]) / (skip_counter + 1); //avg di <noise_remover> ticks -> leggero smoothing

      skip_counter++;
      return;
    }
    ResetTickAvg();
    skip_counter = 0;
    //============================================

    geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();

    //Skipped only at the beginning of the bag
    if(last_time < msg_in.header.stamp.toSec())
    {
      //Calculate wheels angular velocities - Wheels order: fl, fr, rl, rr
      float wheels_velocity[4];
      for (int i = 0; i < N_WHEELS; i++) {
        wheels_velocity[i] = CalculateVelocityFromTickDelta(msg_in.position[i] - last_ticks[i], msg_in.header.stamp.toSec() - last_time);
        //wheels_velocity[i] = msg_in.velocity[i] / T / 60; //SOLO DEBUG
      }

      float vx = WHEEL_RADIUS * (wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] + wheels_velocity[3]) / 4;

      float vy = WHEEL_RADIUS * (- wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] - wheels_velocity[3]) / 4;
      //float vy = r * (- wheels_velocity[0] + wheels_velocity[1] - wheels_velocity[2] + wheels_velocity[3]) / 4;

      float omega = WHEEL_RADIUS * (- wheels_velocity[0] + wheels_velocity[1] - wheels_velocity[2] + wheels_velocity[3]) / (4 * (L_LENGTH + W_LENGTH));
      //float omega = r * (- wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] - wheels_velocity[3]) / (4 * (l + w));

      //Setup message out
      msg_out.header.seq = msg_in.header.seq;
      msg_out.header.stamp = msg_in.header.stamp;
      msg_out.header.frame_id = msg_in.header.frame_id;
      msg_out.twist.linear.x = vx;
      msg_out.twist.linear.y = vy;
      msg_out.twist.angular.z = omega;

      //Publish message out
      pub_vel.publish(msg_out);

      ROS_INFO("Vx: %f, Vy: %f, omega: %f", vx, vy, omega);
      //ROS_INFO("T. prev: %f, tick prev: %f", last_time, last_ticks_fl);


      //REMOVE
      //geometry_msgs::TwistStamped msg_out2 = geometry_msgs::TwistStamped();
      //msg_out2.twist.linear.x = wheels_velocity[0]*60*T;
      //msg_out2.twist.linear.y = wheels_velocity[1]*60*T;
      //msg_out2.twist.linear.z = wheels_velocity[2]*60*T;
      //msg_out2.twist.angular.z = wheels_velocity[3]*60*T;
      //pub_w.publish(msg_out2);
      //REMOVE
    }

      UpdateLastValues(msg_in);
  }
};

  int main(int argc, char **argv){

    ros::init(argc, argv, "calculate_velocity");
    vel_pub my_vel_pub;
    ROS_INFO("VELOCITY NODE");
    ros::spin();
  }