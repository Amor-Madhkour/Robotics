#include "data.h"

#include <ros/ros.h>
#include <std_msgs/Time.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

//  //======== PARAMETERS CALIBRATION =========
//#include <dynamic_reconfigure/server.h>
//#include <project1/parametersConfig.h>
//  //=========================================


class vel_pub 
{

// ================ ATTRIBUTES ================
private:

  //ROS
  ros::NodeHandle nh;
  ros::Publisher pub_vel;
  ros::Subscriber sub_wheels;

  //Calculations
  double last_time;
  float last_ticks[4] = {0, 0, 0, 0};

  //Utility
  //const int noise_remover = 2;
  //int skip_counter = 0;
  //float ticks_avg[4];
  //float r, l, w, N; //PARAMETERS RECALIBRATION

// =============== CONSTRUCTOR ===============
public:

  vel_pub(){

    last_time = 0;

    sub_wheels = nh.subscribe("/wheel_states", 1, &vel_pub::CalcluateVelocityCallback, this);
    pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

  }



// ================ METHODS ==================

  //void ResetTickAvg(){
  //  for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
  //    ticks_avg[i] = 0;
  //}

  void UpdateLastValues(const sensor_msgs::JointState& msg_in){

    last_time = ros::Time::now().toSec();

    for (int i = 0; i < N_WHEELS; i++) {
      last_ticks[i] = msg_in.position[i];
    }
    
  }

  float CalculateVelocityFromTickDelta(float deltaTick, double deltaTime){
    return (2 * PI * (deltaTick / deltaTime) / N_ENCODER) / GEAR_RATIO;
  }



// ================ CALLBACK ==================

  void CalcluateVelocityCallback(const sensor_msgs::JointState& msg_in){

    // ============= PARAMETERS CALIBRATION =============
    // Sappiamo che è sconsigliato utilizzare il getParam all'interno dei callback
    // ma siccome e una parte di codice che abbiamo dovuto commentare e scommentare 
    // spesso abbiamo dato più importanza alla compattezza e alla leggibilità,
    // piuttosto che alle good practices, considerando anche il fatto che nella prodotto
    // finale queste righe sarebbero state rimosse
    //nh.getParam("/dynamic_reconfigure/r", r);
    //nh.getParam("/dynamic_reconfigure/l", l);
    //nh.getParam("/dynamic_reconfigure/w", w);
    //nh.getParam("/dynamic_reconfigure/N", N);
    //ROS_INFO("%f, %f, %f, %f", r, l, w, N);
    // ==================================================

    //============== Decrease Noise (unused) ============
    //Rimosso in quanto induce un ritardo indesiderato
    //if(skip_counter < noise_remover)
    //{
    //  for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
    //    ticks_avg[i] = (ticks_avg[i] * skip_counter + msg_in.position[i]) / (skip_counter + 1); //avg di <noise_remover> ticks -> leggero smoothing
    //
    //  skip_counter++;
    //  return;
    //}
    //ResetTickAvg();
    //skip_counter = 0;
    //===================================================


    //Skipped only at the beginning of the bag
    if(last_time != 0)
    {
      //Calculate wheels angular velocities - Wheels order: fl, fr, rl, rr
      float wheels_velocity[4];
      for (int i = 0; i < N_WHEELS; i++) {
        wheels_velocity[i] = CalculateVelocityFromTickDelta(msg_in.position[i] - last_ticks[i], ros::Time::now().toSec() - last_time);
        //wheels_velocity[i] = msg_in.velocity[i] / GEAR_RATE / 60; //SOLO DEBUG
      }

      float vx = WHEEL_RADIUS * (wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] + wheels_velocity[3]) / 4;
      float vy = WHEEL_RADIUS * (- wheels_velocity[0] + wheels_velocity[1] + wheels_velocity[2] - wheels_velocity[3]) / 4;
      float omega = WHEEL_RADIUS * (- wheels_velocity[0] + wheels_velocity[1] - wheels_velocity[2] + wheels_velocity[3]) / (4 * (L_LENGTH + W_LENGTH));

      //Setup message out
      geometry_msgs::TwistStamped msg_out = geometry_msgs::TwistStamped();
      msg_out.header.seq = msg_in.header.seq;
      msg_out.header.stamp = ros::Time::now();
      msg_out.header.frame_id = msg_in.header.frame_id;
      msg_out.twist.linear.x = vx;
      msg_out.twist.linear.y = vy;
      msg_out.twist.angular.z = omega;

      //Publish message out
      pub_vel.publish(msg_out);

      ROS_INFO("Vx: %f, Vy: %f, omega: %f", vx, vy, omega);
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