# Robotics - Project 1

## Students
- Gabrielli Simone, 10679234
- Madhkour Amor, 10794343
- Vedovato Greta, 10724294

## Files description
#### /src
- **odom_pub.cpp**
	1. Compute odometry from v and ω with Eurler or Runge Kutta;
	1. Publish nav_msgs/Odometry on topic /odom;
	1. Broadcast TF odom->base_link;
	1. Service to reset odom to a given pose;
	1. Dynamic reconfigure to select between Euler and Runge Kutta integration.

- **vel_pub.cpp**
	1. Compute v and ω from wheel ticks through the following formulas:
	~~(Scrivi formule)~~
	1. Publish v and ⍵ as topic /cmd_vel of type geometry_msgs/TwistStamped.

- **wheelRPM_pub.cpp**
	1. Read v and ⍵ from /cmd_vel and computes the speed of each wheel (in RPM) through the following formulas:
	~~(scrivi formula)~~

------------


#### /cfg
- **parameters.cfg**
Defines the enum integration_mode, used for dynamic reconfigure. It used to define also the parameters to calibrate, but they have been commented after the calibration.

------------


#### /include
- **data.h**
Stores the calibrated parameters.

------------


#### /msg
- **StampedWheelRPM.msg**
Defines the message containing the speed of the wheels.

------------

#### /srv
- **reset_odom.srv**
Defines the odom reset service parameters.

## ROS Parameters
- **/initial_x**: the x of the frame odom at the beginning of the bag.
- **/initial_y**: the y of the frame odom at the beginning of the bag.
- **/initial_theta**: the theta of the frame odom at the beginning of the bag.
- **/integration_mode**: the enumerator that defines the integration mode. if equals to 0 the integration is EULER, otherwise it is RUNGE KUTTA.

## TF Tree
TODO

## Custom messages and services
 - **StampedWheelRPM.msg**

		Header header
		float64 rpm_fl
		float64 rpm_fr
		float64 rpm_rl
		float64 rpm_rr

- **reset_odom.srv**

		float64 new_x
		float64 new_y
		float64 new_theta
		
		---

## How to start the nodes
(TODO)


## Extra notes
The parameters have been calibrated thorugh dynamic_reconfigure and plotjuggler. We tried to variate the r,N,w,l in runtime while running tha bag and tried to make the plot of our /odom as similar as we could to the ground truth (/robot/pose).




