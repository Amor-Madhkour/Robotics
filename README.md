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
	1. Compute v and ω from wheel ticks;
	1. Publish v and ⍵ as topic /cmd_vel of type geometry_msgs/TwistStamped.

- **wheelRPM_pub.cpp**
	1. Read v and ⍵ from /cmd_vel and computes the speed of each wheel (in RPM).
	1. Publish the computed wheel speed as custom message on topic /wheels_rpm

------------


#### /cfg
- **parameters.cfg**
Defines the enum integration_mode, used for dynamic reconfigure. It used to define also the parameters to calibrate, but they have been commented after the calibration.

------------


#### /include
- **data.h**
Stores the calibrated parameters (r, l, w, N) and other useful constants.

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
world->odom->base_link

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
Before running the bag it is necessary to uncomment the relative parameters initialization in the ros file:
e.g. before starting the BAG 1 uncomment the lines:

		<!-- BAG 1
		<param name="initial_x" value="0.008"/>
		<param name="initial_y" value="0.003"/>
		<param name="initial_theta" value="0.02"/>
	 	<node pkg="tf2_ros" type="static_transform_publisher" name="world" args="0.008 0.003 0 0 0 0.010 1 world odom"></node>
		-->
These lines allow to initialize the parameters initial_x, initial_y, initial_theta and define a static transform world-odom.


## Extra notes
The parameters have been calibrated thorugh dynamic reconfigure and plotjuggler. We tried to variate the r,N,w,l in runtime while running tha bag and tried to make the plot of our /odom as similar as we could to the ground truth (/robot/pose).




