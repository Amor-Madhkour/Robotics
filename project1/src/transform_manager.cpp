#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tf2_sub_pub
{
private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    ros::Subscriber sub;
    tf2::Quaternion q;
    
public:
    tf2_sub_pub()
    {
        sub = n.subscribe("/odom", 1000, &tf2_sub_pub::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& my_odom)
    {
        // set header
        odom_trans.header.stamp = my_odom->header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link"; 
        // set x,y
        odom_trans.transform.translation.x = my_odom->pose.pose.position.x;
        odom_trans.transform.translation.y = my_odom->pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        // set theta
        q.setRPY(0, 0, my_odom.theta);
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        odom_broadcaster.sendTransform(odom_trans);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_node");
    tf2_sub_pub my_tf2_sub_pub;
    ros::spin();
    return 0;
}