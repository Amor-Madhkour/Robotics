#include "ros/ros.h"
#include "project1/Odom.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class tf_sub_pub
{
private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    ros::Subscriber sub;
    tf2::Quaternion myQuat;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

public:
    tf_sub_pub()
    {
        sub = n.subscribe("/my_odom", 1000, &tf_sub_pub::callback, this);
    }

    void callback(const project1::Odom::ConstPtr& my_odom)
    {
        odom_trans.header.stamp = my_odom->odom.header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link"; 
        odom_trans.transform.translation.x = my_odom->odom.pose.pose.position.x;
        odom_trans.transform.translation.y = my_odom->odom.pose.pose.position.y;
        odom_trans.transform.translation.z = my_odom->odom.pose.pose.position.z;
        odom_trans.transform.rotation.x = my_odom->odom.pose.pose.orientation.x;
        odom_trans.transform.rotation.y = my_odom->odom.pose.pose.orientation.y;
        odom_trans.transform.rotation.z = my_odom->odom.pose.pose.orientation.z;
        odom_trans.transform.rotation.w = my_odom->odom.pose.pose.orientation.w;
        odom_broadcaster.sendTransform(odom_trans);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_node");
    tf_sub_pub my_tf_sub_pub;
    ros::spin();
    return 0;
}