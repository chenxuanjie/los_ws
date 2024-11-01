#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_pub ;

void odomCallback(const nav_msgs::Odometry &msg) {

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //时间戳和父子坐标系
    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "wamv/base_link";

    //旋转
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
    //平移
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;

    br.sendTransform(transformStamped);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    pose.pose.pose.position.x = msg.pose.pose.position.x;
    pose.pose.pose.position.y = msg.pose.pose.position.y;
    pose.pose.pose.position.z = msg.pose.pose.position.z;

    pose.pose.pose.orientation.w = msg.pose.pose.orientation.w;
    pose.pose.pose.orientation.x = msg.pose.pose.orientation.x;
    pose.pose.pose.orientation.y = msg.pose.pose.orientation.y;
    pose.pose.pose.orientation.z = msg.pose.pose.orientation.z;

    pose_pub.publish(pose);

}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "pub_tf_map2base");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/wamv/sensors/position/p3d_wamv", 1, &odomCallback);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/wamv/pose", 1);
    ros::spin();
    return 0;
}