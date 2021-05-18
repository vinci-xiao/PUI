#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

// #include <vector>

ros::Publisher tag_path_pub_;
ros::Subscriber tag_sub_,reset_sub_;
void tag_Cb(const geometry_msgs::Pose& tag);
void reset_Cb(const std_msgs::Bool& data);

nav_msgs::Path path;
geometry_msgs::PoseStamped pose;

// void tag_Cb(const nav_msgs::Odometry& data)
void tag_Cb(const geometry_msgs::Pose& tag)
{
    path.header.frame_id = "/world";
    pose.header.frame_id = "/world";
    pose.pose = tag;
    path.poses.push_back(pose);
    tag_path_pub_.publish(path);
}

void reset_Cb(const std_msgs::Bool& data)
{
    path.poses.clear();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "show_tag_path");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tag_sub_ = nh.subscribe("tag_position", 2, tag_Cb);
  reset_sub_ = nh.subscribe("reset_tag_path", 2, reset_Cb);
  tag_path_pub_ = nh.advertise<nav_msgs::Path>("tag_path", 6);

  ros::spin();
  return 0;
}