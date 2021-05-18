#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"

ros::Publisher marker_pub;
ros::Subscriber tag_sub;

//-----------------------------------------------------------------------------------------------
void Tag_Callback(const geometry_msgs::Pose& tag)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker tag_marker;
    tag_marker.header.frame_id = "/world";
    tag_marker.header.stamp = ros::Time::now();
    tag_marker.ns = "tag_mark";
    tag_marker.id = 0;
    tag_marker.type = shape;
    tag_marker.action = visualization_msgs::Marker::ADD;
    tag_marker.pose.position.x = tag.position.x;
    tag_marker.pose.position.y = tag.position.y;
    tag_marker.pose.position.z = tag.position.z;    
    tag_marker.pose.orientation.x = 0.0;
    tag_marker.pose.orientation.y = 0.0;
    tag_marker.pose.orientation.z = 0.0;
    tag_marker.pose.orientation.w = 1.0;
    tag_marker.scale.x = 0.15;
    tag_marker.scale.y = 0.15;
    tag_marker.scale.z = 0.15;
    tag_marker.color.r = 1.0f;
    tag_marker.color.g = 0.0f;
    tag_marker.color.b = 1.0f;
    tag_marker.color.a = 1.0;
    tag_marker.lifetime = ros::Duration();
    marker_pub.publish(tag_marker);
}

//-----------------------------------------------------------------------------------------------
int main( int argc, char** argv )
{
  ros::init(argc, argv, "tag_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("tag_marker", 5);
  tag_sub = n.subscribe("tag_position", 5, Tag_Callback);

  ros::spin();
  return 0;
}
