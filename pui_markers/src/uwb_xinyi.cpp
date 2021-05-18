#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "uwb_xinyi");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 3);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  float_t uwb_dimension_x = 0.04;
  float_t uwb_dimension_y = 0.01;
  float_t uwb_dimension_z = 0.07;
  //from ground to top 25cm- top to middle of chip 3.5cm
  //float_t uwb_height = 0.25-(uwb_dimension_z/2); 

  while (ros::ok())
  {
    // anchor_0
    visualization_msgs::Marker anchor_0;
    anchor_0.header.frame_id = "/world";
    anchor_0.header.stamp = ros::Time::now();
    anchor_0.ns = "uwb_xinyi";
    anchor_0.id = 0;
    anchor_0.type = shape;
    anchor_0.action = visualization_msgs::Marker::ADD;
    anchor_0.pose.position.x = 0;
    anchor_0.pose.position.y = 0;
    anchor_0.pose.position.z = 0;
    anchor_0.pose.orientation.x = 0.0;
    anchor_0.pose.orientation.y = 0.0;
    anchor_0.pose.orientation.z = 0.0;
    anchor_0.pose.orientation.w = 1.0;
    anchor_0.scale.x = 0.1;
    anchor_0.scale.y = 0.1;
    anchor_0.scale.z = 0.1;
    anchor_0.color.r = 0.0f;
    anchor_0.color.g = 1.0f;
    anchor_0.color.b = 0.0f;
    anchor_0.color.a = 1.0;
    anchor_0.lifetime = ros::Duration();

    // anchor_1
    visualization_msgs::Marker anchor_1;
    anchor_1.header.frame_id = "/world";
    anchor_1.header.stamp = ros::Time::now();
    anchor_1.ns = "uwb_xinyi";
    anchor_1.id = 1;
    anchor_1.type = shape;
    anchor_1.action = visualization_msgs::Marker::ADD;
    anchor_1.pose.position.x = 0;
    anchor_1.pose.position.y = 3.406;
    anchor_1.pose.position.z = 0;
    anchor_1.pose.orientation.x = 0.0;
    anchor_1.pose.orientation.y = 0.0;
    anchor_1.pose.orientation.z = 1.0;
    anchor_1.pose.orientation.w = 1.0;
    anchor_1.scale.x = 0.1;
    anchor_1.scale.y = 0.1;
    anchor_1.scale.z = 0.1;
    anchor_1.color.r = 1.0f;
    anchor_1.color.g = 0.0f;
    anchor_1.color.b = 0.0f;
    anchor_1.color.a = 1.0;
    anchor_1.lifetime = ros::Duration();

    // anchor_2
    visualization_msgs::Marker anchor_2;
    anchor_2.header.frame_id = "/world";
    anchor_2.header.stamp = ros::Time::now();
    anchor_2.ns = "uwb_xinyi";
    anchor_2.id = 2;
    anchor_2.type = shape;
    anchor_2.action = visualization_msgs::Marker::ADD;
    anchor_2.pose.position.x = 2.95;
    anchor_2.pose.position.y = 1.703;
    anchor_2.pose.position.z = 0;
    anchor_2.pose.orientation.x = 0.0;
    anchor_2.pose.orientation.y = 0.0;
    anchor_2.pose.orientation.z = 1;
    anchor_2.pose.orientation.w = 1.0;
    anchor_2.scale.x = 0.1;
    anchor_2.scale.y = 0.1;
    anchor_2.scale.z = 0.1;
    anchor_2.color.r = 0.0f;
    anchor_2.color.g = 0.0f;
    anchor_2.color.b = 1.0f;
    anchor_2.color.a = 1.0;
    anchor_2.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(anchor_0);
    marker_pub.publish(anchor_1);
    marker_pub.publish(anchor_2);

    // Cycle between different shapes
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }

    r.sleep();
  }
}
