#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
using namespace std::chrono_literals;

class TagVISUAL : public rclcpp::Node
{
public:
    nav_msgs::msg::Path tag_record;
    geometry_msgs::msg::PoseStamped tag_pose_record;  

    nav_msgs::msg::Path odom_record;
    geometry_msgs::msg::PoseStamped odom_pose_record;  

    TagVISUAL()
    : Node("tag_visual")
    {
        tag_pos_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "tag_position", 2, std::bind(&TagVISUAL::tag_callback, this, std::placeholders::_1));

        rst_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_tag_path", 1, std::bind(&TagVISUAL::reset_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 2, std::bind(&TagVISUAL::odom_callback, this, std::placeholders::_1));

        tag_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("tag_path", 3);

        odom_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 3);

        // marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_marker", 1);

        // timer_ = this->create_wall_timer(200ms, std::bind(&TagVISUAL::timer_callback, this));
    }
private:
    void tag_callback(geometry_msgs::msg::Pose::SharedPtr tag) 
    {
      tag_record.header.frame_id = "world";
      tag_pose_record.header.frame_id = "world";
      tag_pose_record.pose.position = tag->position;
      tag_pose_record.pose.orientation= tag->orientation;
      tag_record.poses.push_back(tag_pose_record);

      tag_path_publisher_->publish(tag_record);
    }

    void reset_callback(std_msgs::msg::Bool::SharedPtr rst) 
    {
      if(rst->data == true)
      {
        tag_record.poses.clear();
        odom_record.poses.clear();
      }
      
    }

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr odom) 
    {
      odom_record.header.frame_id = "world";
      odom_pose_record.header.frame_id = "world";
      odom_pose_record.pose.position = odom->pose.pose.position;
      // odom_pose_record.pose.orientation = odom->pose.pose.orientation;
      odom_pose_record.pose.orientation.x=0.00;
      odom_pose_record.pose.orientation.y=0.00;
      odom_pose_record.pose.orientation.z=0.00;
      odom_pose_record.pose.orientation.w=1.00;
      odom_record.poses.push_back(odom_pose_record);

      odom_path_publisher_->publish(odom_record);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tag_pos_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rst_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tag_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_publisher_;

    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagVISUAL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}