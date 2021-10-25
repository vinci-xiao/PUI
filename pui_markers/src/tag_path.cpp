#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
using namespace std::chrono_literals;

class TagVISUAL : public rclcpp::Node
{
public:
    nav_msgs::msg::Path path_record;
    geometry_msgs::msg::PoseStamped pose_record;  

    TagVISUAL()
    : Node("tag_visual")
    {
        tag_pos_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "tag_position", 2, std::bind(&TagVISUAL::tag_callback, this, std::placeholders::_1));

        rst_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_tag_path", 1, std::bind(&TagVISUAL::reset_callback, this, std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_record", 3);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_marker", 1);

        timer_ = this->create_wall_timer(200ms, std::bind(&TagVISUAL::timer_callback, this));
    }
private:
    void tag_callback(geometry_msgs::msg::Pose::SharedPtr tag) 
    {
      path_record.header.frame_id = "/base_link";
      pose_record.header.frame_id = "/base_link";
      pose_record.pose.position = tag->position;
      pose_record.pose.orientation = tag->orientation;
      path_record.poses.push_back(pose_record);

      path_publisher_->publish(path_record);
    }
    void reset_callback(std_msgs::msg::Bool::SharedPtr rst) 
    {
      if(rst->data == true)
      path_record.poses.clear();
    }
    void timer_callback()
    {
      static uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;
      static visualization_msgs::msg::Marker tag_marker;
      tag_marker.header.frame_id = "/base_link";
      tag_marker.header.stamp = rclcpp::Clock().now();
      tag_marker.ns = "tag_mark";
      tag_marker.id = 0;
      tag_marker.type = shape;
      tag_marker.action = visualization_msgs::msg::Marker::ADD;
      tag_marker.pose.position = pose_record.pose.position;          
      tag_marker.pose.orientation = pose_record.pose.orientation; 
      tag_marker.scale.x = 0.15;
      tag_marker.scale.y = 0.15;
      tag_marker.scale.z = 0.15;
      tag_marker.color.r = 1.0f;
      tag_marker.color.g = 0.0f;
      tag_marker.color.b = 1.0f;
      tag_marker.color.a = 1.0;
      marker_publisher_->publish(tag_marker);

    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tag_pos_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rst_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagVISUAL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}