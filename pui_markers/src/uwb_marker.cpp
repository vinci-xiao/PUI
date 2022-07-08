#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

float_t uwb_dimension_x = 0.04;
float_t uwb_dimension_y = 0.01;
float_t uwb_dimension_z = 0.07;

class MarkerUWB : public rclcpp::Node
{
public:
    MarkerUWB()
    : Node("uwb_marker")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 3);
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "tag_position", 10, std::bind(&MarkerUWB::tag_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MarkerUWB::timer_callback, this));
    }
private:
    void anchor_setting()
    {
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::msg::Marker::CUBE;

        // anchor_0
        visualization_msgs::msg::Marker anchor_0;
        anchor_0.header.frame_id = "/world";
        anchor_0.header.stamp = rclcpp::Clock().now();
        anchor_0.ns = "uwb_marker";
        anchor_0.id = 0;
        anchor_0.type = shape;
        anchor_0.action = visualization_msgs::msg::Marker::ADD;
        anchor_0.pose.position.x = -6.5;
        anchor_0.pose.position.y = 1.0;
        anchor_0.pose.position.z = 0;
        anchor_0.pose.orientation.x = 0.0;
        anchor_0.pose.orientation.y = 0.0;
        anchor_0.pose.orientation.z = 0.0;
        anchor_0.pose.orientation.w = 1.0;
        anchor_0.scale.x = 0.3;
        anchor_0.scale.y = 0.3;
        anchor_0.scale.z = 0.3;
        anchor_0.color.r = 0.0f;
        anchor_0.color.g = 1.0f;
        anchor_0.color.b = 0.0f;
        anchor_0.color.a = 1.0;
        // anchor_0.lifetime = rclcpp::Duration();

        // anchor_1
        visualization_msgs::msg::Marker anchor_1;
        anchor_1.header.frame_id = "/world";
        anchor_1.header.stamp = rclcpp::Clock().now();
        anchor_1.ns = "uwb_marker";
        anchor_1.id = 1;
        anchor_1.type = shape;
        anchor_1.action = visualization_msgs::msg::Marker::ADD;
        anchor_1.pose.position.x = -11.5;
        anchor_1.pose.position.y = 1.0;
        anchor_1.pose.position.z = 0;
        anchor_1.pose.orientation.x = 0.0;
        anchor_1.pose.orientation.y = 0.0;
        anchor_1.pose.orientation.z = 0.0;
        anchor_1.pose.orientation.w = 1.0;
        anchor_1.scale.x = 0.3;
        anchor_1.scale.y = 0.3;
        anchor_1.scale.z = 0.3;
        anchor_1.color.r = 1.0f;
        anchor_1.color.g = 0.0f;
        anchor_1.color.b = 0.0f;
        anchor_1.color.a = 1.0;
        // anchor_1.lifetime = ros::Duration();

        // anchor_2
        visualization_msgs::msg::Marker anchor_2;
        anchor_2.header.frame_id = "/world";
        anchor_2.header.stamp = rclcpp::Clock().now();
        anchor_2.ns = "uwb_marker";
        anchor_2.id = 2;
        anchor_2.type = shape;
        anchor_2.action = visualization_msgs::msg::Marker::ADD;
        anchor_2.pose.position.x = -7.5;
        anchor_2.pose.position.y = -4.0;
        anchor_2.pose.position.z = 0;
        anchor_2.pose.orientation.x = 0.0;
        anchor_2.pose.orientation.y = 0.0;
        anchor_2.pose.orientation.z = 0.0;
        anchor_2.pose.orientation.w = 1.0;
        anchor_2.scale.x = 0.3;
        anchor_2.scale.y = 0.3;
        anchor_2.scale.z = 0.3;
        anchor_2.color.r = 0.0f;
        anchor_2.color.g = 0.0f;
        anchor_2.color.b = 1.0f;
        anchor_2.color.a = 1.0;
        // anchor_2.lifetime = ros::Duration();

        publisher_->publish(anchor_0);
        publisher_->publish(anchor_1);
        publisher_->publish(anchor_2);
    }
    void tag_callback(geometry_msgs::msg::Pose::SharedPtr tag) 
    {
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::msg::Marker::SPHERE;

        visualization_msgs::msg::Marker tag_marker;
        tag_marker.header.frame_id = "/world";
        tag_marker.header.stamp = rclcpp::Clock().now();
        tag_marker.ns = "tag_mark";
        tag_marker.id = 0;
        tag_marker.type = shape;
        tag_marker.action = visualization_msgs::msg::Marker::ADD;
        tag_marker.pose.position.x = tag->position.x;
        tag_marker.pose.position.y = tag->position.y;
        tag_marker.pose.position.z = tag->position.z;           
        tag_marker.pose.orientation.x = 0.0;
        tag_marker.pose.orientation.y = 0.0;
        tag_marker.pose.orientation.z = 0.0;
        tag_marker.pose.orientation.w = 1.0;
        tag_marker.scale.x = 0.3;
        tag_marker.scale.y = 0.3;
        tag_marker.scale.z = 0.3;
        tag_marker.color.r = 1.0f;
        tag_marker.color.g = 0.0f;
        tag_marker.color.b = 1.0f;
        tag_marker.color.a = 1.0;
        // tag_marker.lifetime = ros::Duration();

        publisher_->publish(tag_marker);
    }
    void timer_callback()
    {
        anchor_setting();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerUWB>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}