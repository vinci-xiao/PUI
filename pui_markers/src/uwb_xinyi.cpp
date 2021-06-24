#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

// Set our initial shape type to be a cube
uint32_t shape = visualization_msgs::msg::Marker::CUBE;

float_t uwb_dimension_x = 0.04;
float_t uwb_dimension_y = 0.01;
float_t uwb_dimension_z = 0.07;

class XinyiUWB : public rclcpp::Node
{
public:
    XinyiUWB()
    : Node("uwb_xinyi")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 3);

        timer_ = this->create_wall_timer(
        500ms, std::bind(&XinyiUWB::timer_callback, this));


    }
private:
    void timer_callback()
    {
        // anchor_0
        visualization_msgs::msg::Marker anchor_0;
        anchor_0.header.frame_id = "/world";
        anchor_0.header.stamp = rclcpp::Clock().now();
        anchor_0.ns = "uwb_xinyi";
        anchor_0.id = 0;
        anchor_0.type = shape;
        anchor_0.action = visualization_msgs::msg::Marker::ADD;
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
        // anchor_0.lifetime = rclcpp::Duration();

        // anchor_1
        visualization_msgs::msg::Marker anchor_1;
        anchor_1.header.frame_id = "/world";
        anchor_1.header.stamp = rclcpp::Clock().now();
        anchor_1.ns = "uwb_xinyi";
        anchor_1.id = 1;
        anchor_1.type = shape;
        anchor_1.action = visualization_msgs::msg::Marker::ADD;
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
        // anchor_1.lifetime = ros::Duration();

        // anchor_2
        visualization_msgs::msg::Marker anchor_2;
        anchor_2.header.frame_id = "/world";
        anchor_2.header.stamp = rclcpp::Clock().now();
        anchor_2.ns = "uwb_xinyi";
        anchor_2.id = 2;
        anchor_2.type = shape;
        anchor_2.action = visualization_msgs::msg::Marker::ADD;
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
        // anchor_2.lifetime = ros::Duration();

        publisher_->publish(anchor_0);
        publisher_->publish(anchor_1);
        publisher_->publish(anchor_2);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XinyiUWB>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}