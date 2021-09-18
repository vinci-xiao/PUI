#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>


class Odometry : public rclcpp::Node
{
public:
    // nav_msgs::msg::Path path_record;
    // geometry_msgs::msg::PoseStamped pose_record;  

    Odometry()
    : Node("odometry")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 2, std::bind(&Odometry::imu_callback, this, std::placeholders::_1));

        wheel_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
        "reset_tag_path", 2, std::bind(&Odometry::wheel_callback, this, std::placeholders::_1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 3);
    }
private:
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu) 
    {
    //   path_record.header.frame_id = "/world";
    //   pose_record.header.frame_id = "/world";
    //   pose_record.pose.position = tag->position;
    //   pose_record.pose.orientation = tag->orientation;
    //   path_record.poses.push_back(pose_record);

    //   publisher_->publish(path_record);
    }
    void wheel_callback(std_msgs::msg::Int8::SharedPtr data) 
    {
    //   if(rst->data==true)
    //   path_record.poses.clear();
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr wheel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Odometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}