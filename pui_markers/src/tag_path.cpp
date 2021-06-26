#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"

class TagPATH : public rclcpp::Node
{
public:
    nav_msgs::msg::Path path_record;
    geometry_msgs::msg::PoseStamped pose_record;  

    TagPATH()
    : Node("tag_path")
    {
        tag_pos_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "tag_position", 2, std::bind(&TagPATH::tag_callback, this, std::placeholders::_1));

        rst_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_tag_path", 2, std::bind(&TagPATH::reset_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_record", 3);
    }
private:
    void tag_callback(geometry_msgs::msg::Pose::SharedPtr tag) 
    {
      path_record.header.frame_id = "/world";
      pose_record.header.frame_id = "/world";
      pose_record.pose.position = tag->position;
      pose_record.pose.orientation = tag->orientation;
      path_record.poses.push_back(pose_record);

      publisher_->publish(path_record);
    }
    void reset_callback(std_msgs::msg::Bool::SharedPtr rst) 
    {
      if(rst->data==true)
      path_record.poses.clear();
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tag_pos_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rst_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagPATH>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}