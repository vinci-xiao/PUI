#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

class TrilaterationMSE : public rclcpp::Node
{
public:
    TrilaterationMSE()
    : Node("trilateration_mse")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("tag_position", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "uwb_range", 10, std::bind(&TrilaterationMSE::topic_callback, this, std::placeholders::_1));
    }
private:
    void topic_callback(geometry_msgs::msg::Pose::SharedPtr msg) 
    {
    //   p.position.x= 1;
    //   p.position.y= 2;
    //   p.position.z= 3;
      publisher_->publish(msg->position.x);
    }
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrilaterationMSE>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}