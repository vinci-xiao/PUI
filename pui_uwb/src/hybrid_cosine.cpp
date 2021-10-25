#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <math.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_msgs/msg/tf_message.hpp"

#include "pui_msgs/msg/multi_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;  

double x1_, y1_, z1_;
double x2_, y2_, z2_;  // defaust x2_ should be same as x3_ 
double x3_, y3_, z3_;
double base_anchor_;
double length_platform_;
/////////////////////////////////////////////////////////////////////////////////////
class HybridCosine : public rclcpp::Node
{
public:
    HybridCosine()
    : Node("hybrid_cosine")
    {
        tag_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("tag_position_3d", 10);
        uwb_subscriber_ = this->create_subscription<pui_msgs::msg::MultiRange>(
            "uwb_range", 10, std::bind(&HybridCosine::uwb_callback, this, std::placeholders::_1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        get_static_tf();
    }
private:
    void get_static_tf(void)
    {
        geometry_msgs::msg::TransformStamped anchor_0_tf;
        geometry_msgs::msg::TransformStamped anchor_1_tf;
        geometry_msgs::msg::TransformStamped anchor_2_tf;

        rclcpp::Time now = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Listening static tf...");

        anchor_0_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_0_link", now, 1s);

        anchor_1_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_1_link", now, 1s);
    
        anchor_2_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_2_link", now, 1s);

        x1_ = anchor_0_tf.transform.translation.x;
        y1_ = anchor_0_tf.transform.translation.y;
        z1_ = anchor_0_tf.transform.translation.z;

        x2_ = anchor_1_tf.transform.translation.x;
        y2_ = anchor_1_tf.transform.translation.y;
        z2_ = anchor_1_tf.transform.translation.z;

        x3_ = anchor_2_tf.transform.translation.x;
        y3_ = anchor_2_tf.transform.translation.y;
        z3_ = anchor_2_tf.transform.translation.z;

        // base_calculation (anchor1 <-->anchor2)
        base_anchor_ = sqrtl(pow((x2_-x3_),2)+pow((y2_-y3_),2)+pow((z2_-z3_),2));
        length_platform_ = fabs(x1_ - x2_);

        RCLCPP_INFO(this->get_logger(), "anchor_0: %lf, %lf, %lf", x1_, y1_, z1_);
        RCLCPP_INFO(this->get_logger(), "anchor_1: %lf, %lf, %lf", x2_, y2_, z2_);
        RCLCPP_INFO(this->get_logger(), "anchor_2: %lf, %lf, %lf", x3_, y3_, z3_);

    }
    double cosine_rule(double distance_a, double distance_b, double distance_c)
    {
        static double angle_alpha, fraction;

        fraction = (pow(distance_b,2)+pow(distance_c,2)-pow(distance_a,2))/(2*distance_b*distance_c);

        //limit check
        // if (fraction >= 1.0)
        // {
        //     fraction = 1.0; 
        //     RCLCPP_INFO(this->get_logger(), "Out Of Limit! fraction = 1.0");
        // }
        // else if(fraction <= -1.0)
        // {
        //     fraction = -1.0; 
        //     RCLCPP_INFO(this->get_logger(), "Out Of Limit! fraction = -1.0");
        // }

        angle_alpha = acos(fraction);
        return angle_alpha;
    }
    void uwb_callback(pui_msgs::msg::MultiRange::SharedPtr msg) 
    {
        static auto p = geometry_msgs::msg::Pose();
        static double r1,r2,r3 = 0;

        r1= msg->ranges[0];
        r2= msg->ranges[1];
        r3= msg->ranges[2];

        p.position.y = y2_ + r3*cos(cosine_rule(r2, r3, base_anchor_));
        p.position.z = z1_; // default same height with anchor0

        if ((r1-r3) >= length_platform_)
        {
            p.position.x = x2_ + r3*sin(cosine_rule(r2, r3, base_anchor_));
        }
        else if((r1-r3) < length_platform_)
        {
            p.position.x = x2_ - r3*sin(cosine_rule(r2, r3, base_anchor_));
        }

        tag_publisher_->publish(p);
    }
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tag_publisher_;
    rclcpp::Subscription<pui_msgs::msg::MultiRange>::SharedPtr uwb_subscriber_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridCosine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}