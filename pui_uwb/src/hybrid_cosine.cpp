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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_msgs/msg/tf_message.hpp"

#include "evpi_interfaces/msg/multi_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;  

double x1_, y1_, z1_;
double x2_, y2_, z2_;  // defaust x2_ should be same as x3_ 
double x3_, y3_, z3_;
double base_anchor_, length_an01_,length_an02_;
double angle_an1_, angle_an2_;
double length_platform_;
auto init_p = geometry_msgs::msg::PoseWithCovarianceStamped();

/////////////////////////////////////////////////////////////////////////////////////
class HybridCosine : public rclcpp::Node
{
public:
    HybridCosine()
    : Node("hybrid_cosine")
    {
        tag_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("tag_position", 3);
        uwb_subscriber_ = this->create_subscription<evpi_interfaces::msg::MultiRange>(
            "/uwb_range", 5, std::bind(&HybridCosine::uwb_callback, this, std::placeholders::_1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        initpose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/t2/initialpose", 1);

        get_static_tf();
        get_parameters();
    }
    void get_parameters(void)
    {
      this->declare_parameter<std::double_t>("/t2/initialpose/x", 0.0);
      this->declare_parameter<std::double_t>("/t2/initialpose/y", 0.0);
      this->declare_parameter<std::double_t>("/t2/initialpose/z", 0.0);
      this->declare_parameter<std::double_t>("/t2/initialpose/theta", 0.0);
      
      this->get_parameter("/t2/initialpose/x", init_p.pose.pose.position.x);
      this->get_parameter("/t2/initialpose/y", init_p.pose.pose.position.y);
      this->get_parameter("/t2/initialpose/z", init_p.pose.pose.position.z);
      this->get_parameter("/t2/initialpose/theta", init_p.pose.pose.orientation.w);

      RCLCPP_INFO(this->get_logger(), "init_pose: x=%lf",init_p.pose.pose.position.x);
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
        length_an01_ = sqrtl(pow((x1_-x2_),2)+pow((y1_-y2_),2)+pow((z1_-z2_),2));
        length_an02_ = sqrtl(pow((x1_-x3_),2)+pow((y1_-y3_),2)+pow((z1_-z3_),2));

        angle_an1_ = cosine_rule(length_an02_, length_an01_, base_anchor_);
        angle_an2_ = cosine_rule(length_an01_, length_an02_, base_anchor_);

        length_platform_ = fabs(x1_ - x2_);

        RCLCPP_INFO(this->get_logger(), "anchor_0: %lf, %lf, %lf", x1_, y1_, z1_);
        RCLCPP_INFO(this->get_logger(), "anchor_1: %lf, %lf, %lf", x2_, y2_, z2_);
        RCLCPP_INFO(this->get_logger(), "anchor_2: %lf, %lf, %lf", x3_, y3_, z3_);
        RCLCPP_INFO(this->get_logger(), "base_anchor_: %lf, length_an01_:%lf, length_an02_:%lf", base_anchor_, length_an01_, length_an02_);
        RCLCPP_INFO(this->get_logger(), "angle_an1_: %lf, angle_an2_:%lf", angle_an1_, angle_an2_);
    }
    double cosine_rule(double distance_a, double distance_b, double distance_c)
    {
        static double angle_alpha, fraction, last_angle_alpha;

        fraction = (pow(distance_b,2)+pow(distance_c,2)-pow(distance_a,2))/(2*distance_b*distance_c);
        angle_alpha = acos(fraction);

        //check NaN error
        if(__isnan(angle_alpha))
        {
            RCLCPP_INFO(this->get_logger(), "NaN value!! when %lf, %lf, %lf",distance_a, distance_b, distance_c);
            return last_angle_alpha;
        }
        else
        {
            last_angle_alpha = angle_alpha;
            return angle_alpha;
        }
    }
    void uwb_callback(evpi_interfaces::msg::MultiRange::SharedPtr msg) 
    {
        static bool is_first_time = true;
        static auto p = geometry_msgs::msg::Pose();
        static double r1,r2,r3 = 0;
        static double measured_angle_an1, measured_angle_an2;

        r1= msg->ranges[0];
        r2= msg->ranges[1];
        r3= msg->ranges[2];

        p.position.z = z1_; // default same height with anchor0
        p.position.y = y3_ + r3*cos(cosine_rule(r2, r3, base_anchor_));

        //Depends on calculated y to decide left or right 
        //Then choosing corresponding angle to check is_rear or not
        if(p.position.y <= 0)
        {
            measured_angle_an1 = cosine_rule(r1, r2, length_an01_);

            if(is_rear(measured_angle_an1, angle_an1_))
            {
                p.position.x = x3_ - r3*sin(cosine_rule(r2, r3, base_anchor_));
                RCLCPP_INFO(this->get_logger(), "Tag is now in rear");
            }
            else // front
            {
                p.position.x = x3_ + r3*sin(cosine_rule(r2, r3, base_anchor_));
            }
        }
        else // only when y>0
        {
            measured_angle_an2 = cosine_rule(r1, r3, length_an02_);

            if(is_rear(measured_angle_an2, angle_an2_))
            {
                p.position.x = x3_ - r3*sin(cosine_rule(r2, r3, base_anchor_));
                RCLCPP_INFO(this->get_logger(), "Tag is now in rear");
            }
            else // front
            {
                p.position.x = x3_ + r3*sin(cosine_rule(r2, r3, base_anchor_));
            }
        }

        if(is_first_time)
        {
            init_p.pose.pose = p;
            init_p.header.frame_id = "map";

            // publish the /initialpose directly
            // initpose_publisher_->publish(init_p);

            // or set the init_pose parameters instead
            this->set_parameter(rclcpp::Parameter("/t2/initialpose/x", init_p.pose.pose.position.x));
            this->set_parameter(rclcpp::Parameter("/t2/initialpose/y", init_p.pose.pose.position.y));
            this->set_parameter(rclcpp::Parameter("/t2/initialpose/z", init_p.pose.pose.position.z));
            this->set_parameter(rclcpp::Parameter("/t2/initialpose/theta", init_p.pose.pose.orientation.w));

            RCLCPP_INFO(this->get_logger(), "Tag Init Position: %lf, %lf, %lf", init_p.pose.pose.position.x, init_p.pose.pose.position.y, init_p.pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Tag Init Ovirentation: %lf, %lf, %lf, %lf", init_p.pose.pose.orientation.x, init_p.pose.pose.orientation.y, init_p.pose.pose.orientation.z, init_p.pose.pose.orientation.w);

            is_first_time = false;
        }
        else
        {
            tag_publisher_->publish(p);
        }

    }
    bool is_rear(double measure_val,double fix_angle) 
    {
        if(measure_val >= fix_angle)
        {
            return false;
        }
        else // only when measure_val < fix_angle
        {
            return true;
        }
    }
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tag_publisher_;
    rclcpp::Subscription<evpi_interfaces::msg::MultiRange>::SharedPtr uwb_subscriber_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridCosine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}