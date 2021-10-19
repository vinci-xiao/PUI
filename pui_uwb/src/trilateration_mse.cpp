#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_msgs/msg/tf_message.hpp"

#include "pui_msgs/msg/multi_range.hpp"

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;  

// double x1_= -0.66;
// double y1_= -0.10;
// double x2_= 0.80;
// double y2_= 0.28;
// double x3_= 0.80;
// double y3_= -0.28;

double x1_, y1_;
double x2_, y2_;
double x3_, y3_;

// AX=b
Matrix<double, 2, 2> A;               // Fixed rows and cols. 
Matrix<double, 2, 1> X;               // Fixed rows and cols. 
Matrix<double, 2, 1> b;               // Fixed rows and cols. 

/////////////////////////////////////////////////////////////////////////////////////

class TrilaterationMSE : public rclcpp::Node
{
public:
    TrilaterationMSE()
    : Node("trilateration_mse")
    {
        tag_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("tag_position", 10);
        uwb_subscriber_ = this->create_subscription<pui_msgs::msg::MultiRange>(
            "uwb_range", 10, std::bind(&TrilaterationMSE::uwb_callback, this, std::placeholders::_1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        get_static_tf();
        matrix_A_setting();
    }
private:
    void get_static_tf(void)
    {
        geometry_msgs::msg::TransformStamped anchor_0_tf;
        geometry_msgs::msg::TransformStamped anchor_1_tf;
        geometry_msgs::msg::TransformStamped anchor_2_tf;

        rclcpp::Time now = this->get_clock()->now();

        anchor_0_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_0_link", now, 1s);

        anchor_1_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_1_link", now, 1s);
    
        anchor_2_tf = tf_buffer_->lookupTransform(
        "base_link","anchor_2_link", now, 1s);

        x1_ = anchor_0_tf.transform.translation.x;
        y1_ = anchor_0_tf.transform.translation.y;
        x2_ = anchor_1_tf.transform.translation.x;
        y2_ = anchor_1_tf.transform.translation.y;
        x3_ = anchor_2_tf.transform.translation.x;
        y3_ = anchor_2_tf.transform.translation.y;
        RCLCPP_INFO(this->get_logger(), "Done process tf");
    }
    void matrix_A_setting(void)
    {
        A.row(0)<< -2*(x1_-x3_), -2*(y1_-y3_);  // populate the first row--shorthand method
        A.row(1)<< -2*(x2_-x3_), -2*(y2_-y3_);  //second row
        // cout<<"\n matrix_A:\n"<< A<<endl; 
    }
    void matrix_b_setting(double r1, double r2, double r3)
    {
        b.row(0)<< -pow(x1_,2) +pow(x3_,2) -pow(y1_,2) +pow(y3_,2) +pow(r1,2)-pow(r3,2);  
        b.row(1)<< -pow(x2_,2) +pow(x3_,2) -pow(y2_,2) +pow(y3_,2) +pow(r2,2)-pow(r3,2); 
        // cout<<"\n r1:"<<r1<<" ,r2:"<<r2<<"  ,r3:"<<r3<<endl; 
        // cout<<"\n matrix_b:\n"<< b<<endl; 
    } 
    geometry_msgs::msg::Pose maximum_likelihood_est(void)
    {
        static double tag_x, tag_y;
        Matrix<double, 2, 2> temp; 

        temp =A.transpose()*A;
        // cout << "temp:\n" << temp << endl;
        // cout << "Mat_temp逆矩阵：\n" << temp.inverse() << endl;

        X =temp.inverse()*A.transpose()*b;

        // cout << "======> X:\n" << X << endl;

        tag_x =X(0,0);  // row0
        tag_y =X(1,0);  // row1

        geometry_msgs::msg::Pose pose;
        pose.position.x= tag_x;
        pose.position.y= tag_y;

        return pose;
    }  
    void uwb_callback(pui_msgs::msg::MultiRange::SharedPtr msg) 
    {
        auto p = geometry_msgs::msg::Pose();
        static double r1,r2,r3 = 0;

        r1= msg->ranges[0];
        r2= msg->ranges[1];
        r3= msg->ranges[2];

        matrix_b_setting(r1, r2, r3);     
        p = maximum_likelihood_est();    
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
    auto node = std::make_shared<TrilaterationMSE>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}