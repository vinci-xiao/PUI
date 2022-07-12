#include <math.h>
#include <iostream>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "evpi_interfaces/msg/multi_range.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std;

int tag_id_;
double min_range_,max_range_;

// Anchor1 and anchor2 need to be on the same horizontal level (z)
// Anchor3 and anchor4 need to be on the same vertiacal axis (y)
double x1_,y1_,z1_;
double x2_,y2_,z2_;
double x3_,y3_,z3_;
double x4_,y4_,z4_;

double base_xy_,base_yz_;

void base_calculation()
{
  base_xy_ = sqrtl(pow((x1_-x2_),2)+pow((y1_-y2_),2)+pow((z1_-z2_),2));
  base_yz_ = sqrtl(pow((x3_-x4_),2)+pow((y3_-y4_),2)+pow((z3_-z4_),2));
}

double cosine_rule(double distance_a, double distance_b, double distance_c)
{
  static double angle_alpha, fraction;

  fraction = (pow(distance_b,2)+pow(distance_c,2)-pow(distance_a,2))/(2*distance_b*distance_c);

  //limit check
  if (fraction >= 1.0){ fraction = 1.0; }
  else if(fraction <= -1.0){ fraction = -1.0; }

  angle_alpha = acos(fraction);
  return angle_alpha;
}

geometry_msgs::msg::Pose pose_est(double r1, double r2, double r3, double r4)
{
    static double xy_alpha, yz_alpha;
    static double tag_x, tag_y, tag_z;
    static double height, R1, R2, gamma;

    //xy
    xy_alpha = cosine_rule(r1, r2, base_xy_);
    tag_y = y2_ + r2*cos(xy_alpha);

    //yz
    yz_alpha = cosine_rule(r3, r4, base_yz_);
    tag_z = z4_ + r4*cos(yz_alpha);

    //project tag to xy-plane
    height = abs(tag_z-z1_);  // Anchor1 and anchor2 need to be on the same horizontal level (z)
    R2 = r2*cos(asin(height/r2));
    R1 = r1*cos(asin(height/r1));

    gamma = cosine_rule(R1, R2, base_xy_);
    tag_x = x2_ + R2*sin(gamma);

    geometry_msgs::msg::Pose pose;
    pose.position.x= tag_x;
    pose.position.y= tag_y;
    pose.position.z= tag_z;

    return pose;
}  

/////////////////////////////////////////////////////////////////////////////////////
class FollowingNode : public rclcpp::Node
{
public:
    FollowingNode()
    : Node("following_node")
    {
        tag_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("tag_position", 5);
        anchor_sub_ = this->create_subscription<evpi_interfaces::msg::MultiRange>(
        "/uwb_range", 5, std::bind(&FollowingNode::topic_callback, this, std::placeholders::_1));
        
        get_parameters();
    }
    void get_parameters(void)
    {
      this->declare_parameter<std::int16_t>("tag_id", 10);
      this->declare_parameter<std::double_t>("min_raw_range", 0.0);
      this->declare_parameter<std::double_t>("max_raw_range", 0.0);
      this->declare_parameter<std::double_t>("anchor_1/x", 0.0);
      this->declare_parameter<std::double_t>("anchor_1/y", 0.0);
      this->declare_parameter<std::double_t>("anchor_1/z", 0.0);
      this->declare_parameter<std::double_t>("anchor_2/x", 0.0);
      this->declare_parameter<std::double_t>("anchor_2/y", 0.0);
      this->declare_parameter<std::double_t>("anchor_2/z", 0.0);
      this->declare_parameter<std::double_t>("anchor_3/x", 0.0);
      this->declare_parameter<std::double_t>("anchor_3/y", 0.0);
      this->declare_parameter<std::double_t>("anchor_3/z", 0.0);
      this->declare_parameter<std::double_t>("anchor_4/x", 0.0);
      this->declare_parameter<std::double_t>("anchor_4/y", 0.0);
      this->declare_parameter<std::double_t>("anchor_4/z", 0.0);
      
      this->get_parameter("tag_id", tag_id_);
      this->get_parameter("min_raw_range", min_range_);
      this->get_parameter("max_raw_range", max_range_);
      this->get_parameter("anchor_1/x", x1_);
      this->get_parameter("anchor_1/y", y1_);
      this->get_parameter("anchor_1/z", z1_);
      this->get_parameter("anchor_2/x", x2_);
      this->get_parameter("anchor_2/y", y2_);
      this->get_parameter("anchor_2/z", z2_);
      this->get_parameter("anchor_3/x", x3_);
      this->get_parameter("anchor_3/y", y3_);
      this->get_parameter("anchor_3/z", z3_);
      this->get_parameter("anchor_4/x", x4_);
      this->get_parameter("anchor_4/y", y4_);
      this->get_parameter("anchor_4/z", z4_);
      RCLCPP_INFO(this->get_logger(), "ROS_PARAMS: tag_id=%d",tag_id_);
    }

private:
    void topic_callback(evpi_interfaces::msg::MultiRange::SharedPtr msg) 
    {
        auto p = geometry_msgs::msg::Pose();
        static double r1,r2,r3, r4 = 0;

        RCLCPP_INFO(this->get_logger(), "tagID= %d, realID=%d",tag_id_, msg->id);

        if (msg->id == tag_id_)
        {
            tag_id_= msg->id;
            r1= msg->ranges[0];
            r2= msg->ranges[1];
            r3= msg->ranges[2];
            r4= msg->ranges[3];

            // Filter out value that could result in +-infi value(Nan error)
            r1 = min(max(r1, min_range_),max_range_);
            r2 = min(max(r2, min_range_),max_range_);
            r3 = min(max(r3, min_range_),max_range_);
            r4 = min(max(r4, min_range_),max_range_);

            p = pose_est(r1, r2, r3, r4);    
            tag_pub_->publish(p);
        }
    }
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tag_pub_;
    rclcpp::Subscription<evpi_interfaces::msg::MultiRange>::SharedPtr anchor_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowingNode>();

    base_calculation();

    RCLCPP_INFO(node->get_logger(), "ROS node initialed: following_node\n");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
