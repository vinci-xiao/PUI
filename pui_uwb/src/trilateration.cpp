#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pui_msgs/MultiRange.h"
#include "geometry_msgs/Pose.h"


#include <sstream>

ros::Publisher tag_pub_;
ros::Subscriber anchor_sub_;

double d_, i_, j_;

//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<double>("anchor2_d", d_, 0.6);
  n_private.param<double>("anchor3_i", i_, 0.5196);
  n_private.param<double>("anchor3_j", j_, 0.30);
}

//-----------------------------------------------------------------------------------------------
void trilaterate_tag(double r1, double r2, double r3)
{
  static double tag_x, tag_y;
  tag_y = (pow(d_,2) +pow(r1,2) -pow(r2,2))/(2*d_);
  tag_x = (pow(i_,2) +pow(j_,2) +pow(r1,2) -pow(r3,2)-2*tag_y*j_)/(2*i_);

  geometry_msgs::Pose pose;
  pose.position.x= tag_x;
  pose.position.y= tag_y;

  tag_pub_.publish(pose);

}

//-----------------------------------------------------------------------------------------------
void Anchor_Callback(const pui_msgs::MultiRange& anchor_range)
{
  static double r1,r2,r3 = 0;
  r1= anchor_range.ranges[0];
  r2= anchor_range.ranges[1];
  r3= anchor_range.ranges[2];

  trilaterate_tag(r1, r2, r3);
}



//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{

  ros::init(argc, argv, "trilateration");
  ros::NodeHandle n;
  get_parameters(n);

  tag_pub_ = n.advertise<geometry_msgs::Pose>("tag_position", 5);
  anchor_sub_ = n.subscribe("uwb_range", 5, Anchor_Callback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    // std_msgs::String msg;
    // pui_msgs::MultiRange range;

    // range.max_range=66.6;
    // range.ranges.push_back(1.1);
    // range.ranges.push_back(2.2);
    // range.ranges.clear();
    // range.ranges.push_back(3.1);
    // range.ranges.push_back(4.2);
    // anchor_pub.publish(range);

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // chatter_pub_.publish(msg);
    

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}