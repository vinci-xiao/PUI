#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <iostream>  

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pui_msgs/MultiRange.h"
#include "geometry_msgs/Pose.h"


using namespace Eigen;  
using namespace std;  

ros::Publisher tag_pub_;
ros::Subscriber anchor_sub_;

double x1_, x2_, x3_;
double y1_, y2_, y3_;

// AX=b
Matrix<double, 2, 2> A;               // Fixed rows and cols. 
Matrix<double, 2, 1> X;               // Fixed rows and cols. 
Matrix<double, 2, 1> b;               // Fixed rows and cols. 

//-----------------------------------------------------------------------------------------------
void get_parameters(ros::NodeHandle n_private)
{
  n_private.param<double>("anchor1_x", x1_, 0.0);
  n_private.param<double>("anchor1_y", y1_, 0.0);
  n_private.param<double>("anchor2_x", x2_, 0.0);
  n_private.param<double>("anchor2_y", y2_, 3.406);
  n_private.param<double>("anchor3_x", x3_, 2.95);
  n_private.param<double>("anchor3_y", y3_, 1.703);
}

//-----------------------------------------------------------------------------------------------
void matrix_A_setting(void)
{
  A.row(0)<< -2*(x1_-x3_), -2*(y1_-y3_);  // populate the first row--shorthand method
  A.row(1)<< -2*(x2_-x3_), -2*(y2_-y3_);  //second row
  cout<<"\n matrix_A:\n"<< A<<endl; 

}

//-----------------------------------------------------------------------------------------------
void matrix_b_setting(double r1, double r2, double r3)
{
  b.row(0)<< pow(x1_,2) -pow(x3_,2) +pow(y1_,2)-pow(y3_,2) +pow(r1,2)-pow(r3,2);  
  b.row(1)<< pow(x2_,2) -pow(x3_,2) +pow(y2_,2)-pow(y3_,2) +pow(r2,2)-pow(r3,2); 
  cout<<"\n r1:"<<r1<<" ,r2:"<<r2<<"  ,r3:"<<r3<<endl; 
  cout<<"\n matrix_b:\n"<< b<<endl; 
}

//-----------------------------------------------------------------------------------------------
void maximum_likelihood_est(void)
{
  static double tag_x, tag_y;
  Matrix<double, 2, 2> temp; 

  temp =A.transpose()*A;
  cout << "temp:\n" << temp << endl;
  cout << "Mat_temp逆矩阵：\n" << temp.inverse() << endl;

  X =temp.inverse()*A.transpose()*b;

  cout << "======> X:\n" << X << endl;

  tag_x =X(0,0);  // row0
  tag_y =X(1,0);  // row1

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
  
  matrix_b_setting(r1, r2, r3);
  maximum_likelihood_est();
}



//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{

  ros::init(argc, argv, "try_eigen");
  ros::NodeHandle n;
  get_parameters(n);

  matrix_A_setting();

  tag_pub_ = n.advertise<geometry_msgs::Pose>("tag_position", 5);
  anchor_sub_ = n.subscribe("uwb_range", 5, Anchor_Callback);

  ros::Rate loop_rate(10);

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
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // chatter_pub_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}