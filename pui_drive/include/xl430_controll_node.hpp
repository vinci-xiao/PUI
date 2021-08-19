#ifndef XL430_CONTROLL_NODE_HPP_
#define XL430_CONTROLL_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "pui_msgs/msg/set_velocity.hpp"
#include "pui_msgs/srv/get_velocity.hpp"

#include "geometry_msgs/msg/twist.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetVelocity = pui_msgs::msg::SetVelocity;
  using GetVelocity = pui_msgs::srv::GetVelocity;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetVelocity>::SharedPtr set_velocity_subscriber_;
  rclcpp::Service<GetVelocity>::SharedPtr get_velocity_server_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;


  int present_velocity;
};

#endif  // READ_WRITE_NODE_HPP_