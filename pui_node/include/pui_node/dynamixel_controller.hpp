#ifndef DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Control table address for XL-430
// https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
// 0.0.229[rev/min] * (2 * 3.14159265359 * wheel_radius) / 60 = 0.00345575191
constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// ref) https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#present-position
// 0.087890625[deg/pulse] * 3.14159265359 / 180 = 0.001533981f
constexpr double DEG_PULSE_TO_RAD = 0.001533981;

class DynamixelController : public rclcpp::Node
{
public:
  DynamixelController();
  virtual ~DynamixelController();

private:
  sensor_msgs::msg::JointState joint_state_msg_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  int present_velocity;

  uint32_t position[2];
  uint32_t velocity[2];
  uint32_t effort[2];

  void run();
  void publish_timer(const std::chrono::milliseconds timeout);

  rclcpp::TimerBase::SharedPtr publish_timer_;

};

#endif  // DYNAMIXEL_CONTROLLER_HPP_
