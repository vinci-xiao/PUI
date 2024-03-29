#ifndef PUI_NODE__PUI_HPP_
#define PUI_NODE__PUI_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <turtlebot3_msgs/msg/sensor_state.hpp>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

// #include "pui_node/control_table.hpp"
// #include "pui_node/dynamixel_sdk_wrapper.hpp"
#include "pui_node/odometry.hpp"

// #include "turtlebot3_node/devices/devices.hpp"
// #include "turtlebot3_node/devices/motor_power.hpp"
// #include "turtlebot3_node/devices/reset.hpp"
// #include "turtlebot3_node/devices/sound.hpp"

// #include "turtlebot3_node/sensors/battery_state.hpp"
// #include "turtlebot3_node/sensors/imu.hpp"
// #include "turtlebot3_node/sensors/joint_state.hpp"
// #include "turtlebot3_node/sensors/sensor_state.hpp"
// #include "turtlebot3_node/sensors/sensors.hpp"

namespace city_science
{
namespace pui
{
// extern const ControlTable extern_control_table;
class Pui : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit Pui(const std::string & usb_port);
  virtual ~Pui() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  // void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  // void check_device_status();
  // void setup_dynamixel(uint8_t dxl_id);

  // void add_sensors();
  // void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  // std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  // std::list<sensors::Sensors *> sensors_;
  // std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace pui
}  // namespace city_science
#endif  // PUI_NODE__PUI_HPP_
