// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run pui_drive xl430_controll_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_velocity dynamixel_sdk_custom_interfaces/SetVelocity "{id: 1, velocity: 1000}"
// $ ros2 service call /get_velocity dynamixel_sdk_custom_interfaces/srv/GetVelocity "id: 1"

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "pui_msgs/msg/set_velocity.hpp"
#include "pui_msgs/srv/get_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "xl430_controll_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/pui_driver"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity = 0;
int dxl_comm_result = COMM_TX_FAIL;

ReadWriteNode::ReadWriteNode()
: Node("xl430_controll_node")
{
  RCLCPP_INFO(this->get_logger(), "Run XL430 Controll Node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_velocity_subscriber_ =
    this->create_subscription<SetVelocity>(
    "set_velocity",
    QOS_RKL10V,
    [this](const SetVelocity::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Velocity Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Velocity Value.
      uint32_t goal_velocity = (unsigned int)msg->velocity;  // Convert int32 -> uint32

      // Write Goal Velocity (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_VELOCITY,
        goal_velocity,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", msg->id, msg->velocity);
      }
    }
    );

  auto get_present_velocity =
    [this](
    const std::shared_ptr<GetVelocity::Request> request,
    std::shared_ptr<GetVelocity::Response> response) -> void
    {
      // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t *>(&present_velocity),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Velocity: %d]",
        request->id,
        present_velocity
      );

      response->velocity = present_velocity;
    };

  cmd_vel_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // uint32_t goal_velocity = (unsigned int)msg->velocity;  

      double wheel_separation_ = 0.160;
      double wheel_radius_ = 0.033;

      double wheel_velocity[2];
      int32_t dynamixel_velocity[2];

      const uint8_t LEFT = 0;
      const uint8_t RIGHT = 1;

      double robot_lin_vel = msg->linear.x;
      double robot_ang_vel = msg->angular.z;
      float rpm = 0.229;
      //  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
      //       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

      double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

      wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
      wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

      // if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      // else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
      // else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

      // if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      // else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
      // else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);

      if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      else dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

      if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      else dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        1,
        ADDR_GOAL_VELOCITY,
        dynamixel_velocity[LEFT],
        &dxl_error
      );

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        2,
        ADDR_GOAL_VELOCITY,
        dynamixel_velocity[RIGHT],
        &dxl_error
      );

    }

    );


  get_velocity_server_ = create_service<GetVelocity>("get_velocity", get_present_velocity);
}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Velocity Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("xl430_controll_node"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("xl430_controll_node"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("xl430_controll_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("xl430_controll_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("xl430_controll_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("xl430_controll_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("xl430_controll_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("xl430_controll_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}