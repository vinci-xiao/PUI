#include <cstdio>
#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "pui_node/dynamixel_controller.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity = 0;
int dxl_comm_result = COMM_TX_FAIL;

DynamixelController::DynamixelController()
: Node("dynamixel_controller")
{
  RCLCPP_INFO(this->get_logger(), "Run Dynamixel Controller!");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  cmd_vel_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    QOS_RKL10V,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      double wheel_separation_ = 0.160;
      double wheel_radius_ = 0.033;

      double wheel_velocity[2];
      uint32_t dynamixel_velocity[2];

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

      if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      else dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value/5); // TODO: ratio of cmd_vel and dynamixel_velocity

      if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      else dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value/5);

      // set velocity for left_wheel
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        1,
        ADDR_GOAL_VELOCITY,
        dynamixel_velocity[LEFT],
        &dxl_error
      );
      // show cmd_result of left_wheel
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: 1] [Goal Velocity: %d]", dynamixel_velocity[LEFT]);
      }

      // set velocity for right_wheel
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        2,
        ADDR_GOAL_VELOCITY,
        -dynamixel_velocity[RIGHT],  // cuz left and right motor are mirrored.
        &dxl_error
      );
      // show cmd_result of right_wheel
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: 2] [Goal Velocity: %d]", dynamixel_velocity[RIGHT]);
      }

    }
    );
  
  joint_state_publisher_= this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 5);

  run();
}

void DynamixelController::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));

}


void DynamixelController::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      //Source: https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/50138263ccccba144c84696e086b2fe0de224a71/dynamixel_workbench_controllers/src/dynamixel_workbench_controllers.cpp
      joint_state_msg_.name.clear();
      joint_state_msg_.position.clear();
      joint_state_msg_.velocity.clear();
      joint_state_msg_.effort.clear();

      // ID=1, position
      dxl_comm_result =
      packetHandler->read4ByteTxRx(
        portHandler,
        1,
        ADDR_PRESENT_POSITION,
        &position[0],
        &dxl_error
      );

      // ID=2, position
      dxl_comm_result =
      packetHandler->read4ByteTxRx(
        portHandler,
        2,
        ADDR_PRESENT_POSITION,
        &position[1],
        &dxl_error
      );

      // ID=1, velocity
      dxl_comm_result =
      packetHandler->read4ByteTxRx(
        portHandler,
        1,
        ADDR_PRESENT_VELOCITY,
        &velocity[0],
        &dxl_error
      );

      // ID=2, velocity
      dxl_comm_result =
      packetHandler->read4ByteTxRx(
        portHandler,
        2,
        ADDR_PRESENT_VELOCITY,
        &velocity[1],
        &dxl_error
      );

      joint_state_msg_.header.frame_id = "base_link";
      joint_state_msg_.header.stamp = now;

      joint_state_msg_.name.push_back("wheel_left_joint");
      joint_state_msg_.name.push_back("wheel_right_joint");

      joint_state_msg_.position.push_back(DEG_PULSE_TO_RAD * position[0]);
      joint_state_msg_.position.push_back(DEG_PULSE_TO_RAD * position[1]);

      joint_state_msg_.velocity.push_back(RPM_TO_MS * (int32_t)velocity[0]);
      joint_state_msg_.velocity.push_back(RPM_TO_MS * (int32_t)velocity[1]);

      // joint_state_msg_.effort.push_back(effort[0]);
      // joint_state_msg_.effort.push_back(effort[1]);

      joint_state_publisher_->publish(joint_state_msg_);

    }
  );
}

DynamixelController::~DynamixelController()
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
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_controller"), "ID:%d, Failed to set Velocity Control Mode.",dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_controller"), "ID:%d, Succeeded to set  in Velocity Control Mode.",dxl_id);
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
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_controller"), "ID:%d, Failed to set Velocity Control Mode.",dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_controller"), "ID:%d, Succeeded to enable torque.",dxl_id);
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_controller"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_controller"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_controller"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_controller"), "Succeeded to set the baudrate.");
  }

  // setupDynamixel(BROADCAST_ID);
  setupDynamixel(1);
  setupDynamixel(2);

  rclcpp::init(argc, argv);

  auto dynamixelcontroller = std::make_shared<DynamixelController>();
  rclcpp::spin(dynamixelcontroller);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    2,
    &dxl_error
  );

  return 0;
}
