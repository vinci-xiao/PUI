#ifndef PUI_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define PUI_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "pui_node/odometry.hpp"

namespace city_science
{
namespace pui
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace pui
}  // namespace city_science
#endif  // PUI_NODE__DIFF_DRIVE_CONTROLLER_HPP_
