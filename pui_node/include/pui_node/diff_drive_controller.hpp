// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

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
