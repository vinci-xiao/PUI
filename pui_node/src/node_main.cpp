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

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "pui_node/diff_drive_controller.hpp"
#include "pui_node/pui.hpp"
#include "pui_node/odometry.hpp"

void help_print()
{
  printf("For pui node : \n");
  printf("pui_node [-i usb_port] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-i usb_port: Connected USB port with OpenCR.");
}

int main(int argc, char * argv[])
{
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyACM0";
  // char * cli_options;
  // cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
  // if (nullptr != cli_options) {
  //   usb_port = std::string(cli_options);
  // }

  rclcpp::executors::SingleThreadedExecutor executor;

  auto pui = std::make_shared<city_science::pui::Pui>(usb_port);
  auto diff_drive_controller =
    std::make_shared<city_science::pui::DiffDriveController>(
    pui->get_wheels()->separation,
    pui->get_wheels()->radius);

  executor.add_node(pui);
  executor.add_node(diff_drive_controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
