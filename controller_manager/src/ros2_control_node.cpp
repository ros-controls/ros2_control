// Copyright 2020 ROS2-Control Development Team
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

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using namespace std::chrono_literals;

constexpr const auto kLoggerName = "ros2_control_node";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  rclcpp::TimerBase::SharedPtr timer;

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    // TODO(anyone): remove robot_hw when ResourceManager is added
    // since RobotHW is not a plugin we had to take some type of robot
    std::make_shared<test_robot_hardware::TestRobotHardware>(),
    executor,
    manager_node_name);

  // Declare default controller manager rate of 100Hz
  cm->declare_parameter("update_rate", 100);
  // load controller_manager update time parameter
  int update_rate;
  if (!cm->get_parameter("update_rate", update_rate)) {
    throw std::runtime_error("update_rate parameter not existing or empty");
  }
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "update rate is %d Hz", update_rate);

  timer = cm->create_wall_timer(
    std::chrono::milliseconds(1000/update_rate),
    std::bind(&controller_manager::ControllerManager::update, cm.get()),
    cm->deterministic_callback_group_);

  executor->add_node(cm);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
