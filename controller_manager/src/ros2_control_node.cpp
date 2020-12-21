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
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor,
    manager_node_name);

  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // When the MutliThreadedExecutor issues are fixed (ros2/rclcpp#1168), this loop should be
  // converted back to a timer.
  std::thread cm_thread([cm]() {
      // load controller_manager update time parameter
      int update_rate = 100;
      if (!cm->get_parameter("update_rate", update_rate)) {
        throw std::runtime_error("update_rate parameter not existing or empty");
      }
      RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", update_rate);

      while (rclcpp::ok()) {
        cm->read();
        cm->update();
        cm->write();
        struct timespec duration{};
        duration.tv_nsec = 1000000000 / update_rate;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &duration, NULL);
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
