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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/control_loop.hpp"
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

void sleep_until(rclcpp::Time time)
{
  std::this_thread::sleep_until(
    std::chrono::system_clock::time_point(std::chrono::nanoseconds(time.nanoseconds())));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // When the MutliThreadedExecutor issues are fixed (ros2/rclcpp#1168), this loop should be
  // converted back to a timer.
  std::thread cm_thread([cm]() {
    RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

    // Use nanoseconds to avoid chrono's rounding
    auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());

    // Functions for control loop
    auto const now = [&cm]() { return cm->now(); };
    auto const sleep_until = [](rclcpp::Time time) {
      std::this_thread::sleep_until(
        std::chrono::system_clock::time_point(std::chrono::nanoseconds(time.nanoseconds())));
    };
    auto const ok = []() { return rclcpp::ok(); };
    auto const do_work = [&cm](rclcpp::Time current_time, rclcpp::Duration period) {
      // Write is called first as the consistent rate of writing to hardware
      // should not be affected by the time it takes to call update and read
      cm->write(current_time, period);
      cm->update(current_time, period);
      cm->read(current_time, period);
    };

    ControlLoop(period, now, ok, do_work, sleep_until);
  });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
