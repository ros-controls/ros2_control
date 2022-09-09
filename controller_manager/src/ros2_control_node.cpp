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

#include "controller_manager/controller_manager.hpp"
#include "controller_manager/realtime.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  std::thread cm_thread([cm]() {
    if (controller_manager::has_realtime_kernel())
    {
      if (!controller_manager::configure_sched_fifo(kSchedPriority))
      {
        RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy");
      }
    }
    else
    {
      RCLCPP_INFO(cm->get_logger(), "RT kernel is recommended for better performance");
    }

    // for calculating sleep time
    auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
    std::chrono::system_clock::time_point next_iteration_time =
      std::chrono::system_clock::time_point(std::chrono::nanoseconds(cm->now().nanoseconds()));

    // for calculating the measured period of the loop
    rclcpp::Time previous_time = cm->now();

    while (rclcpp::ok())
    {
      // calculate measured period
      auto const current_time = cm->now();
      auto const measured_period = current_time - previous_time;
      previous_time = current_time;

      // execute update loop
      cm->read();
      cm->update(cm->now(), measured_period);
      cm->write();

      // wait until we hit the end of the period
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }
  });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
