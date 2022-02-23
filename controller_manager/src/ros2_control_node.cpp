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
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const int DEFAULT_UPDATE_RATE = 100;

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
    // load controller_manager update time parameter
    int update_rate = DEFAULT_UPDATE_RATE;
    if (!cm->get_parameter("update_rate", update_rate))
    {
      RCLCPP_WARN(cm->get_logger(), "'update_rate' parameter not set, using default value.");
    }
    RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", update_rate);

    rclcpp::Time end_period = cm->now();

    // Use nanoseconds to avoid chrono's rounding
    rclcpp::Duration period(std::chrono::nanoseconds(1000000000 / update_rate));

    while (rclcpp::ok())
    {
      // wait until we hit the end of the period
      end_period += period;
      std::this_thread::sleep_for(std::chrono::nanoseconds((end_period - cm->now()).nanoseconds()));

      // execute "real-time" update loop
      cm->read();
      cm->update();
      cm->write();
    }
  });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
