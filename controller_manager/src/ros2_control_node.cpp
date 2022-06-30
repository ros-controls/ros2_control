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
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

// Detect the presence of a RT kernel
bool has_realtime_kernel()
{
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open())
  {
    realtime_file >> has_realtime;
  }
  return has_realtime;
}

// Configure fifo sched for RT
bool configure_sched_fifo(int priority)
{
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &schedp))
  {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  std::thread cm_thread(
    [cm]()
    {
      if (has_realtime_kernel())
      {
        if (!configure_sched_fifo(kSchedPriority))
        {
          RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy");
        }
      }
      else
      {
        RCLCPP_WARN(cm->get_logger(), "RT kernel was not detected");
      }

      rclcpp::Time mut_previous_time = cm->now();
      rclcpp::Time mut_end_period = mut_previous_time;

      // Use nanoseconds to avoid chrono's rounding
      rclcpp::Duration period(std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate()));

      while (rclcpp::ok())
      {
        // wait until we hit the end of the period
        mut_end_period += period;
        std::this_thread::sleep_for(
          std::chrono::nanoseconds((mut_end_period - cm->now()).nanoseconds()));

        // execute update loop
        auto const current_time = cm->now();
        auto const measured_period = current_time - mut_previous_time;
        mut_previous_time = current_time;
        cm->read(current_time, measured_period);
        cm->update(current_time, measured_period);
        cm->write(current_time, measured_period);
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
