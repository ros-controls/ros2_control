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

#include <errno.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors.hpp"
#include "realtime_tools/realtime_helpers.hpp"

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

  rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();
  std::vector<std::string> node_arguments = cm_node_options.arguments();
  for (int i = 1; i < argc; ++i)
  {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      // A simple way to reject non ros args
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  cm_node_options.arguments(node_arguments);

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, manager_node_name, "", cm_node_options);

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);

  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory)
  {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first)
    {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str());
    }
  }

  rclcpp::Parameter cpu_affinity_param;
  if (cm->get_parameter("cpu_affinity", cpu_affinity_param))
  {
    std::vector<int> cpus = {};
    if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      cpus = {static_cast<int>(cpu_affinity_param.as_int())};
    }
    else if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
    {
      const auto cpu_affinity_param_array = cpu_affinity_param.as_integer_array();
      std::for_each(
        cpu_affinity_param_array.begin(), cpu_affinity_param_array.end(),
        [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
    }
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
    if (!affinity_result.first)
    {
      RCLCPP_WARN(
        cm->get_logger(), "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str());
    }
  }

  // wait for the clock to be available
  cm->get_clock()->wait_until_started();
  cm->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0 / cm->get_update_rate()));

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  std::thread cm_thread(
    [cm, thread_priority, use_sim_time]()
    {
      if (!realtime_tools::configure_sched_fifo(thread_priority))
      {
        RCLCPP_WARN(
          cm->get_logger(),
          "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.",
          errno, strerror(errno));
      }
      else
      {
        RCLCPP_INFO(
          cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
          thread_priority);
      }

      // for calculating sleep time
      auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());

      // for calculating the measured period of the loop
      rclcpp::Time previous_time = cm->get_trigger_clock()->now();
      std::this_thread::sleep_for(period);

      std::chrono::steady_clock::time_point next_iteration_time{std::chrono::steady_clock::now()};

      while (rclcpp::ok())
      {
        // calculate measured period
        auto const current_time = cm->get_trigger_clock()->now();
        auto const measured_period = current_time - previous_time;
        previous_time = current_time;

        // execute update loop
        cm->read(cm->get_trigger_clock()->now(), measured_period);
        cm->update(cm->get_trigger_clock()->now(), measured_period);
        cm->write(cm->get_trigger_clock()->now(), measured_period);

        // wait until we hit the end of the period
        if (use_sim_time)
        {
          cm->get_clock()->sleep_until(current_time + period);
        }
        else
        {
          next_iteration_time += period;
          const auto time_now = std::chrono::steady_clock::now();
          if (next_iteration_time < time_now)
          {
            const double time_diff =
              static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - next_iteration_time)
                  .count()) /
              1.e6;
            const double cm_period = 1.e3 / static_cast<double>(cm->get_update_rate());
            const int overrun_count = static_cast<int>(std::ceil(time_diff / cm_period));
            RCLCPP_WARN(
              cm->get_logger(),
              "Overrun detected! The controller manager missed its desired rate of %d Hz. The loop "
              "took %f ms (missed cycles : %d).",
              cm->get_update_rate(), time_diff + cm_period, overrun_count + 1);
            next_iteration_time += (overrun_count * period);
          }
          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
