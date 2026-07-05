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
#include "controller_manager/sleeping_policies.hpp"
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

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  const bool manage_overruns = cm->get_parameter_or<bool>("overruns.manage", true);
  RCLCPP_INFO(
    cm->get_logger(), "Overruns handling is : %s", manage_overruns ? "enabled" : "disabled");
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  const controller_manager::ControlLoopTimingConfig timing_config{
    .use_sim_time = use_sim_time,
    .manage_overruns = manage_overruns,
    .expect_blocking_read_write =
      cm->get_parameter_or<bool>("hardware_synchronization.expect_blocking_read_write", false),
    .minimum_cycle_time =
      cm->get_parameter_or<double>("hardware_synchronization.minimum_cycle_time", 0.0001),
  };
  RCLCPP_INFO_EXPRESSION(
    cm->get_logger(), timing_config.expect_blocking_read_write,
    "Synchronizing control loop with hardware.");

  std::thread cm_thread(
    [cm, thread_priority, timing_config]()
    {
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
          cpus.assign(cpu_affinity_param_array.begin(), cpu_affinity_param_array.end());
        }
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
        if (!affinity_result.first)
        {
          RCLCPP_WARN(
            cm->get_logger(), "Unable to set the CPU affinity : '%s'",
            affinity_result.second.c_str());
        }
      }

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

      // wait for the clock to be available
      cm->get_clock()->wait_until_started();
      cm->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0 / cm->get_update_rate()));

      controller_manager::ControlLoopState state;
      state.period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      state.previous_time = cm->get_trigger_clock()->now();
      std::this_thread::sleep_for(state.period);
      state.next_iteration_time = std::chrono::steady_clock::now();
      while (rclcpp::ok())
      {
        // calculate measured period
        auto const current_time = cm->get_trigger_clock()->now();
        auto const measured_period = current_time - state.previous_time;
        state.previous_time = current_time;

        // execute update loop
        cm->read(cm->get_trigger_clock()->now(), measured_period);
        cm->update(cm->get_trigger_clock()->now(), measured_period);
        cm->write(cm->get_trigger_clock()->now(), measured_period);
        state.cycle_end_time = cm->get_trigger_clock()->now();

        // wait until we hit the end of the period
        if (timing_config.use_sim_time)
        {
          if (!sleep_for_sim_time(cm, state))
          {
            break;
          }
        }
        else if (timing_config.expect_blocking_read_write)
        {
          sleep_for_blocking_read_write(cm, timing_config, state);
        }
        else
        {
          sleep_for_periodic_cycle(cm, timing_config, state);
        }
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
