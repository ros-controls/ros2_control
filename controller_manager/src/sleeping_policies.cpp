// Copyright 2026 ROS2-Control Development Team
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

#include <stdexcept>

#include "controller_manager/sleeping_policies.hpp"

namespace controller_manager
{
SimTimeJumpHandler::SimTimeJumpHandler(const rclcpp::Clock::SharedPtr & clock) : clock_(clock)
{
  if (!clock_)
  {
    throw std::invalid_argument("Cannot observe time jumps of a null clock.");
  }

  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = false;
  threshold.min_forward.nanoseconds = 0;    // 0 disables the threshold
  threshold.min_backward.nanoseconds = -1;  // any backwards movement
  jump_handler_ = clock_->create_jump_callback(
    nullptr,
    [this](const rcl_time_jump_t & jump)
    {
      if (jump.delta.nanoseconds >= 0)
      {
        return;
      }
      backward_jump_.store(true, std::memory_order_relaxed);
      // The control loop waits for a deadline of the timeline that just ended, so wake it up.
      clock_->cancel_sleep_or_wait();
    },
    threshold);
}

bool SimTimeJumpHandler::consume_backward_jump()
{
  return backward_jump_.exchange(false, std::memory_order_relaxed);
}
}  // namespace controller_manager

rclcpp::Duration update_time_base_and_get_period(
  controller_manager::ControlLoopState & state, const rclcpp::Time & current_time,
  bool time_base_reset)
{
  if (time_base_reset)
  {
    // The time base belongs to the timeline that the simulator just left, so it cannot be used to
    // derive a period for the current cycle.
    state.previous_time = current_time;
    return rclcpp::Duration(state.period);
  }

  const auto measured_period = current_time - state.previous_time;
  state.previous_time = current_time;
  return measured_period;
}

bool sleep_for_sim_time(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  controller_manager::ControlLoopState & state)
{
  // Implement sleep logic for simulated time
  try
  {
    // A cancelled sleep, e.g. because simulation time jumped backwards, returns false here. That
    // is expected: the loop must continue in the new timeline instead of treating it as an error.
    cm->get_clock()->sleep_until(state.previous_time + state.period);
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(
      cm->get_logger(), "sleep_until failed with error: %s. Exiting control loop and aborting....",
      e.what());
    return false;
  }
  return true;
}

void sleep_for_blocking_read_write(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  const controller_manager::ControlLoopTimingConfig & config,
  controller_manager::ControlLoopState & state)
{
  // Implement sleep logic for blocking read/write
  if (
    (state.cycle_end_time - state.previous_time).nanoseconds() <
    static_cast<rcl_duration_value_t>(config.minimum_cycle_time * 1e9))
  {
    RCLCPP_WARN_THROTTLE(
      cm->get_logger(), *cm->get_clock(), 1000,
      "Last control cycle was shorter than the minimum cycle time while blocking read or "
      "write is configured. Is the hardware interface not blocking correctly? Note: This "
      "might happen when the hardware component that should block is not active.");
    std::this_thread::sleep_for(
      std::chrono::microseconds(static_cast<int>(config.minimum_cycle_time * 1e6)));
  }
}

void sleep_for_periodic_cycle(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  const controller_manager::ControlLoopTimingConfig & config,
  controller_manager::ControlLoopState & state)
{
  state.next_iteration_time += state.period;
  const auto time_now = std::chrono::steady_clock::now();
  if (config.manage_overruns && state.next_iteration_time < time_now)
  {
    const double time_diff =
      static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - state.next_iteration_time)
          .count()) /
      1.e6;
    const double cm_period = 1.e3 / static_cast<double>(cm->get_update_rate());
    const int overrun_count = static_cast<int>(std::ceil(time_diff / cm_period));
    RCLCPP_WARN_THROTTLE(
      cm->get_logger(), *cm->get_clock(), 1000,
      "Overrun detected! The controller manager missed its desired rate of %d Hz. The loop "
      "took %f ms (missed cycles : %d).",
      cm->get_update_rate(), time_diff + cm_period, overrun_count + 1);
    state.next_iteration_time += (overrun_count * state.period);
  }
  std::this_thread::sleep_until(state.next_iteration_time);
}
