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

#include "controller_manager/sleeping_policies.hpp"

#include <stdexcept>

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
  threshold.min_forward.nanoseconds = 0;
  threshold.min_backward.nanoseconds = -1;
  jump_handler_ = clock_->create_jump_callback(
    nullptr,
    [this](const rcl_time_jump_t & jump)
    {
      if (jump.delta.nanoseconds >= 0)
      {
        return;
      }
      backward_jump_.store(true, std::memory_order_relaxed);
      clock_->cancel_sleep_or_wait();
    },
    threshold);
}

bool SimTimeJumpHandler::consume_backward_jump()
{
  return backward_jump_.exchange(false, std::memory_order_relaxed);
}

rclcpp::Time SimTimeJumpHandler::sample_time(bool & backward_jump)
{
  backward_jump = consume_backward_jump();
  return clock_->now();
}
}  // namespace controller_manager

rclcpp::Duration update_time_base_and_get_period(
  controller_manager::ControlLoopState & state, const rclcpp::Time & current_time,
  bool time_base_reset)
{
  if (time_base_reset)
  {
    state.previous_time = current_time;
    return rclcpp::Duration(state.period);
  }

  const auto measured_period = current_time - state.previous_time;
  state.previous_time = current_time;
  return measured_period;
}

rclcpp::Duration sample_control_loop_period(
  controller_manager::ControlLoopState & state,
  const rclcpp::Clock::SharedPtr & clock,
  controller_manager::SimTimeJumpHandler * jump_handler)
{
  bool time_base_reset = false;
  const auto current_time =
    jump_handler != nullptr ? jump_handler->sample_time(time_base_reset) : clock->now();
  return update_time_base_and_get_period(
    state, current_time, time_base_reset || current_time < state.previous_time);
}
