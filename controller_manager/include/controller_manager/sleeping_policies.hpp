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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

namespace controller_manager
{
class ControllerManager;

struct ControlLoopTimingConfig
{
  bool use_sim_time{false};
  bool manage_overruns{true};
  bool expect_blocking_read_write{false};
  double minimum_cycle_time{0.0001};
};

struct ControlLoopState
{
  rclcpp::Time previous_time;
  std::chrono::steady_clock::time_point next_iteration_time;
  std::chrono::nanoseconds period{0};
  rclcpp::Time cycle_end_time;  //< The time when work was done in the current cycle.
};

/// Detects backwards jumps of simulation time and interrupts pending sleeps on them.
///
/// Simulation time jumps backwards when a simulator resets or reloads its world while the
/// controller manager keeps running, e.g. Gazebo's ``reset.time_only`` service or a timeline
/// restart in Isaac Sim. A pending ``sleep_until()`` waiting for a deadline of the previous
/// timeline would then block until simulation time catches up with that deadline again.
///
/// When ros time is active, ``rclcpp::Clock::sleep_until()`` is woken up by every update of the
/// clock but keeps sleeping until the given deadline is reached, so a backwards jump alone does
/// not end the sleep. The jump callback installed by this class cancels the pending sleep, which
/// lets the control loop continue in the new timeline right away.
///
/// The callback runs on the thread that updates the clock, which is why the communication with
/// the control loop uses atomics only.
class SimTimeJumpHandler
{
public:
  explicit SimTimeJumpHandler(const rclcpp::Clock::SharedPtr & clock);

  /// Returns true once for every backward jump that happened since the last call.
  ///
  /// A jump handler is invoked twice per jump, once before and once after the clock is updated.
  /// Only the post-jump invocation is registered, so a single jump is reported exactly once.
  bool consume_backward_jump();

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
  std::atomic_bool backward_jump_{false};
};
}  // namespace controller_manager

/// Returns the period of the current cycle and moves the time base of the control loop forward.
///
/// \param time_base_reset set to true if the clock jumped backwards since the previous cycle. The
///   stored time base then belongs to the timeline that was left behind, so the difference to the
///   current time does not describe a period that controllers or hardware components could work
///   with. In that case the current time becomes the new time base and the nominal period is
///   returned instead of a negative one.
rclcpp::Duration update_time_base_and_get_period(
  controller_manager::ControlLoopState & state, const rclcpp::Time & current_time,
  bool time_base_reset);

bool sleep_for_sim_time(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  controller_manager::ControlLoopState & state);

void sleep_for_blocking_read_write(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  const controller_manager::ControlLoopTimingConfig & config,
  controller_manager::ControlLoopState & state);

void sleep_for_periodic_cycle(
  std::shared_ptr<controller_manager::ControllerManager> cm,
  const controller_manager::ControlLoopTimingConfig & config,
  controller_manager::ControlLoopState & state);
