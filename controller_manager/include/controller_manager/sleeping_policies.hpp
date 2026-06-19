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

#include <memory>

#include <rclcpp/logging.hpp>

#include "controller_manager/controller_manager.hpp"

namespace controller_manager
{
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
}  // namespace controller_manager

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
