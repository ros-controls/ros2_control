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

#ifndef CONTROLLER_MANAGER__CONTROL_LOOP_HPP_
#define CONTROLLER_MANAGER__CONTROL_LOOP_HPP_

#include <chrono>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/time.hpp"

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
};

void initialize_control_loop_timing(ControllerManager & cm, ControlLoopState & state);

/// Runs one read/update/write cycle and applies the post-cycle synchronization policy.
/// Returns false if the loop must abort (sim-time sleep failure).
bool run_control_loop_cycle(
  ControllerManager & cm, const ControlLoopTimingConfig & config, ControlLoopState & state);

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROL_LOOP_HPP_
