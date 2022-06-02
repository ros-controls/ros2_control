// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/duration.hpp"

/**
 * \brief      Run a control loop at a set period
 * \param[in]  period The period of the loop in nanoseconds
 * \param[in]  now invocable that returns rclcpp::Time
 * \param[in]  ok invocable that returns true if the loop should continue
 * \param[in]  do_work invocable that takes the current time and measured period
 *             and does the work of the control loop
 * \param[in]  sleep_until invocable that takes a rclcpp::Time and sleeps until
 *             that time
 */
template <
  typename NowInvocable, typename OkInvocable, typename WorkInvocable, typename SleepInvocable>
void ControlLoop(
  std::chrono::nanoseconds period, NowInvocable now, OkInvocable ok, WorkInvocable do_work,
  SleepInvocable sleep_until)
{
  auto current_time = now();
  auto measured_period = rclcpp::Duration{0, 0};
  auto next_time = current_time;

  while (ok())
  {
    auto const last_time = current_time;
    current_time = now();
    measured_period = current_time - last_time;
    next_time = next_time + period;

    do_work(current_time, measured_period);
    sleep_until(next_time);
  }
}

#endif  // CONTROLLER_MANAGER__CONTROL_LOOP_HPP_
