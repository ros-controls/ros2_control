// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace hardware_interface
{
enum class return_type : std::uint8_t
{
  OK = 0,
  ERROR = 1,
  DEACTIVATE = 2,
};

/**
 * Struct to store the status of the Hardware read or write methods return state.
 * The status contains information if the cycle was triggered successfully, the result of the
 * cycle method and the execution duration of the method. The status is used to provide
 * feedback to the controller_manager.
 * @var successful: true if it was triggered successfully, false if not.
 * @var result: return_type::OK if update is successfully, otherwise return_type::ERROR.
 * @var execution_time: duration of the execution of the update method.
 */
struct HardwareComponentCycleStatus
{
  bool successful = true;
  return_type result = return_type::OK;
  std::optional<std::chrono::nanoseconds> execution_time = std::nullopt;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
