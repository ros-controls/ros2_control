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

#include <cstdint>

namespace hardware_interface
{
enum class return_type : std::uint8_t
{
  OK = 0,
  ERROR = 1,

  CLAIMED_ERROR = 10,
  ALREADY_CLAIMED = 11,
  NOT_CLAIMED = 11,
  UNATHORIZED_UNCLAIM = 13,
  NON_CLAIMED_WRITE = 15,

  CAN_NOT_READ = 20,

  INTERFACE_NOT_FOUND = 30,
  INTERFACE_VALUE_SIZE_NOT_EQUAL = 31,
  INTERFACE_NOT_PROVIDED = 32,

  COMMAND_OUT_OF_LIMITS = 40,
};

using hardware_interface_ret_t = return_type;
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
