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

namespace hardware_interface
{
enum class HardwareReturnType
{
  HW_RET_OK = 0,
  HW_RET_ERROR = 1,

  HW_RET_CLAIMED_ERROR = 10,
  HW_RET_ALREADY_CLAIMED = 11,
  HW_RET_NOT_CLAIMED = 11,
  HW_RET_UNATHORIZED_UNCLAIM = 13,
  HW_RET_NON_CLAIMED_WRITE = 15,

  HW_RET_CAN_NOT_READ = 20,
};

typedef HardwareReturnType hardware_interface_ret_t;
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_RETURN_VALUES_HPP_
