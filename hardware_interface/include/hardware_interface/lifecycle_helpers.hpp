// Copyright 2024 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__LIFECYCLE_HELPERS_HPP_
#define HARDWARE_INTERFACE__LIFECYCLE_HELPERS_HPP_

#include <lifecycle_msgs/msg/state.hpp>

namespace hardware_interface
{
constexpr bool lifecycleStateThatRequiresNoAction(const lifecycle_msgs::msg::State::_id_type state)
{
  return state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN ||
         state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED ||
         state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
}
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__LIFECYCLE_HELPERS_HPP_
