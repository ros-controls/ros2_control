// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_

namespace hardware_interface
{
/// Constant defining position interface
constexpr char HW_IF_POSITION[] = "position";
/// Constant defining velocity interface
constexpr char HW_IF_VELOCITY[] = "velocity";
/// Constant defining acceleration interface
constexpr char HW_IF_ACCELERATION[] = "acceleration";
/// Constant defining effort interface
constexpr char HW_IF_EFFORT[] = "effort";
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_
