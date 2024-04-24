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
/// Constant defining position interface name
constexpr char HW_IF_POSITION[] = "position";
/// Constant defining velocity interface name
constexpr char HW_IF_VELOCITY[] = "velocity";
/// Constant defining acceleration interface name
constexpr char HW_IF_ACCELERATION[] = "acceleration";
/// Constant defining effort interface name
constexpr char HW_IF_EFFORT[] = "effort";
/// Constant defining torque interface name
constexpr char HW_IF_TORQUE[] = "torque";
/// Constant defining force interface name
constexpr char HW_IF_FORCE[] = "force";
/// Constant defining current interface name
constexpr char HW_IF_CURRENT[] = "current";
/// Constant defining temperature interface name
constexpr char HW_IF_TEMPERATURE[] = "temperature";

/// Gains interface constants
/// Constant defining proportional gain interface name
constexpr char HW_IF_PROPORTIONAL_GAIN[] = "proportional";
/// Constant defining integral gain interface name
constexpr char HW_IF_INTEGRAL_GAIN[] = "integral";
/// Constant defining derivative gain interface name
constexpr char HW_IF_DERIVATIVE_GAIN[] = "derivative";
/// Constant defining integral clamp interface name
constexpr char HW_IF_INTEGRAL_CLAMP_MAX[] = "integral_clamp_max";
/// Constant defining integral clamp interface name
constexpr char HW_IF_INTEGRAL_CLAMP_MIN[] = "integral_clamp_min";
/// Constant defining the feedforward interface name
constexpr char HW_IF_FEEDFORWARD[] = "feedforward";
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_
