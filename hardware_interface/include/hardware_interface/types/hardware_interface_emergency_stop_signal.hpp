// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_EMERGENCY_STOP_SIGNAL_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_EMERGENCY_STOP_SIGNAL_HPP_

#include <cstdint>
#include <vector>

namespace hardware_interface
{
// Count of how many different emergency stop signals there are that can be reported.
const size_t emergency_stop_signal_count = 1;

constexpr char EMERGENCY_STOP_SIGNAL[] = "EMERGENCY_STOP_SIGNAL";
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_EMERGENCY_STOP_SIGNAL_HPP_
