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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_WARNING_SIGNALS_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_WARNING_SIGNALS_HPP_

#include <cstdint>
#include <vector>

namespace hardware_interface
{
// Count of how many different warn signals there are that can be reported.
const size_t warning_signal_count = 32;

constexpr char WARNING_SIGNAL_INTERFACE_NAME[] = "WARNING_SIGNAL";
// Available warning signals names mapping to position in the interface
enum class warning_signal : std::uint8_t
{
  WARNING_SIGNAL_0 = 0,
  WARNING_SIGNAL_1 = 1,
  WARNING_SIGNAL_2 = 2,
  WARNING_SIGNAL_3 = 3,
  WARNING_SIGNAL_4 = 4,
  WARNING_SIGNAL_5 = 5,
  WARNING_SIGNAL_6 = 6,
  WARNING_SIGNAL_7 = 7,
  WARNING_SIGNAL_8 = 8,
  WARNING_SIGNAL_9 = 9,
  WARNING_SIGNAL_10 = 10,
  WARNING_SIGNAL_11 = 11,
  WARNING_SIGNAL_12 = 12,
  WARNING_SIGNAL_13 = 13,
  WARNING_SIGNAL_14 = 14,
  WARNING_SIGNAL_15 = 15,
  WARNING_SIGNAL_16 = 16,
  WARNING_SIGNAL_17 = 17,
  WARNING_SIGNAL_18 = 18,
  WARNING_SIGNAL_19 = 19,
  WARNING_SIGNAL_20 = 20,
  WARNING_SIGNAL_21 = 21,
  WARNING_SIGNAL_22 = 22,
  WARNING_SIGNAL_23 = 23,
  WARNING_SIGNAL_24 = 24,
  WARNING_SIGNAL_25 = 25,
  WARNING_SIGNAL_26 = 26,
  WARNING_SIGNAL_27 = 27,
  WARNING_SIGNAL_28 = 28,
  WARNING_SIGNAL_29 = 29,
  WARNING_SIGNAL_30 = 30,
  WARNING_SIGNAL_31 = 31
};

constexpr char WARNING_SIGNAL_MESSAGE_INTERFACE_NAME[] = "WARNING_SIGNAL_MESSAGE";
// Available WARNING signal message names
enum class warning_signal_message : std::uint8_t
{
  WARNING_SIGNAL_MESSAGE_0 = 0,
  WARNING_SIGNAL_MESSAGE_1 = 1,
  WARNING_SIGNAL_MESSAGE_2 = 2,
  WARNING_SIGNAL_MESSAGE_3 = 3,
  WARNING_SIGNAL_MESSAGE_4 = 4,
  WARNING_SIGNAL_MESSAGE_5 = 5,
  WARNING_SIGNAL_MESSAGE_6 = 6,
  WARNING_SIGNAL_MESSAGE_7 = 7,
  WARNING_SIGNAL_MESSAGE_8 = 8,
  WARNING_SIGNAL_MESSAGE_9 = 9,
  WARNING_SIGNAL_MESSAGE_10 = 10,
  WARNING_SIGNAL_MESSAGE_11 = 11,
  WARNING_SIGNAL_MESSAGE_12 = 12,
  WARNING_SIGNAL_MESSAGE_13 = 13,
  WARNING_SIGNAL_MESSAGE_14 = 14,
  WARNING_SIGNAL_MESSAGE_15 = 15,
  WARNING_SIGNAL_MESSAGE_16 = 16,
  WARNING_SIGNAL_MESSAGE_17 = 17,
  WARNING_SIGNAL_MESSAGE_18 = 18,
  WARNING_SIGNAL_MESSAGE_19 = 19,
  WARNING_SIGNAL_MESSAGE_20 = 20,
  WARNING_SIGNAL_MESSAGE_21 = 21,
  WARNING_SIGNAL_MESSAGE_22 = 22,
  WARNING_SIGNAL_MESSAGE_23 = 23,
  WARNING_SIGNAL_MESSAGE_24 = 24,
  WARNING_SIGNAL_MESSAGE_25 = 25,
  WARNING_SIGNAL_MESSAGE_26 = 26,
  WARNING_SIGNAL_MESSAGE_27 = 27,
  WARNING_SIGNAL_MESSAGE_28 = 28,
  WARNING_SIGNAL_MESSAGE_29 = 29,
  WARNING_SIGNAL_MESSAGE_30 = 30,
  WARNING_SIGNAL_MESSAGE_31 = 31
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_WARNING_SIGNALS_HPP_
