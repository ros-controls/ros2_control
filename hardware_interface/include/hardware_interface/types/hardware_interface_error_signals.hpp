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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_

#include <cstdint>
#include <vector>

namespace hardware_interface
{

// Count of how many different error signals there are that can be reported.
const size_t error_signal_count = 32;

constexpr char ERROR_SIGNAL_INTERFACE_NAME[] = "ERROR_SIGNAL";
// Available error signal names
enum class error_signal : std::uint8_t
{
  ERROR_SIGNAL_0 = 0,
  ERROR_SIGNAL_1 = 1,
  ERROR_SIGNAL_2 = 2,
  ERROR_SIGNAL_3 = 3,
  ERROR_SIGNAL_4 = 4,
  ERROR_SIGNAL_5 = 5,
  ERROR_SIGNAL_6 = 6,
  ERROR_SIGNAL_7 = 7,
  ERROR_SIGNAL_8 = 8,
  ERROR_SIGNAL_9 = 9,
  ERROR_SIGNAL_10 = 10,
  ERROR_SIGNAL_11 = 11,
  ERROR_SIGNAL_12 = 12,
  ERROR_SIGNAL_13 = 13,
  ERROR_SIGNAL_14 = 14,
  ERROR_SIGNAL_15 = 15,
  ERROR_SIGNAL_16 = 16,
  ERROR_SIGNAL_17 = 17,
  ERROR_SIGNAL_18 = 18,
  ERROR_SIGNAL_19 = 19,
  ERROR_SIGNAL_20 = 20,
  ERROR_SIGNAL_21 = 21,
  ERROR_SIGNAL_22 = 22,
  ERROR_SIGNAL_23 = 23,
  ERROR_SIGNAL_24 = 24,
  ERROR_SIGNAL_25 = 25,
  ERROR_SIGNAL_26 = 26,
  ERROR_SIGNAL_27 = 27,
  ERROR_SIGNAL_28 = 28,
  ERROR_SIGNAL_29 = 29,
  ERROR_SIGNAL_30 = 30,
  ERROR_SIGNAL_31 = 31
};

constexpr char ERROR_SIGNAL_MESSAGE_INTERFACE_NAME[] = "ERROR_SIGNAL_MESSAGE";
// Available WARNING signal message names
enum class error_signal_message : std::uint8_t
{
  ERROR_SIGNAL_MESSAGE_0 = 0,
  ERROR_SIGNAL_MESSAGE_1 = 1,
  ERROR_SIGNAL_MESSAGE_2 = 2,
  ERROR_SIGNAL_MESSAGE_3 = 3,
  ERROR_SIGNAL_MESSAGE_4 = 4,
  ERROR_SIGNAL_MESSAGE_5 = 5,
  ERROR_SIGNAL_MESSAGE_6 = 6,
  ERROR_SIGNAL_MESSAGE_7 = 7,
  ERROR_SIGNAL_MESSAGE_8 = 8,
  ERROR_SIGNAL_MESSAGE_9 = 9,
  ERROR_SIGNAL_MESSAGE_10 = 10,
  ERROR_SIGNAL_MESSAGE_11 = 11,
  ERROR_SIGNAL_MESSAGE_12 = 12,
  ERROR_SIGNAL_MESSAGE_13 = 13,
  ERROR_SIGNAL_MESSAGE_14 = 14,
  ERROR_SIGNAL_MESSAGE_15 = 15,
  ERROR_SIGNAL_MESSAGE_16 = 16,
  ERROR_SIGNAL_MESSAGE_17 = 17,
  ERROR_SIGNAL_MESSAGE_18 = 18,
  ERROR_SIGNAL_MESSAGE_19 = 19,
  ERROR_SIGNAL_MESSAGE_20 = 20,
  ERROR_SIGNAL_MESSAGE_21 = 21,
  ERROR_SIGNAL_MESSAGE_22 = 22,
  ERROR_SIGNAL_MESSAGE_23 = 23,
  ERROR_SIGNAL_MESSAGE_24 = 24,
  ERROR_SIGNAL_MESSAGE_25 = 25,
  ERROR_SIGNAL_MESSAGE_26 = 26,
  ERROR_SIGNAL_MESSAGE_27 = 27,
  ERROR_SIGNAL_MESSAGE_28 = 28,
  ERROR_SIGNAL_MESSAGE_29 = 29,
  ERROR_SIGNAL_MESSAGE_30 = 30,
  ERROR_SIGNAL_MESSAGE_31 = 31
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_ERROR_SIGNALS_HPP_
