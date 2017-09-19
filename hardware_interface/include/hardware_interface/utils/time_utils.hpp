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

#ifndef HARDWARE_INTERFACE__UTILS__TIME_UTILS_HPP_
#define HARDWARE_INTERFACE__UTILS__TIME_UTILS_HPP_
/// time manipulation functions that should be replaced by ROS2 client library in the future
/**
 * http://docs.ros.org/diamondback/api/rostime/html/time_8h_source.html
 * http://docs.ros.org/latest/api/rostime/html/namespaceros.html
 */
#include <climits>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace hardware_interface
{
namespace utils
{
/// nomalize seconds and nanoseconds
/**
 * implementation copied from ROS1:
 * http://docs.ros.org/latest/api/rostime/html/namespaceros.html#ac5dc4b680792b93a837ef3b577a7fd79
 */
inline
void
normalize_sec_n_sec_unsigned(int64_t & sec, int64_t & nsec)
{
  int64_t nsec_part = nsec % 1000000000L;
  int64_t sec_part = sec + nsec / 1000000000L;
  if (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > UINT_MAX) {
    throw std::runtime_error("Time is out of dual 32-bit range");
  }

  sec = sec_part;
  nsec = nsec_part;
}

/// Time addition
/**
 * return time + duration
 */
inline
builtin_interfaces::msg::Time
time_add(
  const builtin_interfaces::msg::Time & time, const builtin_interfaces::msg::Duration & duration)
{
  int64_t time_s = static_cast<int64_t>(time.sec) + static_cast<int64_t>(duration.sec);
  int64_t time_ns = static_cast<int64_t>(time.nanosec) + static_cast<int64_t>(duration.nanosec);
  normalize_sec_n_sec_unsigned(time_s, time_ns);

  builtin_interfaces::msg::Time time_sum;
  time_sum.sec = static_cast<int32_t>(time_s);
  time_sum.nanosec = static_cast<uint32_t>(time_ns);

  return time_sum;
}

/// Time less than comparison
/**
 * return true if time_1 < time_2
 */
inline
bool
time_less_than(
  const builtin_interfaces::msg::Time & time_1, const builtin_interfaces::msg::Time & time_2)
{
  if (time_1.sec < time_2.sec) {
    return true;
  } else if ((time_1.sec == time_2.sec) && (time_1.nanosec < time_2.nanosec)) {
    return true;
  }
  return false;
}

/// Time less than equals comparison
/**
 * return true if time_1 < time_2
 */
inline
bool
time_less_than_equal(
  const builtin_interfaces::msg::Time & time_1, const builtin_interfaces::msg::Time & time_2)
{
  if (time_1.sec < time_2.sec) {
    return true;
  } else if ((time_1.sec == time_2.sec) && (time_1.nanosec <= time_2.nanosec)) {
    return true;
  }
  return false;
}

inline
bool
time_is_zero(const builtin_interfaces::msg::Time & time)
{
  return time.sec == 0 && time.nanosec == 0;
}

}  // namespace utils
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__UTILS__TIME_UTILS_HPP_
