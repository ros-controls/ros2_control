// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_
#define HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_

#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include "hardware_interface/handle.hpp"
#include "rclcpp/logging.hpp"

namespace hardware_interface
{
class LoanedCommandInterface
{
public:
  using Deleter = std::function<void(void)>;

  [[deprecated("Replaced by the new version using shared_ptr")]] explicit LoanedCommandInterface(
    CommandInterface & command_interface)
  : LoanedCommandInterface(command_interface, nullptr)
  {
  }

  [[deprecated("Replaced by the new version using shared_ptr")]] LoanedCommandInterface(
    CommandInterface & command_interface, Deleter && deleter)
  : command_interface_(command_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  LoanedCommandInterface(CommandInterface::SharedPtr command_interface, Deleter && deleter)
  : command_interface_(*command_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  LoanedCommandInterface(const LoanedCommandInterface & other) = delete;

  LoanedCommandInterface(LoanedCommandInterface && other) = default;

  virtual ~LoanedCommandInterface()
  {
    auto logger = rclcpp::get_logger(command_interface_.get_name());
    RCLCPP_WARN_EXPRESSION(
      rclcpp::get_logger(get_name()),
      (get_value_statistics_.failed_counter > 0 || get_value_statistics_.timeout_counter > 0),
      "LoanedCommandInterface %s has %u (%.4f %%) timeouts and %u (~ %.4f %%) missed calls out of "
      "%u get_value calls",
      get_name().c_str(), get_value_statistics_.timeout_counter,
      (get_value_statistics_.timeout_counter * 100.0) / get_value_statistics_.total_counter,
      get_value_statistics_.failed_counter,
      (get_value_statistics_.failed_counter * 10.0) / get_value_statistics_.total_counter,
      get_value_statistics_.total_counter);
    RCLCPP_WARN_EXPRESSION(
      rclcpp::get_logger(get_name()),
      (set_value_statistics_.failed_counter > 0 || set_value_statistics_.timeout_counter > 0),
      "LoanedCommandInterface %s has %u (%.4f %%) timeouts and  %u (~ %.4f %%) missed calls out of "
      "%u set_value calls",
      get_name().c_str(), set_value_statistics_.timeout_counter,
      (set_value_statistics_.timeout_counter * 100.0) / set_value_statistics_.total_counter,
      set_value_statistics_.failed_counter,
      (set_value_statistics_.failed_counter * 10.0) / set_value_statistics_.total_counter,
      set_value_statistics_.total_counter);
    if (deleter_)
    {
      deleter_();
    }
  }

  const std::string & get_name() const { return command_interface_.get_name(); }

  const std::string & get_interface_name() const { return command_interface_.get_interface_name(); }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return command_interface_.get_name();
  }

  const std::string & get_prefix_name() const { return command_interface_.get_prefix_name(); }

  /**
   * @brief Set the value of the command interface.
   * @tparam T The type of the value to be set.
   * @param value The value to set.
   * @param max_tries The maximum number of tries to set the value.
   * @return true if the value is set successfully, false otherwise.
   *
   * @note The method is thread-safe and non-blocking.
   * @note When different threads access the internal handle at same instance, and if they are
   * unable to lock the handle to set the value, the handle returns false. If the operation is
   * successful, the handle is updated and returns true.
   * @note The method will try to set the value max_tries times before returning false. The method
   * will yield the thread between tries. If the value is set successfully, the method returns true
   * immediately.
   */
  template <typename T>
  [[nodiscard]] bool set_value(const T & value, unsigned int max_tries = 10)
  {
    unsigned int nr_tries = 0;
    ++set_value_statistics_.total_counter;
    while (!command_interface_.set_limited_value(value))
    {
      ++set_value_statistics_.failed_counter;
      ++nr_tries;
      if (nr_tries == max_tries)
      {
        ++set_value_statistics_.timeout_counter;
        return false;
      }
      std::this_thread::yield();
    }
    return true;
  }

  [[deprecated(
    "Use std::optional<T> get_optional() instead to retrieve the value. This method will be "
    "removed by the ROS 2 Kilted Kaiju release.")]]
  double get_value() const
  {
    double value = std::numeric_limits<double>::quiet_NaN();
    if (get_value(value))
    {
      return value;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  /**
   * @brief Get the value of the command interface.
   * @tparam T The type of the value to be retrieved.
   * @return The value of the command interface if it accessed successfully, std::nullopt otherwise.
   * @param max_tries The maximum number of tries to get the value.
   *
   * @note The method is thread-safe and non-blocking.
   * @note When different threads access the internal handle at same instance, and if they are
   * unable to lock the handle to access the value, the handle returns std::nullopt. If the
   * operation is successful, the value is returned.
   * @note The method will try to get the value max_tries times before returning std::nullopt. The
   * method will yield the thread between tries. If the value is retrieved successfully, the method
   * returns the value immediately.
   */
  template <typename T = double>
  [[nodiscard]] std::optional<T> get_optional(unsigned int max_tries = 10) const
  {
    unsigned int nr_tries = 0;
    do
    {
      ++get_value_statistics_.total_counter;
      const std::optional<T> data = command_interface_.get_optional<T>();
      if (data.has_value())
      {
        return data;
      }
      ++get_value_statistics_.failed_counter;
      ++nr_tries;
      std::this_thread::yield();
    } while (nr_tries < max_tries);

    ++get_value_statistics_.timeout_counter;
    return std::nullopt;
  }

  /**
   * @brief Get the value of the command interface.
   * @tparam T The type of the value to be retrieved.
   * @param value The value of the command interface.
   * @param max_tries The maximum number of tries to get the value.
   * @return true if the value is accessed successfully, false otherwise.
   *
   * @note The method is thread-safe and non-blocking.
   * @note When different threads access the internal handle at same instance, and if they are
   * unable to lock the handle to access the value, the handle returns false. If the operation is
   * successful, the value is updated and returns true.
   * @note The method will try to get the value max_tries times before returning false. The method
   * will yield the thread between tries. If the value is updated successfully, the method returns
   * true immediately.
   */
  template <typename T>
  [[deprecated(
    "Use std::optional<T> get_optional() instead to retrieve the value. This method will be "
    "removed by the ROS 2 Kilted Kaiju release.")]] [[nodiscard]] bool
  get_value(T & value, unsigned int max_tries = 10) const
  {
    unsigned int nr_tries = 0;
    ++get_value_statistics_.total_counter;
    while (!command_interface_.get_value(value))
    {
      ++get_value_statistics_.failed_counter;
      ++nr_tries;
      if (nr_tries == max_tries)
      {
        ++get_value_statistics_.timeout_counter;
        return false;
      }
      std::this_thread::yield();
    }
    return true;
  }

protected:
  CommandInterface & command_interface_;
  Deleter deleter_;

private:
  struct HandleRTStatistics
  {
    unsigned int total_counter = 0;
    unsigned int failed_counter = 0;
    unsigned int timeout_counter = 0;
  };
  mutable HandleRTStatistics get_value_statistics_;
  HandleRTStatistics set_value_statistics_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__LOANED_COMMAND_INTERFACE_HPP_
