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

#ifndef HARDWARE_INTERFACE__LOANED_STATE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__LOANED_STATE_INTERFACE_HPP_

#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include "hardware_interface/handle.hpp"
#include "rclcpp/logging.hpp"
namespace hardware_interface
{
class LoanedStateInterface
{
public:
  using Deleter = std::function<void(void)>;

  [[deprecated("Replaced by the new version using shared_ptr")]] explicit LoanedStateInterface(
    const StateInterface & state_interface)
  : LoanedStateInterface(state_interface, nullptr)
  {
  }

  [[deprecated("Replaced by the new version using shared_ptr")]] LoanedStateInterface(
    const StateInterface & state_interface, Deleter && deleter)
  : state_interface_(state_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  explicit LoanedStateInterface(StateInterface::ConstSharedPtr state_interface)
  : LoanedStateInterface(state_interface, nullptr)
  {
  }

  LoanedStateInterface(StateInterface::ConstSharedPtr state_interface, Deleter && deleter)
  : state_interface_(*state_interface), deleter_(std::forward<Deleter>(deleter))
  {
  }

  LoanedStateInterface(const LoanedStateInterface & other) = delete;

  LoanedStateInterface(LoanedStateInterface && other) = default;

  virtual ~LoanedStateInterface()
  {
    auto logger = rclcpp::get_logger(state_interface_.get_name());
    RCLCPP_WARN_EXPRESSION(
      logger,
      (get_value_statistics_.failed_counter > 0 || get_value_statistics_.timeout_counter > 0),
      "LoanedStateInterface %s has %u (%.4f %%) timeouts and %u (%.4f %%) missed calls out of %u "
      "get_value calls",
      state_interface_.get_name().c_str(), get_value_statistics_.timeout_counter,
      (get_value_statistics_.timeout_counter * 100.0) / get_value_statistics_.total_counter,
      get_value_statistics_.failed_counter,
      (get_value_statistics_.failed_counter * 10.0) / get_value_statistics_.total_counter,
      get_value_statistics_.total_counter);
    if (deleter_)
    {
      deleter_();
    }
  }

  const std::string & get_name() const { return state_interface_.get_name(); }

  const std::string & get_interface_name() const { return state_interface_.get_interface_name(); }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return state_interface_.get_name();
  }

  const std::string & get_prefix_name() const { return state_interface_.get_prefix_name(); }

  double get_value() const
  {
    double value;
    if (get_value(value))
    {
      return value;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  template <typename T>
  [[nodiscard]] bool get_value(T & value, unsigned int max_tries = 10) const
  {
    unsigned int nr_tries = 0;
    ++get_value_statistics_.total_counter;
    while (!state_interface_.get_value(value))
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
  const StateInterface & state_interface_;
  Deleter deleter_;

private:
  struct HandleRTStatistics
  {
    unsigned int total_counter = 0;
    unsigned int failed_counter = 0;
    unsigned int timeout_counter = 0;
  };
  mutable HandleRTStatistics get_value_statistics_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__LOANED_STATE_INTERFACE_HPP_
