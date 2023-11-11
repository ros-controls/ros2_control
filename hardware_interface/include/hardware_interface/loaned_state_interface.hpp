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
#include <string>
#include <utility>
#include <variant>

#include "hardware_interface/handle.hpp"

namespace hardware_interface
{
class LoanedStateInterface
{
public:
  using Deleter = std::function<void(void)>;

  explicit LoanedStateInterface(StateInterface & state_interface)
  : LoanedStateInterface(state_interface, nullptr)
  {
  }

  LoanedStateInterface(StateInterface & state_interface, Deleter && deleter)
  : state_interface_(&state_interface), deleter_(std::forward<Deleter>(deleter)), using_async_state_interface_(false)
  {
  }

  explicit LoanedStateInterface(AsyncStateInterface & state_interface)
  : LoanedStateInterface(state_interface, nullptr)
  {
  }

  LoanedStateInterface(AsyncStateInterface & state_interface, Deleter && deleter)
  : state_interface_(&state_interface), deleter_(std::forward<Deleter>(deleter)), using_async_state_interface_(true)
  {
  }

  LoanedStateInterface(const LoanedStateInterface & other) = delete;

  LoanedStateInterface(LoanedStateInterface && other) = default;

  virtual ~LoanedStateInterface()
  {
    if (deleter_)
    {
      deleter_();
    }
  }

  const std::string get_name() const { return using_async_state_interface_ ? std::get<1>(state_interface_)->get_name() : std::get<0>(state_interface_)->get_name(); }

  const std::string & get_interface_name() const { return using_async_state_interface_ ? std::get<1>(state_interface_)->get_interface_name() : std::get<0>(state_interface_)->get_interface_name(); }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return using_async_state_interface_ ? std::get<1>(state_interface_)->get_name() : std::get<0>(state_interface_)->get_name();
  }

  const std::string & get_prefix_name() const { return using_async_state_interface_ ? std::get<1>(state_interface_)->get_prefix_name() : std::get<0>(state_interface_)->get_prefix_name(); }

  double get_value() const { return using_async_state_interface_ ? std::get<1>(state_interface_)->get_value() : std::get<0>(state_interface_)->get_value(); }

protected:
  std::variant<StateInterface*, AsyncStateInterface*> state_interface_;
  Deleter deleter_;
  const bool using_async_state_interface_ = false;

};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__LOANED_STATE_INTERFACE_HPP_
