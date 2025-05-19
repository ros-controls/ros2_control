// Copyright 2020 ros2_control development team
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

#ifndef TRANSMISSION_INTERFACE__HANDLE_HPP_
#define TRANSMISSION_INTERFACE__HANDLE_HPP_

#include <optional>
#include <string>
#include "hardware_interface/macros.hpp"

namespace transmission_interface
{
class Handle
{
public:
  Handle(const std::string & prefix_name, const std::string & interface_name, double * value_ptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  Handle(const Handle & other) = default;

  Handle(Handle && other) = default;

  Handle & operator=(const Handle & other) = default;

  Handle & operator=(Handle && other) = default;

  virtual ~Handle() = default;

  /// Returns true if handle references a value.
  inline operator bool() const { return value_ptr_ != nullptr; }

  const std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  const std::string & get_prefix_name() const { return prefix_name_; }

  double get_value() const
  {
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }

  [[deprecated(
    "For Transmission Handles use get_value() instead to retrieve the value. This method will be "
    "removed by the ROS 2 Kilted Kaiju release.")]]
  std::optional<double> get_optional() const
  {
    if (value_ptr_)
    {
      return *value_ptr_;
    }
    return std::nullopt;
  }

  bool set_value(double value)
  {
    THROW_ON_NULLPTR(this->value_ptr_);
    *this->value_ptr_ = value;
    return true;
  }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double * value_ptr_ = nullptr;
};

/** A handle used to get and set a value on a given actuator interface. */
class ActuatorHandle : public transmission_interface::Handle
{
public:
  using transmission_interface::Handle::Handle;
};

/** A handle used to get and set a value on a given joint interface. */
class JointHandle : public transmission_interface::Handle
{
public:
  using transmission_interface::Handle::Handle;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__HANDLE_HPP_
