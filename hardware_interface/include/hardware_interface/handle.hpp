// Copyright 2020 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__HANDLE_HPP_
#define HARDWARE_INTERFACE__HANDLE_HPP_

#include <string>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/** A handle used to get and set a value on a given interface. */
template<class HandleType>
class Handle
{
public:
  HARDWARE_INTERFACE_PUBLIC
  Handle(
    const std::string & name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : name_(name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  HARDWARE_INTERFACE_PUBLIC
  HandleType with_value_ptr(double * value_ptr)
  {
    return HandleType(name_, interface_name_, value_ptr);
  }

  HARDWARE_INTERFACE_PUBLIC
  const std::string & get_name() const
  {
    return name_;
  }

  HARDWARE_INTERFACE_PUBLIC
  const std::string & get_interface_name() const
  {
    return interface_name_;
  }

  HARDWARE_INTERFACE_PUBLIC
  double get_value() const
  {
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }

  HARDWARE_INTERFACE_PUBLIC
  void set_value(double value)
  {
    THROW_ON_NULLPTR(value_ptr_);
    *value_ptr_ = value;
  }

protected:
  std::string name_;
  std::string interface_name_;
  double * value_ptr_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
