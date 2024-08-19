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
#include <variant>

#include "hardware_interface/macros.hpp"

namespace hardware_interface
{

using HANDLE_DATATYPE = std::variant<double>;

/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  Handle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  explicit Handle(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  explicit Handle(const char * interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
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

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

  double get_value() const
  {
    // BEGIN (Handle export change): for backward compatibility
    // TODO(Manuel) return value_ if old functionality is removed
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
    // END
  }

  void set_value(double value)
  {
    // BEGIN (Handle export change): for backward compatibility
    // TODO(Manuel) set value_ directly if old functionality is removed
    THROW_ON_NULLPTR(this->value_ptr_);
    *this->value_ptr_ = value;
    // END
  }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  HANDLE_DATATYPE value_;
  // BEGIN (Handle export change): for backward compatibility
  // TODO(Manuel) redeclare as HANDLE_DATATYPE * value_ptr_ if old functionality is removed
  double * value_ptr_;
  // END
};

class StateInterface : public Handle
{
public:
  explicit StateInterface(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : Handle(prefix_name, interface_name, value_ptr)
  {
  }

  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using Handle::Handle;
};

class CommandInterface : public Handle
{
public:
  explicit CommandInterface(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : Handle(prefix_name, interface_name, value_ptr)
  {
  }
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  CommandInterface(const CommandInterface & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using Handle::Handle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
