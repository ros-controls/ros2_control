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

#include <limits>
#include <memory>
#include <string>
#include <variant>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"

namespace hardware_interface
{

using HANDLE_DATATYPE = std::variant<double>;

/// A handle used to get and set a value on a given interface.
class Handle
{
public:
  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  Handle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name),
    interface_name_(interface_name),
    handle_name_(prefix_name_ + "/" + interface_name_),
    value_ptr_(value_ptr)
  {
  }

  explicit Handle(const InterfaceDescription & interface_description)
  : prefix_name_(interface_description.get_prefix_name()),
    interface_name_(interface_description.get_interface_name()),
    handle_name_(interface_description.get_name())
  {
    // As soon as multiple datatypes are used in HANDLE_DATATYPE
    // we need to initialize according the type passed in interface description
    value_ = std::numeric_limits<double>::quiet_NaN();
    value_ptr_ = std::get_if<double>(&value_);
  }

  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  explicit Handle(const std::string & interface_name)
  : interface_name_(interface_name), handle_name_("/" + interface_name_), value_ptr_(nullptr)
  {
  }

  [[deprecated("Use InterfaceDescription for initializing the Interface")]]

  explicit Handle(const char * interface_name)
  : interface_name_(interface_name), handle_name_("/" + interface_name_), value_ptr_(nullptr)
  {
  }

  Handle(const Handle & other) = default;

  Handle(Handle && other) = default;

  Handle & operator=(const Handle & other) = default;

  Handle & operator=(Handle && other) = default;

  virtual ~Handle() = default;

  /// Returns true if handle references a value.
  inline operator bool() const { return value_ptr_ != nullptr; }

  const std::string & get_name() const { return handle_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string &
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
  std::string handle_name_;
  HANDLE_DATATYPE value_;
  // BEGIN (Handle export change): for backward compatibility
  // TODO(Manuel) redeclare as HANDLE_DATATYPE * value_ptr_ if old functionality is removed
  double * value_ptr_;
  // END
};

class StateInterface : public Handle
{
public:
  explicit StateInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description)
  {
  }

  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using Handle::Handle;

  using SharedPtr = std::shared_ptr<StateInterface>;
  using ConstSharedPtr = std::shared_ptr<const StateInterface>;
};

class CommandInterface : public Handle
{
public:
  explicit CommandInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description)
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

  using SharedPtr = std::shared_ptr<CommandInterface>;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
