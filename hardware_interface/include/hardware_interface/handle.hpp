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
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
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

  Handle(const Handle & other) noexcept
  {
    std::unique_lock<std::shared_mutex> lock(other.handle_mutex_);
    std::unique_lock<std::shared_mutex> lock_this(handle_mutex_);
    prefix_name_ = other.prefix_name_;
    interface_name_ = other.interface_name_;
    handle_name_ = other.handle_name_;
    value_ = other.value_;
    value_ptr_ = other.value_ptr_;
  }

  Handle(Handle && other) noexcept
  {
    std::unique_lock<std::shared_mutex> lock(other.handle_mutex_);
    std::unique_lock<std::shared_mutex> lock_this(handle_mutex_);
    prefix_name_ = std::move(other.prefix_name_);
    interface_name_ = std::move(other.interface_name_);
    handle_name_ = std::move(other.handle_name_);
    value_ = std::move(other.value_);
    value_ptr_ = std::move(other.value_ptr_);
  }

  Handle & operator=(const Handle & other)
  {
    if (this != &other)
    {
      std::unique_lock<std::shared_mutex> lock(other.handle_mutex_);
      std::unique_lock<std::shared_mutex> lock_this(handle_mutex_);
      prefix_name_ = other.prefix_name_;
      interface_name_ = other.interface_name_;
      handle_name_ = other.handle_name_;
      value_ = other.value_;
      value_ptr_ = other.value_ptr_;
    }
    return *this;
  }

  Handle & operator=(Handle && other)
  {
    if (this != &other)
    {
      std::unique_lock<std::shared_mutex> lock(other.handle_mutex_);
      std::unique_lock<std::shared_mutex> lock_this(handle_mutex_);
      prefix_name_ = std::move(other.prefix_name_);
      interface_name_ = std::move(other.interface_name_);
      handle_name_ = std::move(other.handle_name_);
      value_ = std::move(other.value_);
      value_ptr_ = std::move(other.value_ptr_);
    }
    return *this;
  }

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

  [[deprecated("Use bool get_value(double & value) instead to retrieve the value.")]]
  double get_value() const
  {
    std::shared_lock<std::shared_mutex> lock(handle_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    // BEGIN (Handle export change): for backward compatibility
    // TODO(Manuel) return value_ if old functionality is removed
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
    // END
  }

  [[nodiscard]] bool get_value(double & value) const
  {
    std::shared_lock<std::shared_mutex> lock(handle_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      return false;
    }
    // BEGIN (Handle export change): for backward compatibility
    // TODO(Manuel) set value directly if old functionality is removed
    THROW_ON_NULLPTR(value_ptr_);
    value = *value_ptr_;
    return true;
    // END
  }

  [[nodiscard]] bool set_value(double value)
  {
    std::unique_lock<std::shared_mutex> lock(handle_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      return false;
    }
    // BEGIN (Handle export change): for backward compatibility
    // TODO(Manuel) set value_ directly if old functionality is removed
    THROW_ON_NULLPTR(this->value_ptr_);
    *this->value_ptr_ = value;
    return true;
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
  mutable std::shared_mutex handle_mutex_;
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
