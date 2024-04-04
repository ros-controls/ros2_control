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
#include <utility>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.
class ReadOnlyHandle
{
public:
  ReadOnlyHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : prefix_name_(prefix_name), interface_name_(interface_name), value_ptr_(value_ptr)
  {
  }

  explicit ReadOnlyHandle(const std::string & interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  explicit ReadOnlyHandle(const char * interface_name)
  : interface_name_(interface_name), value_ptr_(nullptr)
  {
  }

  ReadOnlyHandle(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle(ReadOnlyHandle && other) = default;

  ReadOnlyHandle & operator=(const ReadOnlyHandle & other) = default;

  ReadOnlyHandle & operator=(ReadOnlyHandle && other) = default;

  virtual ~ReadOnlyHandle() = default;

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
    THROW_ON_NULLPTR(value_ptr_);
    return *value_ptr_;
  }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double * value_ptr_;
};

class ReadWriteHandle : public ReadOnlyHandle
{
public:
  ReadWriteHandle(
    const std::string & prefix_name, const std::string & interface_name,
    double * value_ptr = nullptr)
  : ReadOnlyHandle(prefix_name, interface_name, value_ptr)
  {
  }

  explicit ReadWriteHandle(const std::string & interface_name) : ReadOnlyHandle(interface_name) {}

  explicit ReadWriteHandle(const char * interface_name) : ReadOnlyHandle(interface_name) {}

  ReadWriteHandle(const ReadWriteHandle & other) = default;

  ReadWriteHandle(ReadWriteHandle && other) = default;

  ReadWriteHandle & operator=(const ReadWriteHandle & other) = default;

  ReadWriteHandle & operator=(ReadWriteHandle && other) = default;

  virtual ~ReadWriteHandle() = default;

  void set_value(double value)
  {
    THROW_ON_NULLPTR(this->value_ptr_);
    *this->value_ptr_ = value;
  }
};

class StateInterface : public ReadOnlyHandle
{
public:
  StateInterface(const StateInterface & other) = default;

  StateInterface(StateInterface && other) = default;

  using ReadOnlyHandle::ReadOnlyHandle;
};

class CommandInterface : public ReadWriteHandle
{
public:
  /// CommandInterface copy constructor is actively deleted.
  /**
   * Command interfaces are having a unique ownership and thus
   * can't be copied in order to avoid simultaneous writes to
   * the same resource.
   */
  CommandInterface(const CommandInterface & other) = delete;

  CommandInterface(CommandInterface && other) = default;

  using ReadWriteHandle::ReadWriteHandle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
