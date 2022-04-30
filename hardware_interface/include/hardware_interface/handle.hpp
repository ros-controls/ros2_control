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
#include <vector>
#include <utility>

#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

#include <hardware_interface/hardware_info.hpp>

namespace hardware_interface
{
struct InterfaceConfiguration
{
  std::string joint_name;
  InterfaceInfo interface_info;
};

/// A variant class used to store value on a given interface
class Variant
{
enum value_type
{
  boolean = 0,
  integer,
  floating,
  vector_boolean,
  vector_integer,
  vector_floating,
};

public:
  Variant(bool value)
  {
    value_type_ = value_type::boolean;
    bool_value_ = value;
  }

  Variant(int value)
  {
    value_type_ = value_type::integer;
    int_value_ = value;
  }

  Variant(double value)
  {
    value_type_ = value_type::floating;
    double_value_ = value;
  }

  Variant(const std::vector<bool> & value)
  {
    value_type_ = value_type::vector_boolean;
    vector_bool_value_ = value;
  }

  Variant(const std::vector<int> & value)
  {
    value_type_ = value_type::vector_integer;
    vector_int_value_ = value;
  }

  Variant(const std::vector<double> & value)
  {
    value_type_ = value_type::vector_floating;
    vector_double_value_ = value;
  }

  void set_value(bool value)
  {
    bool_value_ = value;
  }

  void set_value(int value)
  {
    int_value_ = value;
  }

  void set_value(double value)
  {
    double_value_ = value;
  }

  template <typename DataT, typename std::enable_if<std::is_integral<DataT>, bool> = true>
  DataT get_value()
  {
    return bool_value_;
  }

  template <typename DataT, typename std::enable_if<std::is_integral<DataT>, int> = true>
  DataT get_value()
  {
    return int_value_;
  }

  template <typename DataT, typename std::enable_if<std::is_floating_point<DataT>, double> = true>
  DataT get_value()
  {
    return double_value_;
  }

  bool is_valid()
  {
    // Check if nan or empty to return "false"
  }

private:
  value_type value_type_;
  bool bool_value_;
  int int_value_;
  double double_value_;
  std::vector<bool> vector_bool_value_;
  std::vector<int> vector_int_value_;
  std::vector<double> vector_double_value_;
};

/// A handle used to get and set a value on a given interface.
class ReadOnlyHandle
{
public:
  ReadOnlyHandle(
    const std::string & name, const std::string & interface_name, Variant * value_ptr = nullptr)
  : name_(name), interface_name_(interface_name), value_ptr_(value_ptr)
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

  const std::string & get_name() const { return name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  const std::string get_full_name() const { return name_ + "/" + interface_name_; }

  template <typedef DataT>
  DataT get_value() const
  {
    THROW_ON_NULLPTR(value_ptr_);
    return value_ptr_->get_value();
  }

protected:
  std::string name_;
  std::string interface_name_;
  Variant * value_ptr_;
};

class ReadWriteHandle : public ReadOnlyHandle
{
public:
  ReadWriteHandle(
    const std::string & name, const std::string & interface_name, Variant * value_ptr = nullptr)
  : ReadOnlyHandle(name, interface_name, value_ptr)
  {
  }

  explicit ReadWriteHandle(const std::string & interface_name) : ReadOnlyHandle(interface_name) {}

  explicit ReadWriteHandle(const char * interface_name) : ReadOnlyHandle(interface_name) {}

  ReadWriteHandle(const ReadWriteHandle & other) = default;

  ReadWriteHandle(ReadWriteHandle && other) = default;

  ReadWriteHandle & operator=(const ReadWriteHandle & other) = default;

  ReadWriteHandle & operator=(ReadWriteHandle && other) = default;

  virtual ~ReadWriteHandle() = default;

  template <typename DataT>
  void set_value(DataT value)
  {
    THROW_ON_NULLPTR(this->value_ptr_);
    this->value_ptr_->set_value(value);
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
