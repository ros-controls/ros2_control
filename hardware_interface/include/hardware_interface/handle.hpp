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

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/macros.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// A handle used to get and set a value on a given interface.
class StateHandle
{
  virtual double read_state() = 0;
};

class CommandHandle
{
  virtual void write_command(const double & command) = 0;
};
class Handle
{
public:
  Handle(const std::string & prefix_name, const std::string & interface_name)
  : prefix_name_(prefix_name), interface_name_(interface_name)
  {
  }

  explicit Handle(const std::string & interface_name) : interface_name_(interface_name) {}

  explicit Handle(const char * interface_name) : interface_name_(interface_name) {}

  // Handle should be unique
  Handle(const Handle & other) = delete;

  Handle(Handle && other) = default;

  Handle & operator=(const Handle & other) = default;

  Handle & operator=(Handle && other) = default;

  virtual ~Handle() = default;

  const std::string get_name() const { return prefix_name_ + "/" + interface_name_; }

  const std::string & get_interface_name() const { return interface_name_; }

  [[deprecated(
    "Replaced by get_name method, which is semantically more correct")]] const std::string
  get_full_name() const
  {
    return get_name();
  }

  const std::string & get_prefix_name() const { return prefix_name_; }

  // only expose to hw => we could add functionality which keeps track
  // if values have been read/are new for ctrls.
  // Same from ctrl -> hw via State-/CommandHandle
  void hw_set_state(const double & value) { value_ = value; }

  double hw_get_state() const { return value_; }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double value_;
};

class StateInterface : public Handle, public StateHandle
{
public:
  StateInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description.prefix_name, interface_description.interface_info.name)
  {
  }

  StateInterface(const StateInterface & other) = delete;

  StateInterface(StateInterface && other) = default;

  double read_state() override { return value_; }

  using Handle::Handle;
};

class CommandInterface : public Handle, public CommandHandle, public StateHandle
{
public:
  CommandInterface(const InterfaceDescription & interface_description)
  : Handle(interface_description.prefix_name, interface_description.interface_info.name)
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

  double read_state() override { return value_; }

  void write_command(const double & command) override { value_ = command; }

  using Handle::Handle;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__HANDLE_HPP_
