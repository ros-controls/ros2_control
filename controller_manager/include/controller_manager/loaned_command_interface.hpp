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

#ifndef CONTROLLER_MANAGER__LOANED_COMMAND_INTERFACE_HPP_
#define CONTROLLER_MANAGER__LOANED_COMMAND_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"

namespace controller_manager
{

class LoanedCommandInterface
{
public:
  using Deleter = std::function<void (void)>;

  LoanedCommandInterface(
    hardware_interface::CommandInterface & command_interface,
    Deleter && deleter)
  : command_interface_(command_interface),
    deleter_(std::forward<Deleter>(deleter))
  {}

  LoanedCommandInterface(const LoanedCommandInterface & other) = delete;

  LoanedCommandInterface(LoanedCommandInterface && other) = default;

  virtual ~LoanedCommandInterface()
  {
    if (deleter_) {
      deleter_();
    }
  }

  void set_value(double val)
  {
    command_interface_.set_value(val);
  }

private:
  hardware_interface::CommandInterface & command_interface_;
  Deleter deleter_;
};

}  // namespace controller_manager
#endif  // CONTROLLER_MANAGER__LOANED_COMMAND_INTERFACE_HPP_
