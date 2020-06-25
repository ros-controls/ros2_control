// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "hardware_interface/joint_command_handle.hpp"

#include <stdexcept>
#include <string>

#include "hardware_interface/macros.hpp"

namespace hardware_interface
{

JointCommandHandle::JointCommandHandle()
: name_(),
  cmd_(nullptr)
{}

JointCommandHandle::JointCommandHandle(
  const std::string & name,
  double * cmd)
: name_(name), cmd_(cmd)
{
  THROW_ON_NULLPTR(cmd)
}

const std::string &
JointCommandHandle::get_name() const
{
  return name_;
}

double
JointCommandHandle::get_cmd() const
{
  THROW_ON_NULLPTR(cmd_)

  return *cmd_;
}

void
JointCommandHandle::set_cmd(double cmd)
{
  THROW_ON_NULLPTR(cmd_)

  * cmd_ = cmd;
}

bool JointCommandHandle::valid_pointers() const
{
  return cmd_;
}

}  // namespace hardware_interface
