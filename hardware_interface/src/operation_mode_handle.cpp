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

#include "hardware_interface/operation_mode_handle.hpp"

#include <stdexcept>
#include <string>

#include "hardware_interface/macros.hpp"

namespace hardware_interface
{

OperationModeHandle::OperationModeHandle()
: name_(),
  mode_(nullptr)
{}

OperationModeHandle::OperationModeHandle(
  const std::string & name,
  OperationMode * mode)
: name_(name),
  mode_(mode)
{
  THROW_ON_NULLPTR(mode)
}

const std::string &
OperationModeHandle::get_name() const
{
  return name_;
}

void
OperationModeHandle::set_mode(OperationMode mode)
{
  THROW_ON_NULLPTR(mode_)

  * mode_ = mode;
}

bool OperationModeHandle::valid_pointers() const
{
  return mode_;
}

}  // namespace hardware_interface
