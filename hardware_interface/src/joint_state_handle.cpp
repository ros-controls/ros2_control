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

#include "hardware_interface/joint_state_handle.hpp"

#include <stdexcept>
#include <string>

#include "hardware_interface/macros.hpp"

namespace hardware_interface
{

JointStateHandle::JointStateHandle()
: name_(),
  pos_(nullptr),
  vel_(nullptr),
  eff_(nullptr)
{}

JointStateHandle::JointStateHandle(
  const std::string & name,
  const double * pos,
  const double * vel,
  const double * eff)
: name_(name), pos_(pos), vel_(vel), eff_(eff)
{
  THROW_ON_NULLPTR(pos)
  THROW_ON_NULLPTR(vel)
  THROW_ON_NULLPTR(eff)
}

const std::string &
JointStateHandle::get_name() const
{
  return name_;
}

double
JointStateHandle::get_position() const
{
  THROW_ON_NULLPTR(pos_)

  return *pos_;
}

double
JointStateHandle::get_velocity() const
{
  THROW_ON_NULLPTR(vel_)

  return *vel_;
}

double
JointStateHandle::get_effort() const
{
  THROW_ON_NULLPTR(eff_)

  return *eff_;
}

bool JointStateHandle::valid_pointers() const
{
  return pos_ && vel_ && eff_;
}

}  // namespace hardware_interface
