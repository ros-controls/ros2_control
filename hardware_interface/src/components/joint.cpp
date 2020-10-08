// Copyright 2020 ros2_control Development Team
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace hardware_interface
{
namespace components
{

Joint::Joint(std::shared_ptr<JointInterface> impl, Deleter deleter)
: impl_(std::move(impl)),
  deleter_(deleter)
{}

Joint::~Joint()
{
  if (deleter_) {deleter_();}
}

return_type Joint::configure(const ComponentInfo & joint_info)
{
  return impl_->configure(joint_info);
}

std::vector<std::string> Joint::get_command_interfaces() const
{
  return impl_->get_command_interfaces();
}

std::vector<std::string> Joint::get_state_interfaces() const
{
  return impl_->get_state_interfaces();
}

return_type Joint::get_command(
  std::vector<double> & command, const std::vector<std::string> & interfaces) const
{
  return impl_->get_command(command, interfaces);
}

return_type Joint::get_command(std::vector<double> & command) const
{
  return impl_->get_command(command);
}

return_type Joint::set_command(
  const std::vector<double> & command, const std::vector<std::string> & interfaces)
{
  return impl_->set_command(command, interfaces);
}

return_type Joint::set_command(const std::vector<double> & command)
{
  return impl_->set_command(command);
}

return_type Joint::get_state(
  std::vector<double> & state, const std::vector<std::string> & interfaces) const
{
  return impl_->get_state(state, interfaces);
}

return_type Joint::get_state(std::vector<double> & state) const
{
  return impl_->get_state(state);
}

return_type Joint::set_state(
  const std::vector<double> & state, const std::vector<std::string> & interfaces)
{
  return impl_->set_state(state, interfaces);
}

return_type Joint::set_state(const std::vector<double> & state)
{
  return impl_->set_state(state);
}

}  // namespace components
}  // namespace hardware_interface
