// Copyright 2020 PAL Robotics SL
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

#ifndef CONTROLLER_MANAGER__CONTROLLER_LOADER_PLUGINLIB_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_LOADER_PLUGINLIB_HPP_

#include "controller_manager/controller_loader_interface.hpp"

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"

namespace controller_manager
{

class ControllerLoaderPluginlib : public ControllerLoaderInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  ControllerLoaderPluginlib();

  CONTROLLER_MANAGER_PUBLIC
  virtual ~ControllerLoaderPluginlib() = default;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceSharedPtr create(const std::string & controller_type);

  CONTROLLER_MANAGER_PUBLIC
  bool is_available(const std::string & controller_type) const;

private:
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_LOADER_PLUGINLIB_HPP_
