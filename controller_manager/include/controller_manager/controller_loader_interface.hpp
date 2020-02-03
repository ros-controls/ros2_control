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

#ifndef CONTROLLER_MANAGER__CONTROLLER_LOADER_INTERFACE_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_LOADER_INTERFACE_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/visibility_control.h"

namespace controller_manager
{

class ControllerLoaderInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  ControllerLoaderInterface() = default;

  CONTROLLER_MANAGER_PUBLIC
  virtual ~ControllerLoaderInterface() = default;

  CONTROLLER_MANAGER_PUBLIC
  virtual controller_interface::ControllerInterfaceSharedPtr create(
    const std::string & controller_type) = 0;

  CONTROLLER_MANAGER_PUBLIC
  virtual bool is_available(const std::string & controller_type) const = 0;
};

using ControllerLoaderInterfaceSharedPtr = std::shared_ptr<ControllerLoaderInterface>;

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_LOADER_INTERFACE_HPP_
