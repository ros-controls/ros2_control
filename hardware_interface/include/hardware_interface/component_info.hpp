// Copyright 2020 ROS2-Control Development Team
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


#ifndef HARDWARE_INTERFACE__COMPONENT_INFO_HPP_
#define HARDWARE_INTERFACE__COMPONENT_INFO_HPP_

#include <unordered_map>
#include <string>
#include <vector>

namespace hardware_interface
{

struct ComponentInfo
{
  std::string name;
  std::string type;
  std::string class_type;
  std::string joint;
  std::vector<std::string> interface_names;
  std::unordered_map<std::string, std::string> parameters;
  std::vector<ComponentInfo> subcomponents;

  std::string hardware_class_type;
  std::unordered_map<std::string, std::string> hardware_parameters;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__COMPONENT_INFO_HPP_
