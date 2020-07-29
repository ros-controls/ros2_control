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


#ifndef HARDWARE_INTERFACE__COMPONENT_INFO_HPP_
#define HARDWARE_INTERFACE__COMPONENT_INFO_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface
{

/**
 * \brief This structure stores information about components defined in a robot's URDF.
 */
struct ComponentInfo
{
  /**
   * \brief name of the component.
   */
  std::string name;
  /**
   * \brief type of the component: sensor or actuator.
   */
  std::string type;
  /**
   * \brief component's class, which will be dynamically loaded.
   */
  std::string class_type;
  /**
   * \brief joint where component is placed.
   */
  std::vector<std::string> joints;
  /**
   * \brief name of the interface, e.g. "position", "velocity", etc. for meaning of data this component holds.
   */
  std::vector<std::string> interface_names;
  /**
   * \brief (optional) key-value pairs of components parameters.
   */
  std::unordered_map<std::string, std::string> parameters;

  /**
   * \brief hardware class of the component that will be dynamically loaded.
   */
  std::string hardware_class_type;
  /**
   * \brief (optional) key-value pairs for components hardware.
   */
  std::unordered_map<std::string, std::string> hardware_parameters;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__COMPONENT_INFO_HPP_
