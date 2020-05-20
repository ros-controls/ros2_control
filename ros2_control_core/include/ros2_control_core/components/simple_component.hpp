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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_COMPONENT_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_COMPONENT_HPP_

#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/ros2_control_types.h"

#include "ros2_control_core/components/component.hpp"

#include "control_msgs/msg/interface_value.hpp"


namespace ros2_control_core_components
{

template < typename ComponentDescriptionType, typename ComponentHardwareType >
class SimpleComponent : Component<ComponentDescriptionType, ComponentHardwareType>
{
public:
  ROS2_CONTROL_CORE_PUBLIC SimpleComponent() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~SimpleComponent() = default;

protected:
  control_msgs::msg::InterfaceValue values;
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_COMPONENT_HPP_
