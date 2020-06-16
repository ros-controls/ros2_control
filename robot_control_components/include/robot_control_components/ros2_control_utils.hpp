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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_UTILS_H_
#define ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_UTILS_H_

#include <map>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_components/visibility_control.h"

// TODO: Create util library instead of inline functions?

namespace ros2_control_utils
{
// Classes
template < typename T >
class ROS2ControlLoaderPluginlib
{
public:
  ROBOT_CONTROL_COMPONENTS_PUBLIC ROS2ControlLoaderPluginlib(const std::string package, const std::string base_type) : loader_(std::make_shared<pluginlib::ClassLoader< T >>(package, base_type))
  {
  };

  ROBOT_CONTROL_COMPONENTS_PUBLIC
  virtual ~ROS2ControlLoaderPluginlib() = default;

  // TODO: Add try-catch
  ROBOT_CONTROL_COMPONENTS_PUBLIC
  std::shared_ptr<T> create(const std::string & type)
  {
    return std::shared_ptr<T>(loader_->createUnmanagedInstance(type));
  };

  ROBOT_CONTROL_COMPONENTS_PUBLIC
  bool is_available(const std::string & type)
  {
    return loader_->isClassAvailable(type);
  };

private:
  std::shared_ptr<pluginlib::ClassLoader< T >> loader_;
};



}  // namespace ros2_control_utils

#endif  // ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_UTILS_H_
