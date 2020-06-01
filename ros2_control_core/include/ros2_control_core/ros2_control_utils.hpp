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

#ifndef ROS2_CONTROL_CORE__ROS2_CONTROL_UTILS_H_
#define ROS2_CONTROL_CORE__ROS2_CONTROL_UTILS_H_

#include <map>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"

// TODO: Create util library instead of inline functions?

namespace ros2_control_utils
{
  // Classes
  template < typename ROS2ControlType >
  class ROS2ControlLoaderPluginlib
  {
  public:
    ROS2_CONTROL_CORE_PUBLIC ROS2ControlLoaderPluginlib(const std::string package, const std::string base_type) : loader_(std::make_shared<pluginlib::ClassLoader< ROS2ControlType >>(package, base_type))
    {
    };

    ROS2_CONTROL_CORE_PUBLIC
    virtual ~ROS2ControlLoaderPluginlib() = default;

    // TODO: Add try-catch
    ROS2_CONTROL_CORE_PUBLIC
    std::shared_ptr<ROS2ControlType> create(const std::string & type)
    {
      return std::shared_ptr<ROS2ControlType>(loader_->createUnmanagedInstance(type));
    };

    ROS2_CONTROL_CORE_PUBLIC
    bool is_available(const std::string & type)
    {
      return loader_->isClassAvailable(type);
    };

  private:
    std::shared_ptr<pluginlib::ClassLoader< ROS2ControlType >> loader_;
  };

  // Functions
  template<typename T>
  inline std::shared_ptr<T> load_component_from_parameter(std::string parameter_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
  {
    std::shared_ptr<T> component;
    std::string class_name;
    bool class_available;

    parameters_interface->declare_parameter(parameter_name);
    class_name = parameters_interface->get_parameter(parameter_name).as_string();
    class_available = class_loader.is_available(class_name);
    if (class_available)
    {
      component = class_loader.create(class_name);
      //FIXME:DEBUG
      RCLCPP_INFO(logger, "%s class is loaded.", class_name.c_str());
    }
    else
    {
      RCLCPP_WARN(logger, "%s: %s class is _not_ available.", parameter_name.c_str(), class_name.c_str());
    }
    return component;
  };

  template<typename T>
  inline std::map<std::string, std::shared_ptr<T>> load_components_from_parameters(std::string parameters_prefix, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, std::vector<std::string> name_list, ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
  {
    std::map<std::string, std::shared_ptr<T>> loaded_components;
    for (auto name: name_list)
    {
      loaded_components[name] = load_component_from_parameter<T>(parameters_prefix + "." + name + ".type", parameters_interface, class_loader, logger);
    }
    return loaded_components;
  };

}  // namespace ros2_control_utils

#endif  // ROS2_CONTROL_CORE__ROS2_CONTROL_UTILS_H_
