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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_

#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/hardware/component_hardware.hpp"

#include "ros2_control_core/loaders_pluginlib.hpp"
#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"


namespace ros2_control_core_components
{

template < typename ComponentHardwareType >
class Component
{
public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(Component);

  ROS2_CONTROL_CORE_PUBLIC Component() = default;

  ROS2_CONTROL_CORE_PUBLIC Component(std::string parameters_path, std::string type, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
  {
    configure(parameters_path, type, logging_interface, parameters_interface, services_interface);
  };

  ROS2_CONTROL_CORE_PUBLIC virtual ~Component() = default;

  // This is here because of: https://isocpp.org/wiki/faq/templates#templates-defn-vs-decl
  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(std::string parameters_path, std::string type, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
  {
    parameters_path_ = parameters_path;
    type_ = type;
    logging_interface_ = logging_interface;
    parameters_interface_ = parameters_interface;
    services_interface_ = services_interface;

    parameters_interface_->declare_parameter(parameters_path_ + ".name");
    name_ = parameters_interface_->get_parameter(parameters_path_ + ".name").as_string();

    parameters_interface_->declare_parameter(parameters_path_ + ".has_hardware");
    has_hardware_ = parameters_interface_->get_parameter(parameters_path_ + ".has_hardware").as_bool();

    return ros2_control_types::ROS2C_RETURN_OK;
  };

//   ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init(ComponentDescriptionType description_in);

  // TODO: Remove if not used...
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type init(std::string name, ros2_control_types::HardwareDescription hardware_description);

//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type recover() = 0;

//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type stop() = 0;

protected:
  /**
   * @brief Components parameter prefix.
   *
   */
  std::string parameters_path_;
  std::string type_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface_;

  std::string name_;
  bool has_hardware_;

  uint n_dof_;
//   ros2_control_types::component_state_type state_ = 0;

  std::shared_ptr<ComponentHardwareType> hardware_;


  template<typename T>
  std::shared_ptr<T> load_component_from_parameter(std::string parameter_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, ros2_control_core::ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
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
    }
    else
    {
      RCLCPP_WARN(logger, "Robot %s class is _not_ available.", class_name.c_str());
    }
    return component;
  };

  template<typename T>
  std::map<std::string, std::shared_ptr<T>> loadSubComponents(std::string parameters_prefix, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, std::vector<std::string> name_list, ros2_control_core::ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
  {
    std::map<std::string, std::shared_ptr<T>> loaded_components;
    for (auto name: name_list)
    {
      loaded_components[name] = load_component_from_parameter<T>(parameters_prefix + "." + name + ".type", parameters_interface, class_loader, logger);
    }
    return loaded_components;
  };

  template<typename T>
  void load_hardware(ros2_control_core::ROS2ControlLoaderPluginlib<T> class_loader)
  {
    if (has_hardware_)
    {
      hardware_ = load_component_from_parameter(parameters_path_ + "." + type_ + "Hardware.type", parameters_interface_, class_loader, logging_interface_->get_logger());
    }
  };

};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_
