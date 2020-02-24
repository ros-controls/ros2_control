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

#include "controller_interface/controller_interface.hpp"

#include <memory>
#include <string>

#include "yaml-cpp/yaml.h"

namespace controller_interface
{

controller_interface_ret_t
ControllerInterface::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  robot_hardware_ = robot_hardware;
  lifecycle_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(controller_name);

  lifecycle_node_->register_on_configure(
    std::bind(&ControllerInterface::on_configure, this, std::placeholders::_1));

  lifecycle_node_->register_on_cleanup(
    std::bind(&ControllerInterface::on_cleanup, this, std::placeholders::_1));

  lifecycle_node_->register_on_activate(
    std::bind(&ControllerInterface::on_activate, this, std::placeholders::_1));

  lifecycle_node_->register_on_deactivate(
    std::bind(&ControllerInterface::on_deactivate, this, std::placeholders::_1));

  lifecycle_node_->register_on_shutdown(
    std::bind(&ControllerInterface::on_shutdown, this, std::placeholders::_1));

  lifecycle_node_->register_on_error(
    std::bind(&ControllerInterface::on_error, this, std::placeholders::_1));

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
ControllerInterface::get_lifecycle_node()
{
  return lifecycle_node_;
}

void ControllerInterface::load_params_from_yaml(const std::string & yaml_config_file)
{
  if (yaml_config_file.empty()) {
    throw std::runtime_error("yaml config file path is empty");
  }

  YAML::Node root_node = YAML::LoadFile(yaml_config_file);
  load_params_from_yaml_node(root_node);
}

void ControllerInterface::load_params_from_yaml_node(YAML::Node& yaml_node)
{
  std::function<void(YAML::Node, const std::string&,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode>&)>
    feed_yaml_to_node_rec =
    [&](YAML::Node yaml_node, const std::string& key,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
  {
    static constexpr char separator = '.';
    if (yaml_node.Type() == YAML::NodeType::Scalar) {
      std::string val_str = yaml_node.as<std::string>();

      // @todo: Do stricter typing for set_parameter value types. 
      // (ie. Numbers should be converted to int/double/etc instead of strings)
      node->declare_parameter(key);
      node->set_parameter({rclcpp::Parameter(key, val_str)});
      return;
    } else if (yaml_node.Type() == YAML::NodeType::Map) {
      for (auto yaml_node_it : yaml_node) {
        feed_yaml_to_node_rec(
          yaml_node_it.second, key + separator + yaml_node_it.first.as<std::string>(), node);
      }
    } else if (yaml_node.Type() == YAML::NodeType::Sequence) {
      size_t index = 0;
      for (auto yaml_node_it : yaml_node) {
        feed_yaml_to_node_rec(yaml_node_it, key + separator + std::to_string((index++)), node);
      }
    }
  };
  feed_yaml_to_node_rec(yaml_node, "", lifecycle_node_);
}

}  // namespace controller_interface
