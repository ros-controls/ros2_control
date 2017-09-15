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

#include <memory>
#include <string>

#include "controller_parameter_server/parameter_server.hpp"
#include "controller_parameter_server/yaml_parser.hpp"

namespace controller_parameter_server
{

ParameterServer::ParameterServer()
: rclcpp::node::Node("controller_parameter_server")
{}

void
ParameterServer::init()
{
  parameter_service_ =
    std::make_shared<rclcpp::parameter_service::ParameterService>(shared_from_this());
}

void
ParameterServer::load_parameters(const std::string & yaml_config_file)
{
  if (!parameter_service_) {
    throw std::runtime_error("parameter server is not initialized");
  }

  if (yaml_config_file.empty()) {
    throw std::runtime_error("yaml config file path is empty");
  }

  YamlParser parser;
  parser.parse(yaml_config_file);

  auto key_values = parser.get_key_value_pairs();
  for (auto pair : key_values) {
    this->set_parameters({rclcpp::parameter::ParameterVariant(pair.first, pair.second)});
  }
}

void
ParameterServer::load_parameters(const std::string & key, const std::string & value)
{
  if (!parameter_service_) {
    throw std::runtime_error("parameter server is not initialized");
  }

  this->set_parameters({rclcpp::parameter::ParameterVariant(key, value)});
}

}  // namespace controller_parameter_server
