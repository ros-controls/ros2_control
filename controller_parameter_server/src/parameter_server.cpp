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

ParameterServer::ParameterServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_parameter_server", options)
{}

void
ParameterServer::load_parameters(const std::string & yaml_config_file)
{
  if (yaml_config_file.empty()) {
    throw std::runtime_error("yaml config file path is empty");
  }

  YamlParser parser;
  parser.parse(yaml_config_file);

  auto key_values = parser.get_key_value_pairs();
  for (auto pair : key_values) {
    this->set_parameters({rclcpp::Parameter(pair.first, pair.second)});
  }
}

void
ParameterServer::load_parameters(const std::string & key, const std::string & value)
{
  this->set_parameters({rclcpp::Parameter(key, value)});
}

}  // namespace controller_parameter_server
