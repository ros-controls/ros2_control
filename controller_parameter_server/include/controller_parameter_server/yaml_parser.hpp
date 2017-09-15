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

#ifndef CONTROLLER_PARAMETER_SERVER__YAML_PARSER_HPP_
#define CONTROLLER_PARAMETER_SERVER__YAML_PARSER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "controller_parameter_server/visibility_control.h"

#include "yaml-cpp/yaml.h"

namespace controller_parameter_server
{

class YamlParser
{
public:
  CONTROLLER_PARAMETER_SERVER_PUBLIC
  YamlParser() = default;

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  virtual
  ~YamlParser() = default;

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  void
  parse(const std::string & absolute_file_path);

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  std::unordered_map<std::string, std::string>
  get_key_value_pairs();

private:
  YAML::Node root_node_;
};

}  // namespace controller_parameter_server

#endif  // CONTROLLER_PARAMETER_SERVER__YAML_PARSER_HPP_
