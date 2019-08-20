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

#ifndef CONTROLLER_PARAMETER_SERVER__PARAMETER_SERVER_HPP_
#define CONTROLLER_PARAMETER_SERVER__PARAMETER_SERVER_HPP_

#include <memory>
#include <string>

#include "controller_parameter_server/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

namespace controller_parameter_server
{

class ParameterServer : public rclcpp::Node
{
public:
  CONTROLLER_PARAMETER_SERVER_PUBLIC
  explicit ParameterServer(
    const rclcpp::NodeOptions & options = (
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)));

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  virtual
  ~ParameterServer() = default;

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  void
  load_parameters(const std::string & yaml_config_file);

  CONTROLLER_PARAMETER_SERVER_PUBLIC
  void
  load_parameters(const std::string & key, const std::string & value);
};

}  // namespace controller_parameter_server

#endif  // CONTROLLER_PARAMETER_SERVER__PARAMETER_SERVER_HPP_
