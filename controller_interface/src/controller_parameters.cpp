// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
//
/// \author: Denis Stogl

#include "controller_interface/controller_parameters.hpp"

#include <string>
#include <vector>

#include "rcutils/logging_macros.h"

namespace controller_interface
{
ControllerParameters::ControllerParameters(
  int nr_bool_params, int nr_double_params, int nr_string_params, int nr_string_list_params)
{
  bool_parameters_.reserve(nr_bool_params);
  double_parameters_.reserve(nr_double_params);
  string_parameters_.reserve(nr_string_params);
  string_list_parameters_.reserve(nr_string_list_params);
}

void ControllerParameters::declare_parameters(rclcpp::Node::SharedPtr node)
{
  logger_name_ = std::string(node->get_name()) + "::parameters";

  declare_parameters_from_list(node, bool_parameters_);
  declare_parameters_from_list(node, double_parameters_);
  declare_parameters_from_list(node, string_parameters_);
  declare_parameters_from_list(node, string_list_parameters_);
}

/**
  * Gets all defined parameters from.
  *
  * \param[node] shared pointer to the node where parameters should be read.
  * \return true if all parameters are read Successfully, false if a parameter is not provided or
  * parameter configuration is wrong.
  */
bool ControllerParameters::get_parameters(
  rclcpp::Node::SharedPtr node, bool check_validity, bool update)
{
  bool ret = false;

  ret = get_parameters_from_list(node, bool_parameters_) &&
        get_parameters_from_list(node, double_parameters_) &&
        get_parameters_from_list(node, string_parameters_) &&
        get_parameters_from_list(node, string_list_parameters_);

  if (ret && check_validity)
  {
    ret = check_if_parameters_are_valid();
  }

  if (ret && update)
  {
    this->update();
  }

  return ret;
}

rcl_interfaces::msg::SetParametersResult ControllerParameters::set_parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & input_parameter : parameters)
  {
    bool found = false;

    try
    {
      found = find_and_assign_parameter_value(bool_parameters_, input_parameter);
      if (!found)
      {
        found = find_and_assign_parameter_value(double_parameters_, input_parameter);
      }
      if (!found)
      {
        found = find_and_assign_parameter_value(string_parameters_, input_parameter);
      }
      if (!found)
      {
        found = find_and_assign_parameter_value(string_list_parameters_, input_parameter);
      }

      RCUTILS_LOG_INFO_EXPRESSION_NAMED(
        found, logger_name_.c_str(),
        "Dynamic parameters got changed! To update the parameters internally please "
        "restart the controller.");
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      result.successful = false;
      result.reason = e.what();
      RCUTILS_LOG_ERROR_NAMED(logger_name_.c_str(), "%s", result.reason.c_str());
      break;
    }
    catch (const std::runtime_error & e)
    {
      result.successful = false;
      result.reason = e.what();
      RCUTILS_LOG_ERROR_NAMED(logger_name_.c_str(), "%s", result.reason.c_str());
      break;
    }
  }

  return result;
}

}  // namespace controller_interface
