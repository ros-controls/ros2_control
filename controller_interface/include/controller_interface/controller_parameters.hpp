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

#ifndef CONTROLLER_INTERFACE__CONTROLLER_PARAMETERS_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_PARAMETERS_HPP_

#include <vector>

#include "rclcpp/node.hpp"
#include "rcutils/logging_macros.h"

namespace controller_interface
{

struct Parameter
{
  Parameter() = default;
  
  Parameter(const std::string & name, bool configurable = false) :
  name(name), configurable(configurable)
  {}
  
  std::string name;
  bool configurable;
};
    
class ControllerParameters
{
public:
  ControllerParameters(
    int nr_bool_params = 0, int nr_double_params = 0, 
    int nr_string_params = 0, int nr_string_list_params = 0);
  
  virtual ~ControllerParameters() = default;
  
  virtual void declare_parameters(rclcpp::Node::SharedPtr node);
  
  /**
   * Gets all defined parameters from.
   * 
   * \param[node] shared pointer to the node where parameters should be read.
   * \return true if all parameters are read Successfully, false if a parameter is not provided or 
   * parameter configuration is wrong.
   */
  virtual bool get_parameters(rclcpp::Node::SharedPtr node, bool check_validity = true);
  
  virtual
  rcl_interfaces::msg::SetParametersResult
  set_parameter_callback(const std::vector<rclcpp::Parameter> & parameters);
  
  /**
   * Default implemenation of parameter check. No check is done. Always returns true.
   *
   * \return true
   */
  virtual bool check_if_parameters_are_valid() {
    return true;
  }

  virtual void update() = 0;
  
  // TODO(destogl): Test this: "const rclcpp::Node::SharedPtr & node"
  
protected:
  void add_bool_parameter(
    const std::string & name, bool configurable = false, bool default_value = false)
  {
    bool_parameters_.push_back({Parameter(name, configurable), default_value});
  }
  
  void add_double_parameter(
    const std::string & name, bool configurable = false, 
    double default_value = std::numeric_limits<double>::quiet_NaN())
  {
    double_parameters_.push_back({Parameter(name, configurable), default_value});
  }
  
  void add_string_parameter(
    const std::string & name, bool configurable = false, 
    const std::string & default_value = "")
  {
    string_parameters_.push_back({Parameter(name, configurable), default_value});
  }
  
  void add_string_list_parameter(
    const std::string & name, bool configurable = false, 
    const std::vector<std::string> & default_value = {})
  {
    string_list_parameters_.push_back({Parameter(name, configurable), default_value});
  }
  
  template<typename ParameterT>
  bool get_parameters_from_list(
    const rclcpp::Node::SharedPtr node, std::vector<std::pair<Parameter, ParameterT>> & parameters)
  {
    bool ret = true;
    for (auto & parameter : parameters) {
      try {
        rclcpp::Parameter input_parameter;  // TODO(destogl): will this be alocated each time?
        ret = node->get_parameter(parameter.first.name, input_parameter);
        parameter.second = input_parameter.get_value<ParameterT>();
      } catch (rclcpp::ParameterTypeException & e) {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(),
          "'%s' parameter has wrong type", parameter.first.name.c_str());
        ret = false;
        break;
      }
    }
    return ret;
  }
  
  template<typename ParameterT>
  bool empty_parameter_in_list(const std::vector<std::pair<Parameter, ParameterT>> & parameters)
  {
    bool ret = false;
    for (const auto & parameter : parameters) {
      if (parameter.second.empty()) {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(),
          "'%s' parameter is empty", parameter.first.name.c_str());
        ret = true;
      }
    }
    return ret;
  }
  
  bool empty_parameter_in_list(const std::vector<std::pair<Parameter, double>> & parameters)
  {
    bool ret = false;
    for (const auto & parameter : parameters) {
      if (std::isnan(parameter.second)) {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(),
          "'%s' parameter is not set", parameter.first.name.c_str());
        ret = true;
      }
    }
    return ret;
  }
  
  template<typename ParameterT>
  bool
  find_and_assign_parameter_value(
    std::vector<std::pair<Parameter, ParameterT>> & parameter_list, 
    const rclcpp::Parameter & input_parameter) 
  {
    auto is_in_list = [&](const auto & parameter) {
        return parameter.first.name == input_parameter.get_name();
    };
    
    auto it = std::find_if(
      parameter_list.begin(), parameter_list.end(), is_in_list);
    
    if (it != parameter_list.end()) {
      if (!it->first.configurable) {
        throw std::runtime_error("Parameter " + input_parameter.get_name() +
                                 " is not configurable.");
      } else {
        it->second = input_parameter.get_value<ParameterT>();
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(),
          "'%s' parameter is updated to value: %s", it->first.name.c_str(),
                                input_parameter.value_to_string().c_str());
        return true;
      }
    } else {
      return false;
    }
  }
  
  std::vector<std::pair<Parameter, bool>> bool_parameters_;
  std::vector<std::pair<Parameter, double>> double_parameters_;
  std::vector<std::pair<Parameter, std::string>> string_parameters_;
  std::vector<std::pair<Parameter, std::vector<std::string>>> string_list_parameters_;

  std::string logger_name_;
  
private:
  template<typename ParameterT>
  void declare_parameters_from_list(
    rclcpp::Node::SharedPtr node, const std::vector<std::pair<Parameter, ParameterT>> & parameters)
  {
    for (const auto & parameter : parameters) {
      node->declare_parameter<ParameterT>(parameter.first.name, parameter.second);
    }
  }
};


}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_PARAMETERS_HPP_
