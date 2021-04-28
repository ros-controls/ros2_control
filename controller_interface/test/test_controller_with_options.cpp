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

#include <gtest/gtest.h>
#include <string>
#include "test_controller_with_options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller_with_options
{
controller_interface::return_type ControllerWithOptions::init(
  const std::string & controller_name)
{
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  auto result = ControllerInterface::init(controller_name, options);
  if (result == controller_interface::return_type::ERROR) {
    return result;
  }
  if (node_->get_parameters("parameter_list", params)) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "I found " << params.size() << " parameters.");
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}
controller_interface::InterfaceConfiguration
ControllerWithOptions::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}
controller_interface::InterfaceConfiguration
ControllerWithOptions::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}
controller_interface::return_type ControllerWithOptions::update()
{
  return controller_interface::return_type::OK;
}
}  // namespace controller_with_options

class FriendControllerWithOptions : public controller_with_options::ControllerWithOptions
{
  FRIEND_TEST(ControllerWithOption, init_with_overrides);
  FRIEND_TEST(ControllerWithOption, init_without_overrides);
};

TEST(ControllerWithOption, init_with_overrides) {
  // mocks the declaration of overrides parameters in a yaml file
  int argc = 8;
  char const * const argv[8] = {"", "--ros-args", "-p", "parameter_list.parameter1:=1.", "-p",
    "parameter_list.parameter2:=2.", "-p", "parameter_list.parameter3:=3."};
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  EXPECT_EQ(controller.init("controller_name"), controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.node_->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 1.);
  EXPECT_EQ(controller.params["parameter2"], 2.);
  EXPECT_EQ(controller.params["parameter3"], 3.);
  rclcpp::shutdown();
}

TEST(ControllerWithOption, init_without_overrides) {
  // mocks the declaration of overrides parameters in a yaml file
  int argc = 1;
  char const * const argv[8] = {""};
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  EXPECT_EQ(controller.init("controller_name"), controller_interface::return_type::ERROR);
  // checks that the node options have been updated
  const auto & node_options = controller.node_->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  // checks that no parameter has been declared from overrides
  EXPECT_EQ(controller.params.size(), 0u);
  rclcpp::shutdown();
}
