// Copyright 2021 ros2_control development team
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

#include "test_controller_with_options.hpp"

#include <gtest/gtest.h>
#include <string>
#include "ament_index_cpp/get_package_prefix.hpp"

class FriendControllerWithOptions : public controller_with_options::ControllerWithOptions
{
  FRIEND_TEST(ControllerWithOption, init_with_overrides);
  FRIEND_TEST(ControllerWithOption, init_without_overrides);
};

template <class T, size_t N>
constexpr size_t arrlen(T (&)[N])
{
  return N;
}

TEST(ControllerWithOption, init_with_overrides)
{
  // mocks the declaration of overrides parameters in a yaml file
  char const * const argv[] = {"",   "--ros-args",
                               "-p", "parameter_list.parameter1:=1.",
                               "-p", "parameter_list.parameter2:=2.",
                               "-p", "parameter_list.parameter3:=3."};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  EXPECT_EQ(
    controller.init("controller_name", "", 50.0, "", controller.define_custom_node_options()),
    controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  EXPECT_TRUE(node_options.arguments().empty());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 1.);
  EXPECT_EQ(controller.params["parameter2"], 2.);
  EXPECT_EQ(controller.params["parameter3"], 3.);
  rclcpp::shutdown();
}

TEST(ControllerWithOption, init_with_node_options_arguments_parameters)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  auto controller_node_options = controller.define_custom_node_options();
  controller_node_options.arguments(
    {"--ros-args", "-p", "parameter_list.parameter1:=1.", "-p", "parameter_list.parameter2:=2.",
     "-p", "parameter_list.parameter3:=3."});
  EXPECT_EQ(
    controller.init("controller_name", "", 50.0, "", controller_node_options),
    controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  EXPECT_EQ(7lu, node_options.arguments().size());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 1.);
  EXPECT_EQ(controller.params["parameter2"], 2.);
  EXPECT_EQ(controller.params["parameter3"], 3.);
  rclcpp::shutdown();
}

TEST(ControllerWithOption, init_with_node_options_arguments_parameters_file)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  const std::string params_file_path = ament_index_cpp::get_package_prefix("controller_interface") +
                                       "/test/test_controller_node_options.yaml";
  std::cerr << params_file_path << std::endl;
  auto controller_node_options = controller.define_custom_node_options();
  controller_node_options.arguments({"--ros-args", "--params-file", params_file_path});
  EXPECT_EQ(
    controller.init("controller_name", "", 50.0, "", controller_node_options),
    controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  EXPECT_EQ(3lu, node_options.arguments().size());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 20.0);
  EXPECT_EQ(controller.params["parameter2"], 23.14);
  EXPECT_EQ(controller.params["parameter3"], -52.323);
  bool use_sim_time(true);
  controller.get_node()->get_parameter_or("use_sim_time", use_sim_time, false);
  ASSERT_FALSE(use_sim_time);
  rclcpp::shutdown();
}

TEST(
  ControllerWithOption, init_with_node_options_arguments_parameters_file_and_override_command_line)
{
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  const std::string params_file_path = ament_index_cpp::get_package_prefix("controller_interface") +
                                       "/test/test_controller_node_options.yaml";
  std::cerr << params_file_path << std::endl;
  auto controller_node_options = controller.define_custom_node_options();
  controller_node_options.arguments(
    {"--ros-args", "--params-file", params_file_path, "-p", "parameter_list.parameter1:=562.785",
     "-p", "use_sim_time:=true"});
  EXPECT_EQ(
    controller.init("controller_name", "", 50.0, "", controller_node_options),
    controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  EXPECT_EQ(7lu, node_options.arguments().size());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 562.785);
  EXPECT_EQ(controller.params["parameter2"], 23.14);
  EXPECT_EQ(controller.params["parameter3"], -52.323);
  bool use_sim_time(false);
  controller.get_node()->get_parameter_or("use_sim_time", use_sim_time, false);
  ASSERT_TRUE(use_sim_time);
  rclcpp::shutdown();
}

TEST(ControllerWithOption, init_without_overrides)
{
  // mocks the declaration of overrides parameters in a yaml file
  char const * const argv[] = {""};
  int argc = arrlen(argv);
  rclcpp::init(argc, argv);
  // creates the controller
  FriendControllerWithOptions controller;
  EXPECT_EQ(
    controller.init("controller_name", "", 50.0, "", controller.define_custom_node_options()),
    controller_interface::return_type::ERROR);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  // checks that no parameter has been declared from overrides
  EXPECT_EQ(controller.params.size(), 0u);
  rclcpp::shutdown();
}
