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

#include "rclcpp/rclcpp.hpp"

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
  EXPECT_EQ(controller.init("controller_name"), controller_interface::return_type::OK);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  // checks that the parameters have been correctly processed
  EXPECT_EQ(controller.params.size(), 3u);
  EXPECT_EQ(controller.params["parameter1"], 1.);
  EXPECT_EQ(controller.params["parameter2"], 2.);
  EXPECT_EQ(controller.params["parameter3"], 3.);
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
  EXPECT_EQ(controller.init("controller_name"), controller_interface::return_type::ERROR);
  // checks that the node options have been updated
  const auto & node_options = controller.get_node()->get_node_options();
  EXPECT_TRUE(node_options.allow_undeclared_parameters());
  EXPECT_TRUE(node_options.automatically_declare_parameters_from_overrides());
  // checks that no parameter has been declared from overrides
  EXPECT_EQ(controller.params.size(), 0u);
  rclcpp::shutdown();
}
