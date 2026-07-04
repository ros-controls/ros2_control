// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CONTROLLER_INTERFACE_HPP_
#define TEST_CONTROLLER_INTERFACE_HPP_

#include "controller_interface/controller_interface.hpp"

constexpr char TEST_CONTROLLER_NAME[] = "testable_controller_interface";

class TestableControllerInterface : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }
};

class TestableControllerInterfaceInitError : public TestableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::ERROR;
  }
};

class TestableControllerInterfaceInitFailure : public TestableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::FAILURE;
  }
};

#endif  // TEST_CONTROLLER_INTERFACE_HPP_
