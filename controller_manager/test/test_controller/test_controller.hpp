// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef TEST_CONTROLLER__TEST_CONTROLLER_HPP_
#define TEST_CONTROLLER__TEST_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "controller_interface/visibility_control.h"

namespace test_controller
{

constexpr char TEST_CONTROLLER_NAME[] = "test_controller_name";
constexpr char TEST_CONTROLLER_TYPE[] = "test_controller";
class TestController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestController();

  CONTROLLER_MANAGER_PUBLIC
  virtual
  ~TestController() = default;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  update() override;

  CONTROLLER_MANAGER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  size_t internal_counter = 0;
};

}  // namespace test_controller

#endif  // TEST_CONTROLLER__TEST_CONTROLLER_HPP_
