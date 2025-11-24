// Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_
#define TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_

#include <memory>
#include <string>

#include <utility>
#include "controller_manager/controller_manager.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "std_msgs/msg/string.hpp"

class TestControllerManager : public controller_manager::ControllerManager
{
public:
  using ControllerManager::ControllerManager;

  // Expose callbacks
  using ControllerManager::robot_description_callback;

  using ControllerManager::is_resource_manager_initialized;

  using ControllerManager::resource_manager_;

  using ControllerManager::has_valid_robot_description;
};

class ControllerManagerTest : public ::testing::Test
{
protected:
  static std::shared_ptr<rclcpp::Node> node_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  static std::unique_ptr<hardware_interface::ResourceManager> test_resource_manager_;
  virtual void SetUp();
  virtual void TearDown();
};

#endif  // TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_
