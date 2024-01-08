// Copyright 2023 ros2_control Developers
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

// Authors: Dr. Denis

#ifndef TEST_RESOURCE_MANAGER_HPP_
#define TEST_RESOURCE_MANAGER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

class ResourceManagerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp() {}
};

// Forward declaration
namespace hardware_interface
{
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend ResourceManagerTest;

  FRIEND_TEST(ResourceManagerTest, initialization_with_urdf_manual_validation);
  FRIEND_TEST(ResourceManagerTest, post_initialization_add_components);
  FRIEND_TEST(ResourceManagerTest, managing_controllers_reference_interfaces);
  FRIEND_TEST(ResourceManagerTest, resource_availability_and_claiming_in_lifecycle);
  FRIEND_TEST(ResourceManagerTest, test_unitilizable_hardware_no_validation);

  TestableResourceManager() : hardware_interface::ResourceManager() {}

  TestableResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
};

std::vector<hardware_interface::return_type> set_components_state(
  TestableResourceManager & rm, const std::vector<std::string> & components, const uint8_t state_id,
  const std::string & state_name)
{
  auto int_components = components;
  if (int_components.empty())
  {
    int_components = {"TestActuatorHardware", "TestSensorHardware", "TestSystemHardware"};
  }
  std::vector<hardware_interface::return_type> results;
  for (const auto & component : int_components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    const auto result = rm.set_component_state(component, state);
    results.push_back(result);
  }
  return results;
}
#endif  // TEST_RESOURCE_MANAGER_HPP_
