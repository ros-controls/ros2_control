// Copyright 2024 Sherpa Mobile Robotics
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

/*
 * Authors: Thibault Poignonec
 */

#include "test_semantic_component_command_interface.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

void SemanticCommandInterfaceTest::TearDown() { semantic_component_.reset(nullptr); }

TEST_F(SemanticCommandInterfaceTest, validate_command_interfaces)
{
  // create 'test_component' with 3 interfaces using default naming
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ = std::make_unique<TestableSemanticCommandInterface>(component_name_);

  // generate the interface_names_
  std::vector<std::string> interface_names = semantic_component_->get_command_interface_names();

  // validate assign_loaned_command_interfaces
  // create interfaces and assign values to it
  std::vector<double> interface_values = {
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  hardware_interface::CommandInterface cmd_interface_1{component_name_, "1", &interface_values[0]};
  hardware_interface::CommandInterface cmd_interface_2{component_name_, "2", &interface_values[1]};
  hardware_interface::CommandInterface cmd_interface_3{component_name_, "3", &interface_values[2]};

  // create local command interface vector
  std::vector<hardware_interface::LoanedCommandInterface> temp_command_interfaces;
  temp_command_interfaces.reserve(3);
  // insert the interfaces in jumbled sequence
  temp_command_interfaces.emplace_back(cmd_interface_1);
  temp_command_interfaces.emplace_back(cmd_interface_3);
  temp_command_interfaces.emplace_back(cmd_interface_2);

  // now call the function to make them in order like interface_names
  EXPECT_TRUE(semantic_component_->assign_loaned_command_interfaces(temp_command_interfaces));

  // validate the count of command_interfaces_
  ASSERT_EQ(semantic_component_->command_interfaces_.size(), 3u);

  // Validate correct assignment
  const std::vector<double> test_cmd_values = {0.1, 0.2, 0.3};
  EXPECT_TRUE(semantic_component_->set_values(test_cmd_values));

  EXPECT_EQ(interface_values[0], test_cmd_values[0]);
  EXPECT_EQ(interface_values[1], test_cmd_values[1]);
  EXPECT_EQ(interface_values[2], test_cmd_values[2]);

  // release the state_interfaces_
  semantic_component_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(semantic_component_->command_interfaces_.size(), 0u);

  // validate that release_interfaces() does not touch interface_names_
  ASSERT_TRUE(std::equal(
    semantic_component_->interface_names_.begin(), semantic_component_->interface_names_.end(),
    interface_names.begin(), interface_names.end()));
}
