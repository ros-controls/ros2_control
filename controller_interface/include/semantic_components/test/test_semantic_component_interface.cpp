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

/*
 * Authors: Subhas Das, Denis Stogl
 */

#include "test_semantic_component_interface.hpp"

#include <memory>
#include <string>
#include <vector>

void SemanticComponentInterfaceTest::TearDown()
{
  semantic_component_.reset(nullptr);
}

TEST_F(SemanticComponentInterfaceTest, Validate_Semantic_Component)
{
  // create 'test_component' with 5 interfaces,
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ = std::make_unique<FriendSemanticComponentInterface>(component_name_, size_);

  // validate the component name
  ASSERT_EQ(semantic_component_->name_, component_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(semantic_component_->interface_names_.size(), size_);
  ASSERT_EQ(semantic_component_->state_interfaces_.capacity(), size_);

  // validate the interface_names_
  std::vector<std::string> temp_interface_names = semantic_component_->get_state_interface_types();
  ASSERT_EQ(temp_interface_names, semantic_component_->interface_names_);

  // validate assign_loaned_state_interfaces
  // // create interfaces and assign values to it
  // std::vector<double> interface_values = {1.1, 3.3, 5.5};

  // // assign 1.1 to interface_1, 3.3 to interface_3 and 5.5 to interface_5
  // hardware_interface::StateInterface interface_1{
  //   component_name_, semantic_component_->interface_names_[0], &interface_values[0]};
  // hardware_interface::StateInterface interface_3{
  //   component_name_, semantic_component_->interface_names_[2], &interface_values[1]};
  // hardware_interface::StateInterface interface_5{
  //   component_name_, semantic_component_->interface_names_[4], &interface_values[2]};

  // // create local state interface vector
  // std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  // // insert the interfaces in jumbled sequence
  // temp_state_interfaces.emplace_back(interface_5);
  // temp_state_interfaces.emplace_back(interface_1);
  // temp_state_interfaces.emplace_back(interface_3);

  // // now call the function to make them in order like interface_names
  // semantic_component_->assign_loaned_state_interfaces(temp_state_interfaces);

  // // validate the values of state_interfaces_
  // std::vector<double> temp_values = semantic_component_->get_values();
  // ASSERT_EQ(temp_values, interface_values);

  // // validate the state_interfaces_ count
  // ASSERT_EQ(semantic_component_->state_interfaces_.size(), size_ - 2);
}
