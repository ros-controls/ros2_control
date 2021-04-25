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

#ifndef TEST_SEMANTIC_COMPONENT_INTERFACE_HPP_
#define TEST_SEMANTIC_COMPONENT_INTERFACE_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "semantic_components/semantic_component_interface.hpp"

// subclassing and friending so we can access member varibles
class FriendSemanticComponentInterface : public semantic_components::
  SemanticComponentInterface
{
public:
  explicit FriendSemanticComponentInterface(const std::string & name, size_t size)
  : SemanticComponentInterface(name, size)
  {
    // generate the interface_names_
    for (size_t i = 1; i <= size; ++i) {
      interface_names_.emplace_back(name_ + "_interface_" + std::to_string(i));
    }
  }
  FRIEND_TEST(SemanticComponentInterfaceTest, Validate_Semantic_Component);
};

class SemanticComponentInterfaceTest : public ::testing::Test
{
public:
  void TearDown();

protected:
  const std::string component_name_ = "test_component";
  const size_t size_ = 5;
  std::unique_ptr<FriendSemanticComponentInterface> semantic_component_;
};

#endif  // TEST_SEMANTIC_COMPONENT_INTERFACE_HPP_
