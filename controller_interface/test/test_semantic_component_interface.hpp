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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "geometry_msgs/msg/wrench.hpp"
#include "semantic_components/semantic_component_interface.hpp"

// implementing and friending so we can access member variables
class TestableSemanticComponentInterface
: public semantic_components::SemanticComponentInterface<geometry_msgs::msg::Wrench>
{
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_default_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_custom_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_state_interfaces);

public:
  // Use generation of interface names
  explicit TestableSemanticComponentInterface(const std::string & name, size_t size)
  : SemanticComponentInterface<geometry_msgs::msg::Wrench>(name, size)
  {
  }
  // Use custom interface names
  explicit TestableSemanticComponentInterface(size_t size)
  : SemanticComponentInterface("TestSemanticComponent", size)
  {
    // generate the interface_names_
    for (auto i = 0u; i < size; ++i)
    {
      interface_names_.emplace_back(
        std::string("TestSemanticComponent") + "/i" + std::to_string(i + 5));
    }
  }

  virtual ~TestableSemanticComponentInterface() = default;

  std::string test_name_ = "TestSemanticComponent";
};

class SemanticComponentInterfaceTest : public ::testing::Test
{
public:
  void TearDown();

protected:
  const std::string component_name_ = "test_component";
  const size_t size_ = 5;
  std::unique_ptr<TestableSemanticComponentInterface> semantic_component_;
};

#endif  // TEST_SEMANTIC_COMPONENT_INTERFACE_HPP_
