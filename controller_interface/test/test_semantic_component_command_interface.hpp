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

#ifndef TEST_SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_
#define TEST_SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "semantic_components/semantic_component_command_interface.hpp"

// implementing and friending so we can access member variables
class TestableSemanticCommandInterface
: public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Pose>
{
  FRIEND_TEST(SemanticCommandInterfaceTest, validate_default_names);
  FRIEND_TEST(SemanticCommandInterfaceTest, validate_custom_names);
  FRIEND_TEST(SemanticCommandInterfaceTest, validate_command_interfaces);

public:
  // Use generation of interface names
  explicit TestableSemanticCommandInterface(const std::string & name, size_t size)
  : SemanticComponentCommandInterface<geometry_msgs::msg::Pose>(name, size)
  {
  }
  // Use custom interface names
  explicit TestableSemanticCommandInterface(size_t size)
  : SemanticComponentCommandInterface("TestSemanticCommandInterface", size)
  {
    // generate the interface_names_
    for (auto i = 0u; i < size; ++i)
    {
      interface_names_.emplace_back(
        std::string("TestSemanticCommandInterface") + "/i" + std::to_string(i + 5));
    }
  }

  virtual ~TestableSemanticCommandInterface() = default;

  std::string test_name_ = "TestSemanticCommandInterface";
};

class SemanticCommandInterfaceTest : public ::testing::Test
{
public:
  void TearDown();

protected:
  const std::string component_name_ = "test_component";
  const size_t size_ = 3;
  std::unique_ptr<TestableSemanticCommandInterface> semantic_component_;
};

#endif  // TEST_SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_
