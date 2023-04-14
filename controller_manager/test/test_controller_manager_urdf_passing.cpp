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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"

class TestControllerManagerWithTestableCM;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerManagerWithTestableCM;

  FRIEND_TEST(TestControllerManagerWithTestableCM, callback_gets_passed);
  FRIEND_TEST(TestControllerManagerWithTestableCM, initial_failing);

public:
  TestableControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "")
  : controller_manager::ControllerManager(
      std::move(resource_manager), executor, manager_node_name, namespace_)
  {
  }
};

class TestControllerManagerWithTestableCM
: public ControllerManagerFixture<TestableControllerManager>,
  public testing::WithParamInterface<Strictness>
{
};

// only exemplary to test if working not a useful test yet
TEST_P(TestControllerManagerWithTestableCM, callback_gets_passed)
{
  ASSERT_FALSE(cm_->resource_manager_->load_urdf_called());
}

TEST_P(TestControllerManagerWithTestableCM, initial_failing)
{
  ASSERT_TRUE(cm_->resource_manager_->load_urdf_called());
}

INSTANTIATE_TEST_SUITE_P(
  test_best_effort, TestControllerManagerWithTestableCM, testing::Values(best_effort));
