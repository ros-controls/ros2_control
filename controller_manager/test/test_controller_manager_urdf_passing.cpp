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

#include "ros2_control_test_assets/descriptions.hpp"

class TestControllerManagerWithTestableCM;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerManagerWithTestableCM;

  FRIEND_TEST(TestControllerManagerWithTestableCM, initial_no_load_urdf_called);
  FRIEND_TEST(TestControllerManagerWithTestableCM, load_urdf_called_after_callback);
  FRIEND_TEST(TestControllerManagerWithTestableCM, load_urdf_called_after_invalid_urdf_passed);
  FRIEND_TEST(TestControllerManagerWithTestableCM, load_urdf_called_after_callback);
  FRIEND_TEST(TestControllerManagerWithTestableCM, load_urdf_called_after_callback);

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
public:
  // create cm with no urdf
  TestControllerManagerWithTestableCM()
  : ControllerManagerFixture<TestableControllerManager>("", false)
  {
  }
};

TEST_P(TestControllerManagerWithTestableCM, initial_no_load_urdf_called)
{
  ASSERT_FALSE(cm_->resource_manager_->is_urdf_already_loaded());
}

TEST_P(TestControllerManagerWithTestableCM, load_urdf_called_after_callback)
{
  ASSERT_FALSE(cm_->resource_manager_->is_urdf_already_loaded());
  // mimic callback
  auto msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_robot_urdf;
  cm_->robot_description_callback(msg);
  ASSERT_TRUE(cm_->resource_manager_->is_urdf_already_loaded());
}

TEST_P(TestControllerManagerWithTestableCM, load_urdf_called_after_invalid_urdf_passed)
{
  ASSERT_FALSE(cm_->resource_manager_->is_urdf_already_loaded());
  // mimic callback
  auto msg = std_msgs::msg::String();
  msg.data = ros2_control_test_assets::minimal_robot_missing_command_keys_urdf;
  cm_->robot_description_callback(msg);
  ASSERT_TRUE(cm_->resource_manager_->is_urdf_already_loaded());
}

INSTANTIATE_TEST_SUITE_P(
  test_best_effort, TestControllerManagerWithTestableCM, testing::Values(best_effort));
