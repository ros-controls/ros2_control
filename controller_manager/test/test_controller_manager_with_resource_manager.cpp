// Copyright 2025 b-robotized group
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

#include "test_controller_manager_with_resource_manager.hpp"

std::shared_ptr<rclcpp::Node> ControllerManagerTest::node_ = nullptr;
std::unique_ptr<hardware_interface::ResourceManager> ControllerManagerTest::test_resource_manager_ =
  nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ControllerManagerTest::executor_ =
  nullptr;

void ControllerManagerTest::SetUp()
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("controller_manager_test_node");
  auto clock = node_->get_clock();
  auto logger = node_->get_logger();

  test_resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(clock, logger);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void ControllerManagerTest::TearDown()
{
  node_.reset();
  test_resource_manager_.reset();
  executor_.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_urdf_without_hardware_plugin)
{
  const std::string invalid_urdf = ros2_control_test_assets::invalid_urdf_without_hardware_plugin;
  std_msgs::msg::String valid_robot_description;
  valid_robot_description.data = ros2_control_test_assets::minimal_robot_urdf;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());
  cm.robot_description_callback(valid_robot_description);
  EXPECT_TRUE(cm.is_resource_manager_initialized());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_invalid_urdf)
{
  const std::string invalid_urdf = R"(<robot malformed></robot>)";
  std_msgs::msg::String valid_robot_description;
  valid_robot_description.data = ros2_control_test_assets::minimal_robot_urdf;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());
  cm.robot_description_callback(valid_robot_description);
  EXPECT_TRUE(cm.is_resource_manager_initialized());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_empty_urdf)
{
  const std::string invalid_urdf = "";
  std_msgs::msg::String valid_robot_description;
  valid_robot_description.data = ros2_control_test_assets::minimal_robot_urdf;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());
  cm.robot_description_callback(valid_robot_description);
  EXPECT_TRUE(cm.is_resource_manager_initialized());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_nonexistent_plugins)
{
  const std::string invalid_urdf = ros2_control_test_assets::invalid_urdf_with_nonexistent_plugin;
  std_msgs::msg::String valid_robot_description;
  valid_robot_description.data = ros2_control_test_assets::minimal_robot_urdf;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());
  cm.robot_description_callback(valid_robot_description);
  EXPECT_TRUE(cm.is_resource_manager_initialized());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_no_geometry)
{
  const std::string invalid_urdf = ros2_control_test_assets::invalid_urdf_no_geometry;
  std_msgs::msg::String valid_robot_description;
  valid_robot_description.data = ros2_control_test_assets::minimal_robot_urdf;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());
  cm.robot_description_callback(valid_robot_description);
  EXPECT_TRUE(cm.is_resource_manager_initialized());
}

TEST_F(ControllerManagerTest, cm_constructor_with_invalid_rm_object)
{
  EXPECT_THROW(
    { TestControllerManager cm(std::move(test_resource_manager_), executor_); },
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
