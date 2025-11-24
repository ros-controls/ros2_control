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

#include "test_controller_manager_with_resource_manager.hpp"

std::shared_ptr<rclcpp::Node> ControllerManagerTest::node_ = nullptr;
std::unique_ptr<hardware_interface::ResourceManager> ControllerManagerTest::test_resource_manager_ =
  nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ControllerManagerTest::executor_ =
  nullptr;

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_without_hardware_plugin;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_invalid_urdf)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = R"(<robot malformed></robot>)";

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_empty_urdf)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = "";

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_wrong_plugins)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_with_wrong_plugin;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_no_geometry)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_no_geometry;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, init_controller_manager_with_invalid_urdf)
{
  const std::string invalid_urdf = ros2_control_test_assets::invalid_urdf_with_wrong_plugin;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
