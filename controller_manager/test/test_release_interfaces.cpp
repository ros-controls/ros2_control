// Copyright 2021 Department of Engineering Cybernetics, NTNU.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller_with_interfaces/test_controller_with_interfaces.hpp"

using ::testing::_;
using ::testing::Return;

class TestReleaseInterfaces : public ControllerManagerFixture<controller_manager::ControllerManager>
{
};

TEST_F(TestReleaseInterfaces, switch_controllers_same_interface)
{
  std::string controller_type =
    test_controller_with_interfaces::TEST_CONTROLLER_WITH_INTERFACES_CLASS_NAME;

  // Load two controllers of different names
  std::string controller_name1 = "test_controller1";
  std::string controller_name2 = "test_controller2";
  ASSERT_NO_THROW(cm_->load_controller(controller_name1, controller_type));
  ASSERT_NO_THROW(cm_->load_controller(controller_name2, controller_type));
  ASSERT_EQ(2u, cm_->get_loaded_controllers().size());
  controller_manager::ControllerSpec abstract_test_controller1 = cm_->get_loaded_controllers()[0];
  controller_manager::ControllerSpec abstract_test_controller2 = cm_->get_loaded_controllers()[1];

  // Configure controllers
  ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(controller_name1));
  ASSERT_EQ(controller_interface::return_type::OK, cm_->configure_controller(controller_name2));

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller1.c->get_lifecycle_state().id());
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    abstract_test_controller2.c->get_lifecycle_state().id());

  {  // Test starting the first controller
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #1");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test starting the second controller when the first is running
    // Fails as they have the same command interface
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #2");
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping controller #1 and starting controller #2
    RCLCPP_INFO(cm_->get_logger(), "Stopping controller #1 and starting controller #2");
    std::vector<std::string> start_controllers = {controller_name2};
    std::vector<std::string> stop_controllers = {controller_name1};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping controller #2 and starting controller #1
    RCLCPP_INFO(cm_->get_logger(), "Starting controller #1 and stopping controller #2");
    std::vector<std::string> start_controllers = {controller_name1};
    std::vector<std::string> stop_controllers = {controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping both controllers when only controller #1 is running
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping both controllers (will fail in STRICT mode as controller #2 is not running)");
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1, controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    // The call to switch above will fail and the CM will not block
    // ASSERT_EQ(
    //   std::future_status::timeout,
    //   switch_future.wait_for(std::chrono::milliseconds(100))) <<
    //   "switch_controller should be blocking until next update cycle";
    // ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::ERROR, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test stopping both controllers when only controller #1 is running
    RCLCPP_INFO(
      cm_->get_logger(),
      "Stopping both controllers (will fail in BEST EFFORT mode as controller #2 is not running)");
    std::vector<std::string> start_controllers = {};
    std::vector<std::string> stop_controllers = {controller_name1, controller_name2};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, BEST_EFFORT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }

  {  // Test starting both controllers at the same time
    RCLCPP_INFO(
      cm_->get_logger(),
      "Starting both controllers at the same time (should notify about resource conflict)");
    std::vector<std::string> start_controllers = {controller_name1, controller_name2};
    std::vector<std::string> stop_controllers = {};
    auto switch_future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_,
      start_controllers, stop_controllers, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, switch_future.wait_for(std::chrono::milliseconds(100)))
      << "switch_controller should be blocking until next update cycle";
    ControllerManagerRunner cm_runner(this);
    EXPECT_EQ(controller_interface::return_type::OK, switch_future.get());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      abstract_test_controller1.c->get_lifecycle_state().id());
    ASSERT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      abstract_test_controller2.c->get_lifecycle_state().id());
  }
}

class TestControllerInterfacesRemapping;

class TestableControllerManager : public controller_manager::ControllerManager
{
  friend TestControllerInterfacesRemapping;

  FRIEND_TEST(TestControllerInterfacesRemapping, check_resource_manager_resources);

public:
  TestableControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & node_namespace = "")
  : controller_manager::ControllerManager(
      std::move(resource_manager), executor, manager_node_name, node_namespace)
  {
  }
};

class TestControllerInterfacesRemapping : public ControllerManagerFixture<TestableControllerManager>
{
public:
  TestControllerInterfacesRemapping()
  : ControllerManagerFixture<TestableControllerManager>(
      std::string(ros2_control_test_assets::urdf_head) +
      std::string(ros2_control_test_assets::hardware_resources_with_types_of_effort) +
      std::string(ros2_control_test_assets::urdf_tail))
  {
  }

  void SetUp() override
  {
    ControllerManagerFixture::SetUp();

    update_timer_ = cm_->create_wall_timer(
      std::chrono::milliseconds(10),
      [&]()
      {
        cm_->read(time_, PERIOD);
        cm_->update(time_, PERIOD);
        cm_->write(time_, PERIOD);
      });

    update_executor_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);

    update_executor_->add_node(cm_);
    update_executor_spin_future_ =
      std::async(std::launch::async, [this]() -> void { update_executor_->spin(); });
    // This sleep is needed to prevent a too fast test from ending before the
    // executor has began to spin, which causes it to hang
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override { update_executor_->cancel(); }

protected:
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Using a MultiThreadedExecutor so we can call update on a separate thread from service callbacks
  std::shared_ptr<rclcpp::Executor> update_executor_;
  std::future<void> update_executor_spin_future_;
};

TEST_F(TestControllerInterfacesRemapping, check_resource_manager_resources)
{
  ASSERT_TRUE(cm_->is_resource_manager_initialized());
  ASSERT_TRUE(cm_->resource_manager_->are_components_initialized());
  std::vector<std::string> expected_command_interfaces(
    {"joint1/position", "joint1/max_velocity", "joint1/effort", "joint2/position",
     "joint2/max_velocity", "joint2/torque", "joint3/position", "joint3/max_velocity",
     "joint3/force"});
  std::vector<std::string> expected_state_interfaces(
    {"joint1/position", "joint1/velocity", "joint1/effort", "joint2/position", "joint2/velocity",
     "joint2/torque", "joint3/position", "joint3/velocity", "joint3/force"});

  for (const auto & itf : expected_command_interfaces)
  {
    ASSERT_TRUE(cm_->resource_manager_->command_interface_is_available(itf))
      << "Couldn't find command interface: " << itf;
    ASSERT_FALSE(cm_->resource_manager_->command_interface_is_claimed(itf))
      << "The command interface is not supposed to be claimed by any controller: " << itf;
  }
  for (const auto & itf : expected_state_interfaces)
  {
    ASSERT_TRUE(cm_->resource_manager_->state_interface_is_available(itf))
      << "Couldn't find state interface: " << itf;
  }

  // There is no effort interface for joint2 and joint3
  ASSERT_FALSE(cm_->resource_manager_->command_interface_is_available("joint2/effort"));
  ASSERT_FALSE(cm_->resource_manager_->command_interface_is_available("joint3/effort"));

  const std::string test_file_path = ament_index_cpp::get_package_prefix("controller_manager") +
                                     "/test/test_controller_spawner_with_type.yaml";

  // Provide controller type via the parsed file
  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(
    std::system(std::string(
                  "ros2 run controller_manager spawner controller_joint1 controller_joint2 "
                  "controller_joint3 -c test_controller_manager -p " +
                  test_file_path)
                  .c_str()),
    0);

  // once the controllers are successfully started, check the command interfaces are remapped as
  // expected
  for (const auto & itf : {"joint1/effort", "joint2/torque", "joint3/force"})
  {
    ASSERT_TRUE(cm_->resource_manager_->command_interface_is_available(itf))
      << "The command interface are not supposed to be available: " << itf;
    ASSERT_TRUE(cm_->resource_manager_->command_interface_is_claimed(itf))
      << "The command interface is supposed to be claimed by the controller : " << itf;
  }

  EXPECT_EQ(
    std::system(
      std::string(
        "ros2 run controller_manager unspawner controller_joint1 -c test_controller_manager")
        .c_str()),
    0);
  ASSERT_FALSE(cm_->resource_manager_->command_interface_is_claimed("joint1/effort"));

  // Now unspawn the controller_joint2 and controller_joint3 and see if the respective interfaces
  // are released
  EXPECT_EQ(
    std::system(
      std::string(
        "ros2 run controller_manager unspawner controller_joint2 -c test_controller_manager")
        .c_str()),
    0);
  ASSERT_FALSE(cm_->resource_manager_->command_interface_is_claimed("joint2/torque"));

  EXPECT_EQ(
    std::system(
      std::string(
        "ros2 run controller_manager unspawner controller_joint3 -c test_controller_manager")
        .c_str()),
    0);
  ASSERT_FALSE(cm_->resource_manager_->command_interface_is_claimed("joint3/force"));
}
