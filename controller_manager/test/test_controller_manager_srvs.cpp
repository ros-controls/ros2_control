// Copyright 2020 PAL Robotics S.L.
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
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using ::testing::_;
using ::testing::Return;

using namespace std::chrono_literals;

class TestControllerManagerSrvs : public ControllerManagerFixture
{
public:
  TestControllerManagerSrvs() {}

  void SetUp() override
  {
    ControllerManagerFixture::SetUp();

    update_timer_ = cm_->create_wall_timer(std::chrono::milliseconds(10), [&]() {
      cm_->read();
      cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      cm_->write();
    });

    executor_->add_node(cm_);

    executor_spin_future_ = std::async(std::launch::async, [this]() -> void { executor_->spin(); });
    // This sleep is needed to prevent a too fast test from ending before the
    // executor has began to spin, which causes it to hang
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override { executor_->cancel(); }

  template <typename T>
  std::shared_ptr<typename T::Response> call_service_and_wait(
    rclcpp::Client<T> & client, std::shared_ptr<typename T::Request> request,
    rclcpp::Executor & service_executor, bool update_controller_while_spinning = false)
  {
    EXPECT_TRUE(client.wait_for_service(std::chrono::milliseconds(500)));
    auto result = client.async_send_request(request);
    // Wait for the result.
    if (update_controller_while_spinning)
    {
      while (service_executor.spin_until_future_complete(result, 50ms) !=
             rclcpp::FutureReturnCode::SUCCESS)
      {
        cm_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      }
    }
    else
    {
      EXPECT_EQ(
        service_executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);
    }
    return result.get();
  }

protected:
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::future<void> executor_spin_future_;
};

TEST_F(TestControllerManagerSrvs, list_controller_types)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllerTypes>(
      "test_controller_manager/list_controller_types");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllerTypes::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  // Number depends on the controllers that exist on the system
  size_t controller_types = result->types.size();
  ASSERT_GE(controller_types, 1u);
  ASSERT_EQ(controller_types, result->base_classes.size());
  ASSERT_THAT(result->types, ::testing::Contains(test_controller::TEST_CONTROLLER_CLASS_NAME));
  ASSERT_THAT(
    result->base_classes, ::testing::Contains("controller_interface::ControllerInterface"));
}

TEST_F(TestControllerManagerSrvs, list_controllers_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ListControllers>(
      "test_controller_manager/list_controllers");
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(0u, result->controller.size());

  auto test_controller = std::make_shared<test_controller::TestController>();
  controller_interface::InterfaceConfiguration cmd_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint2/velocity"}};
  controller_interface::InterfaceConfiguration state_cfg = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/position", "joint1/velocity", "joint2/position"}};
  test_controller->set_command_interface_configuration(cmd_cfg);
  test_controller->set_state_interface_configuration(state_cfg);
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ(test_controller::TEST_CONTROLLER_NAME, result->controller[0].name);
  ASSERT_EQ(test_controller::TEST_CONTROLLER_CLASS_NAME, result->controller[0].type);
  ASSERT_EQ("unconfigured", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);
  ASSERT_THAT(
    result->controller[0].claimed_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre("joint1/position", "joint1/velocity", "joint2/position"));

  cmd_cfg = {controller_interface::interface_configuration_type::ALL};
  test_controller->set_command_interface_configuration(cmd_cfg);
  state_cfg = {controller_interface::interface_configuration_type::ALL};
  test_controller->set_state_interface_configuration(state_cfg);
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("active", result->controller[0].state);
  ASSERT_THAT(
    result->controller[0].claimed_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre(
      "joint1/position", "joint1/some_unlisted_interface", "joint1/velocity", "joint2/acceleration",
      "joint2/position", "joint2/velocity", "joint3/acceleration", "joint3/position",
      "joint3/velocity", "sensor1/velocity"));

  cm_->switch_controller(
    {}, {test_controller::TEST_CONTROLLER_NAME},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(1u, result->controller.size());
  ASSERT_EQ("inactive", result->controller[0].state);
  ASSERT_TRUE(result->controller[0].claimed_interfaces.empty());
  ASSERT_THAT(
    result->controller[0].required_command_interfaces,
    testing::ElementsAre("joint1/position", "joint2/velocity", "joint3/velocity"));
  ASSERT_THAT(
    result->controller[0].required_state_interfaces,
    testing::ElementsAre(
      "joint1/position", "joint1/some_unlisted_interface", "joint1/velocity", "joint2/acceleration",
      "joint2/position", "joint2/velocity", "joint3/acceleration", "joint3/position",
      "joint3/velocity", "sensor1/velocity"));

  ASSERT_EQ(
    controller_interface::return_type::OK,
    cm_->unload_controller(test_controller::TEST_CONTROLLER_NAME));
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_EQ(0u, result->controller.size());
}

TEST_F(TestControllerManagerSrvs, reload_controller_libraries_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ReloadControllerLibraries>(
      "test_controller_manager/reload_controller_libraries");
  auto request =
    std::make_shared<controller_manager_msgs::srv::ReloadControllerLibraries::Request>();

  // Reload with no controllers running
  request->force_kill = false;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_TRUE(result->ok);

  // Add a controller, but unconfigured
  std::shared_ptr<test_controller::TestController> test_controller =
    std::dynamic_pointer_cast<test_controller::TestController>(cm_->load_controller(
      test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));

  // weak_ptr so the only controller shared_ptr instance is owned by the controller_manager and
  // can be completely destroyed before reloading the library
  std::weak_ptr<controller_interface::ControllerInterface> test_controller_weak(test_controller);

  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should have have a copy of this shared ptr";

  size_t cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  ASSERT_EQ(cleanup_calls, 1u);
  ASSERT_EQ(test_controller.use_count(), 0)
    << "No more references to the controller after reloading.";
  test_controller.reset();

  // Add a controller, but inactive
  test_controller = std::dynamic_pointer_cast<test_controller::TestController>(cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));
  test_controller_weak = test_controller;
  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);

  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should have have a copy of this shared ptr";

  cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  ASSERT_EQ(cleanup_calls, 1u);
  ASSERT_EQ(test_controller.use_count(), 0)
    << "No more references to the controller after reloading.";
  test_controller.reset();

  test_controller = std::dynamic_pointer_cast<test_controller::TestController>(cm_->load_controller(
    test_controller::TEST_CONTROLLER_NAME, test_controller::TEST_CONTROLLER_CLASS_NAME));
  test_controller_weak = test_controller;
  cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME);
  // Start Controller
  cm_->switch_controller(
    {test_controller::TEST_CONTROLLER_NAME}, {},
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());

  // Failed reload due to active controller
  request->force_kill = false;
  result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Cannot reload if controllers are running";
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, test_controller->get_state().id());
  ASSERT_GT(test_controller.use_count(), 1)
    << "Controller manager should still have have a copy of "
       "this shared ptr, no unloading was performed";

  cleanup_calls = 0;
  test_controller->cleanup_calls = &cleanup_calls;
  test_controller.reset();  // destroy our copy of the controller

  // Force stop active controller
  request->force_kill = true;
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);

  ASSERT_EQ(test_controller_weak.use_count(), 0)
    << "No more references to the controller after reloading.";
  ASSERT_EQ(cleanup_calls, 1u)
    << "Controller should have been stopped and cleaned up with force_kill = true";
}

TEST_F(TestControllerManagerSrvs, load_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::LoadController>(
      "test_controller_manager/load_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "There's no param specifying the type for " << request->name;
  rclcpp::Parameter controller_type_parameter(
    std::string(test_controller::TEST_CONTROLLER_NAME) + ".type",
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->set_parameter(controller_type_parameter);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}

TEST_F(TestControllerManagerSrvs, unload_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::UnloadController>(
      "test_controller_manager/unload_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;

  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(0u, cm_->get_loaded_controllers().size());
}

TEST_F(TestControllerManagerSrvs, configure_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ConfigureController>(
      "test_controller_manager/configure_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;

  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}

TEST_F(TestControllerManagerSrvs, load_configure_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::LoadConfigureController>(
      "test_controller_manager/load_and_configure_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::LoadConfigureController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "There's no param specifying the type for " << request->name;
  rclcpp::Parameter controller_type_parameter(
    std::string(test_controller::TEST_CONTROLLER_NAME) + ".type",
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->set_parameter(controller_type_parameter);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(test_controller::TEST_CONTROLLER_NAME, cm_->get_loaded_controllers()[0].info.name);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}

TEST_F(TestControllerManagerSrvs, load_start_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::LoadStartController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::LoadStartController>(
      "test_controller_manager/load_and_start_controller");

  auto request = std::make_shared<controller_manager_msgs::srv::LoadStartController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "There's no param specifying the type for " << request->name;
  rclcpp::Parameter controller_type_parameter(
    std::string(test_controller::TEST_CONTROLLER_NAME) + ".type",
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  cm_->set_parameter(controller_type_parameter);
  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(test_controller::TEST_CONTROLLER_NAME, cm_->get_loaded_controllers()[0].info.name);
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}

TEST_F(TestControllerManagerSrvs, configure_start_controller_srv)
{
  rclcpp::executors::SingleThreadedExecutor srv_executor;
  rclcpp::Node::SharedPtr srv_node = std::make_shared<rclcpp::Node>("srv_client");
  srv_executor.add_node(srv_node);
  rclcpp::Client<controller_manager_msgs::srv::ConfigureStartController>::SharedPtr client =
    srv_node->create_client<controller_manager_msgs::srv::ConfigureStartController>(
      "test_controller_manager/configure_and_start_controller");

  auto request =
    std::make_shared<controller_manager_msgs::srv::ConfigureStartController::Request>();
  request->name = test_controller::TEST_CONTROLLER_NAME;
  auto result = call_service_and_wait(*client, request, srv_executor);
  ASSERT_FALSE(result->ok) << "Controller not loaded: " << request->name;

  auto test_controller = std::make_shared<test_controller::TestController>();
  auto abstract_test_controller = cm_->add_controller(
    test_controller, test_controller::TEST_CONTROLLER_NAME,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());

  result = call_service_and_wait(*client, request, srv_executor, true);
  ASSERT_TRUE(result->ok);
  EXPECT_EQ(1u, cm_->get_loaded_controllers().size());
  EXPECT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    cm_->get_loaded_controllers()[0].c->get_state().id());
}
