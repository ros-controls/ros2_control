// Copyright 2021 PAL Robotics S.L.
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

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;

using namespace std::chrono_literals;
class TestLoadController : public ControllerManagerFixture<controller_manager::ControllerManager>
{
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
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override { update_executor_->cancel(); }

protected:
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Using a MultiThreadedExecutor so we can call update on a separate thread from service callbacks
  std::shared_ptr<rclcpp::Executor> update_executor_;
  std::future<void> update_executor_spin_future_;
};

int call_spawner(const std::string extra_args)
{
  std::string spawner_script = "ros2 run controller_manager spawner ";
  return std::system((spawner_script + extra_args).c_str());
}

int call_unspawner(const std::string extra_args)
{
  std::string spawner_script = "ros2 run controller_manager unspawner ";
  return std::system((spawner_script + extra_args).c_str());
}

TEST_F(TestLoadController, spawner_with_no_arguments_errors)
{
  EXPECT_NE(call_spawner(""), 0) << "Missing mandatory arguments";
}

TEST_F(TestLoadController, spawner_without_manager_errors)
{
  EXPECT_NE(call_spawner("ctrl_1"), 0) << "Wrong controller manager name";
}

TEST_F(TestLoadController, spawner_without_type_parameter_or_arg_errors)
{
  EXPECT_NE(call_spawner("ctrl_1 -c test_controller_manager"), 0) << "Missing .type parameter";
}

TEST_F(TestLoadController, spawner_test_type_in_param)
{
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(call_spawner("ctrl_1 -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(ctrl_1.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // Try to spawn again, it should fail because already active
  EXPECT_NE(call_spawner("ctrl_1 -c test_controller_manager"), 0) << "Cannot configure from active";

  std::vector<std::string> start_controllers = {};
  std::vector<std::string> stop_controllers = {"ctrl_1"};
  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  // We should be able to reconfigure and activate a configured controller
  EXPECT_EQ(call_spawner("ctrl_1 -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(ctrl_1.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  cm_->unload_controller("ctrl_1");
  cm_->load_controller("ctrl_1");

  // We should be able to reconfigure and activate am unconfigured loaded controller
  EXPECT_EQ(call_spawner("ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(ctrl_1.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // Unload and reload
  EXPECT_EQ(call_unspawner("ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul) << "Controller should have been unloaded";
  EXPECT_EQ(call_spawner("ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul) << "Controller should have been loaded";
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(ctrl_1.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
}

TEST_F(TestLoadController, multi_ctrls_test_type_in_param)
{
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_3.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(call_spawner("ctrl_1 ctrl_2 -c test_controller_manager"), 0);

  auto validate_loaded_controllers = [&]()
  {
    auto loaded_controllers = cm_->get_loaded_controllers();
    for (size_t i = 0; i < loaded_controllers.size(); i++)
    {
      auto ctrl = loaded_controllers[i];
      const std::string controller_name = "ctrl_" + std::to_string(i + 1);

      RCLCPP_ERROR(rclcpp::get_logger("test_controller_manager"), "%s", controller_name.c_str());
      auto it = std::find_if(
        loaded_controllers.begin(), loaded_controllers.end(),
        [&](const auto & controller) { return controller.info.name == controller_name; });
      ASSERT_TRUE(it != loaded_controllers.end());
      ASSERT_EQ(ctrl.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
      ASSERT_EQ(ctrl.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    }
  };

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);
  {
    validate_loaded_controllers();
  }

  // Try to spawn again multiple but one of them is already active, it should fail because already
  // active
  EXPECT_NE(call_spawner("ctrl_1 ctrl_3 -c test_controller_manager"), 0)
    << "Cannot configure from active";

  std::vector<std::string> start_controllers = {};
  std::vector<std::string> stop_controllers = {"ctrl_1"};
  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  // We should be able to reconfigure and activate a configured controller
  EXPECT_EQ(call_spawner("ctrl_1 ctrl_3 -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul);
  {
    validate_loaded_controllers();
  }

  stop_controllers = {"ctrl_1", "ctrl_2"};
  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  for (auto ctrl : stop_controllers)
  {
    cm_->unload_controller(ctrl);
    cm_->load_controller(ctrl);
  }

  // We should be able to reconfigure and activate am unconfigured loaded controller
  EXPECT_EQ(call_spawner("ctrl_1 ctrl_2 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul);
  {
    validate_loaded_controllers();
  }

  // Unload and reload
  EXPECT_EQ(call_unspawner("ctrl_1 ctrl_3 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul) << "Controller should have been unloaded";
  EXPECT_EQ(call_spawner("ctrl_1 ctrl_3 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul) << "Controller should have been loaded";
  {
    validate_loaded_controllers();
  }

  // Now unload everything and load them as a group in a single operation
  EXPECT_EQ(call_unspawner("ctrl_1 ctrl_2 ctrl_3 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul) << "Controller should have been unloaded";
  EXPECT_EQ(call_spawner("ctrl_1 ctrl_2 ctrl_3 -c test_controller_manager --activate-as-group"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul) << "Controller should have been loaded";
  {
    validate_loaded_controllers();
  }
}

TEST_F(TestLoadController, spawner_test_type_in_arg)
{
  // Provide controller type via -t argument
  EXPECT_EQ(
    call_spawner(
      "ctrl_2 -c test_controller_manager -t " +
      std::string(test_controller::TEST_CONTROLLER_CLASS_NAME)),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  auto ctrl_2 = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_2.info.name, "ctrl_2");
  ASSERT_EQ(ctrl_2.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(ctrl_2.c->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLoadController, unload_on_kill)
{
  // Launch spawner with unload on kill
  // timeout command will kill it after the specified time with signal SIGINT
  std::stringstream ss;
  ss << "timeout --signal=INT 5 "
     << "ros2 run controller_manager spawner "
     << "ctrl_3 -c test_controller_manager -t "
     << std::string(test_controller::TEST_CONTROLLER_CLASS_NAME) << " --unload-on-kill";

  EXPECT_NE(std::system(ss.str().c_str()), 0)
    << "timeout should have killed spawner and returned non 0 code";

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul);
}
