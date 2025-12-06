// Copyright 2025 PAL Robotics S.L.
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

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "gmock/gmock.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "test_chainable_controller/test_chainable_controller.hpp"
#include "test_controller/test_controller.hpp"

using ::testing::_;
using ::testing::Return;
const char coveragepy_script[] = "python3 -m coverage run --append --branch";

using namespace std::chrono_literals;
class TestLoadController : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
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

int call_advanced_spawner(const std::string extra_args)
{
  std::string spawner_script =
    std::string(coveragepy_script) +
    " $(ros2 pkg prefix controller_manager)/lib/controller_manager/spawner ";
  return std::system((spawner_script + extra_args).c_str());
}

TEST_F(TestLoadController, advanced_spawner_single_controller)
{
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
}

TEST_F(TestLoadController, advanced_spawner_multiple_controllers)
{
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  // ctrl_1 inactive, ctrl_2 active (default)
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_1 --inactive --controller ctrl_2 -c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  auto loaded_controllers = cm_->get_loaded_controllers();

  auto it_ctrl_1 = std::find_if(
    loaded_controllers.begin(), loaded_controllers.end(),
    [](const auto & c) { return c.info.name == "ctrl_1"; });
  ASSERT_NE(it_ctrl_1, loaded_controllers.end());
  ASSERT_EQ(
    it_ctrl_1->c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto it_ctrl_2 = std::find_if(
    loaded_controllers.begin(), loaded_controllers.end(),
    [](const auto & c) { return c.info.name == "ctrl_2"; });
  ASSERT_NE(it_ctrl_2, loaded_controllers.end());
  ASSERT_EQ(
    it_ctrl_2->c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLoadController, advanced_spawner_activate_as_group)
{
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(
    call_advanced_spawner(
      "--activate-as-group --controller ctrl_1 --controller ctrl_2 -c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  auto loaded_controllers = cm_->get_loaded_controllers();

  auto it_ctrl_1 = std::find_if(
    loaded_controllers.begin(), loaded_controllers.end(),
    [](const auto & c) { return c.info.name == "ctrl_1"; });
  ASSERT_NE(it_ctrl_1, loaded_controllers.end());
  ASSERT_EQ(
    it_ctrl_1->c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto it_ctrl_2 = std::find_if(
    loaded_controllers.begin(), loaded_controllers.end(),
    [](const auto & c) { return c.info.name == "ctrl_2"; });
  ASSERT_NE(it_ctrl_2, loaded_controllers.end());
  ASSERT_EQ(
    it_ctrl_2->c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLoadController, advanced_spawner_multiple_param_files)
{
  const std::string main_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_type.yaml");
  const std::string overriding_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_overriding_parameters.yaml");

  ControllerManagerRunner cm_runner(this);

  // Pass multiple param files to ctrl_1
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_type --load-only -c test_controller_manager "
      "--param-file " +
      main_test_file_path + " --param-file " + overriding_test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  auto ctrl = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl.info.name, "ctrl_with_parameters_and_type");
  ASSERT_THAT(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string_array(),
    std::vector<std::string>({main_test_file_path, overriding_test_file_path}));
}

TEST_F(TestLoadController, advanced_spawner_unload_on_kill_only_active)
{
  // Launch spawner with unload on kill
  // timeout command will kill it after the specified time with signal SIGINT
  ControllerManagerRunner cm_runner(this);
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  // ctrl_1 inactive, ctrl_2 active
  // On kill, ctrl_2 should be unloaded, ctrl_1 should remain loaded (but unconfigured/inactive)

  std::stringstream ss;
  ss << "timeout --signal=INT 5 "
     << std::string(coveragepy_script) +
          " $(ros2 pkg prefix controller_manager)/lib/controller_manager/spawner "
     << "--unload-on-kill -c test_controller_manager "
     << "--controller ctrl_1 --inactive "
     << "--controller ctrl_2";

  EXPECT_NE(std::system(ss.str().c_str()), 0)
    << "timeout should have killed spawner and returned non 0 code";

  // Check what remains
  // ctrl_2 should be gone. ctrl_1 should be there.

  auto loaded = cm_->get_loaded_controllers();
  ASSERT_EQ(loaded.size(), 1ul);
  ASSERT_EQ(loaded[0].info.name, "ctrl_1");
  ASSERT_EQ(
    loaded[0].c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}
