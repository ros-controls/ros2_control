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
#include "example_interfaces/srv/set_bool.hpp"
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

int call_unspawner(const std::string extra_args)
{
  std::string unspawner_script =
    std::string(coveragepy_script) +
    " $(ros2 pkg prefix controller_manager)/lib/controller_manager/unspawner ";
  return std::system((unspawner_script + extra_args).c_str());
}

void verify_ctrl_parameter(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & ctrl_node, bool has_param_3)
{
  if (!ctrl_node->has_parameter("joint_names"))
  {
    ctrl_node->declare_parameter("joint_names", std::vector<std::string>({"random_joint"}));
  }
  ASSERT_THAT(
    ctrl_node->get_parameter("joint_names").as_string_array(),
    std::vector<std::string>({"joint1"}));

  if (!ctrl_node->has_parameter("param1"))
  {
    ctrl_node->declare_parameter("param1", -10.0);
  }
  ASSERT_THAT(ctrl_node->get_parameter("param1").as_double(), 1.0);

  if (!ctrl_node->has_parameter("param2"))
  {
    ctrl_node->declare_parameter("param2", -10.0);
  }
  ASSERT_THAT(ctrl_node->get_parameter("param2").as_double(), 2.0);

  if (!ctrl_node->has_parameter("param3"))
  {
    ctrl_node->declare_parameter("param3", -10.0);
  }
  ASSERT_THAT(ctrl_node->get_parameter("param3").as_double(), has_param_3 ? 3.0 : -10.0);
};

TEST_F(TestLoadController, advanced_spawner_with_no_arguments_errors)
{
  EXPECT_NE(call_advanced_spawner(""), 0) << "Missing mandatory arguments";
}

TEST_F(TestLoadController, advanced_spawner_without_manager_errors_with_given_timeout)
{
  EXPECT_NE(call_advanced_spawner("--controller ctrl_1 --controller-manager-timeout 1.0"), 0)
    << "Wrong controller manager name";
}

TEST_F(TestLoadController, advanced_spawner_without_type_parameter_or_arg_errors)
{
  EXPECT_NE(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0)
    << "Missing .type parameter";
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

TEST_F(TestLoadController, advanced_spawner_test_type_in_param)
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

  // Try to spawn again, it should fail because already active
  EXPECT_NE(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0)
    << "Cannot configure from active";

  std::vector<std::string> start_controllers = {};
  std::vector<std::string> stop_controllers = {"ctrl_1"};
  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));

  // We should be able to reconfigure and activate a configured controller
  EXPECT_EQ(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  cm_->switch_controller(
    start_controllers, stop_controllers,
    controller_manager_msgs::srv::SwitchController::Request::STRICT, true, rclcpp::Duration(0, 0));
  cm_->unload_controller("ctrl_1");
  cm_->load_controller("ctrl_1");

  // We should be able to reconfigure and activate am unconfigured loaded controller
  EXPECT_EQ(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // Unload and reload
  EXPECT_EQ(call_unspawner("ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul) << "Controller should have been unloaded";
  EXPECT_EQ(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul) << "Controller should have been loaded";
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
}

TEST_F(TestLoadController, advanced_spawner_test_with_params_file_string_parameter)
{
  const std::string test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_type.yaml");

  cm_->set_parameter(
    rclcpp::Parameter(
      "ctrl_with_parameters_and_type.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(
    rclcpp::Parameter("ctrl_with_parameters_and_type.params_file", test_file_path));

  ControllerManagerRunner cm_runner(this);
  // Provide controller type via the parsed file
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_type --load-only -c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  auto ctrl_with_parameters_and_type = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_with_parameters_and_type.info.name, "ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_with_parameters_and_type.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_with_parameters_and_type.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string(), test_file_path);
  auto ctrl_node = ctrl_with_parameters_and_type.c->get_node();
  ASSERT_THAT(
    ctrl_with_parameters_and_type.info.parameters_files,
    std::vector<std::string>({test_file_path}));
  if (!ctrl_node->has_parameter("joint_names"))
  {
    ctrl_node->declare_parameter("joint_names", std::vector<std::string>({"random_joint"}));
  }
  ASSERT_THAT(
    ctrl_node->get_parameter("joint_names").as_string_array(),
    std::vector<std::string>({"joint0"}));

  if (!ctrl_node->has_parameter("interface_name"))
  {
    ctrl_node->declare_parameter("interface_name", "invalid_interface");
  }
  ASSERT_EQ(ctrl_node->get_parameter("interface_name").as_string(), "position");
}

TEST_F(TestLoadController, advanced_spawner_test_type_in_params_file)
{
  const std::string test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_type.yaml");

  ControllerManagerRunner cm_runner(this);
  // Provide controller type via the parsed file
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_type --param-file " + test_file_path +
      " --load-only "
      "--controller chainable_ctrl_with_parameters_and_type --param-file " +
      test_file_path +
      " --load-only "
      "-c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  auto ctrl_with_parameters_and_type = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_with_parameters_and_type.info.name, "ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_with_parameters_and_type.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_with_parameters_and_type.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string_array()[0],
    test_file_path);

  auto chain_ctrl_with_parameters_and_type = cm_->get_loaded_controllers()[1];
  ASSERT_EQ(
    chain_ctrl_with_parameters_and_type.info.name, "chainable_ctrl_with_parameters_and_type");
  ASSERT_EQ(
    chain_ctrl_with_parameters_and_type.info.type,
    test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    chain_ctrl_with_parameters_and_type.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    cm_->get_parameter("chainable_ctrl_with_parameters_and_type.params_file").as_string_array()[0],
    test_file_path);

  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_no_type --param-file " + test_file_path +
      " "
      "-c test_controller_manager --controller-manager-timeout 1.0"),
    256);
  // Will still be same as the current call will fail
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  auto ctrl_1 = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_1.info.name, "ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string_array()[0],
    test_file_path);

  auto ctrl_2 = cm_->get_loaded_controllers()[1];
  ASSERT_EQ(ctrl_2.info.name, "chainable_ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_2.info.type, test_chainable_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_2.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    cm_->get_parameter("chainable_ctrl_with_parameters_and_type.params_file").as_string_array()[0],
    test_file_path);
}

TEST_F(TestLoadController, advanced_spawner_unload_on_kill)
{
  // Launch spawner with unload on kill
  // timeout command will kill it after the specified time with signal SIGINT
  ControllerManagerRunner cm_runner(this);
  cm_->set_parameter(rclcpp::Parameter("ctrl_3.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  std::stringstream ss;
  ss << "timeout --signal=INT 5 "
     << std::string(coveragepy_script) +
          " $(ros2 pkg prefix controller_manager)/lib/controller_manager/spawner "
     << "--controller ctrl_3 -c test_controller_manager --unload-on-kill";

  EXPECT_NE(std::system(ss.str().c_str()), 0)
    << "timeout should have killed spawner and returned non 0 code";

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul);
}

TEST_F(TestLoadController, advanced_spawner_unload_on_kill_activate_as_group)
{
  // Launch spawner with unload on kill
  // timeout command will kill it after the specified time with signal SIGINT
  ControllerManagerRunner cm_runner(this);
  cm_->set_parameter(rclcpp::Parameter("ctrl_3.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  std::stringstream ss;
  ss << "timeout --signal=INT 5 "
     << std::string(coveragepy_script) +
          " $(ros2 pkg prefix controller_manager)/lib/controller_manager/spawner "
     << "--controller ctrl_3 --controller ctrl_2 --activate-as-group -c test_controller_manager "
        "--unload-on-kill";

  EXPECT_NE(std::system(ss.str().c_str()), 0)
    << "timeout should have killed spawner and returned non 0 code";

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 0ul);
}

TEST_F(TestLoadController, advanced_spawner_test_to_check_parameter_overriding)
{
  const std::string main_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_type.yaml");
  const std::string overriding_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_overriding_parameters.yaml");

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_type --load-only -c "
      "test_controller_manager --param-file " +
      main_test_file_path + " --param-file " + overriding_test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  auto ctrl_with_parameters_and_type = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_with_parameters_and_type.info.name, "ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_with_parameters_and_type.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_with_parameters_and_type.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_THAT(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string_array(),
    std::vector<std::string>({main_test_file_path, overriding_test_file_path}));
  auto ctrl_node = ctrl_with_parameters_and_type.c->get_node();
  ASSERT_THAT(
    ctrl_with_parameters_and_type.info.parameters_files,
    std::vector<std::string>({main_test_file_path, overriding_test_file_path}));
  if (!ctrl_node->has_parameter("joint_names"))
  {
    ctrl_node->declare_parameter("joint_names", std::vector<std::string>({"random_joint"}));
  }
  ASSERT_THAT(
    ctrl_node->get_parameter("joint_names").as_string_array(),
    std::vector<std::string>({"joint10"}));

  if (!ctrl_node->has_parameter("interface_name"))
  {
    ctrl_node->declare_parameter("interface_name", "invalid_interface");
  }
  ASSERT_EQ(ctrl_node->get_parameter("interface_name").as_string(), "impedance")
    << "The parameter should be overridden";

  if (!ctrl_node->has_parameter("joint_offset"))
  {
    ctrl_node->declare_parameter("joint_offset", -M_PI);
  }
  ASSERT_EQ(ctrl_node->get_parameter("joint_offset").as_double(), 0.2);
}

TEST_F(TestLoadController, advanced_spawner_test_to_check_parameter_overriding_reverse)
{
  const std::string main_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_overriding_parameters.yaml");
  const std::string overriding_test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_type.yaml");

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_parameters_and_type --load-only -c "
      "test_controller_manager --param-file " +
      main_test_file_path + " --param-file " + overriding_test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  auto ctrl_with_parameters_and_type = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_with_parameters_and_type.info.name, "ctrl_with_parameters_and_type");
  ASSERT_EQ(ctrl_with_parameters_and_type.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_with_parameters_and_type.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_THAT(
    cm_->get_parameter("ctrl_with_parameters_and_type.params_file").as_string_array(),
    std::vector<std::string>({main_test_file_path, overriding_test_file_path}));
  auto ctrl_node = ctrl_with_parameters_and_type.c->get_node();
  ASSERT_THAT(
    ctrl_with_parameters_and_type.info.parameters_files,
    std::vector<std::string>({main_test_file_path, overriding_test_file_path}));
  if (!ctrl_node->has_parameter("joint_names"))
  {
    ctrl_node->declare_parameter("joint_names", std::vector<std::string>({"random_joint"}));
  }
  ASSERT_THAT(
    ctrl_node->get_parameter("joint_names").as_string_array(),
    std::vector<std::string>({"joint0"}));

  if (!ctrl_node->has_parameter("interface_name"))
  {
    ctrl_node->declare_parameter("interface_name", "invalid_interface");
  }
  ASSERT_EQ(ctrl_node->get_parameter("interface_name").as_string(), "position")
    << "The parameter should be overridden";

  if (!ctrl_node->has_parameter("joint_offset"))
  {
    ctrl_node->declare_parameter("joint_offset", -M_PI);
  }
  ASSERT_EQ(ctrl_node->get_parameter("joint_offset").as_double(), 0.2);
}

TEST_F(TestLoadController, advanced_spawner_test_fallback_controllers)
{
  const std::string test_file_path =
    std::string(PARAMETERS_FILE_PATH) +
    std::string("test_controller_spawner_with_fallback_controllers.yaml");

  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_3.type", test_controller::TEST_CONTROLLER_CLASS_NAME));

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_1 -c test_controller_manager --load-only --param-file " + test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_TRUE(ctrl_1.info.fallback_controllers_names.empty());
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_EQ(cm_->get_parameter("ctrl_1.params_file").as_string_array()[0], test_file_path);
  }

  // Try to spawn now the controller with fallback controllers inside the yaml
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_2 --param-file " + test_file_path +
      " --load-only --controller ctrl_3 --param-file " + test_file_path +
      " --load-only -c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul);
  {
    auto ctrl_1 = cm_->get_loaded_controllers()[0];
    ASSERT_EQ(ctrl_1.info.name, "ctrl_1");
    ASSERT_EQ(ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_TRUE(ctrl_1.info.fallback_controllers_names.empty());
    ASSERT_EQ(
      ctrl_1.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_EQ(cm_->get_parameter("ctrl_1.params_file").as_string_array()[0], test_file_path);

    auto ctrl_2 = cm_->get_loaded_controllers()[1];
    ASSERT_EQ(ctrl_2.info.name, "ctrl_2");
    ASSERT_EQ(ctrl_2.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_THAT(
      ctrl_2.info.fallback_controllers_names, testing::ElementsAre("ctrl_6", "ctrl_7", "ctrl_8"));
    ASSERT_EQ(
      ctrl_2.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_EQ(cm_->get_parameter("ctrl_2.params_file").as_string_array()[0], test_file_path);

    auto ctrl_3 = cm_->get_loaded_controllers()[2];
    ASSERT_EQ(ctrl_3.info.name, "ctrl_3");
    ASSERT_EQ(ctrl_3.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_THAT(ctrl_3.info.fallback_controllers_names, testing::ElementsAre("ctrl_9"));
    ASSERT_EQ(
      ctrl_3.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_EQ(cm_->get_parameter("ctrl_3.params_file").as_string_array()[0], test_file_path);
  }
}

TEST_F(TestLoadController, advanced_spawner_with_many_controllers)
{
  std::stringstream ss;
  const size_t num_controllers = 50;
  const std::string controller_base_name = "ctrl_";
  for (size_t i = 0; i < num_controllers; i++)
  {
    const std::string controller_name = controller_base_name + std::to_string(static_cast<int>(i));
    cm_->set_parameter(
      rclcpp::Parameter(controller_name + ".type", test_controller::TEST_CONTROLLER_CLASS_NAME));
    ss << "--controller " << controller_name << " ";
  }

  ControllerManagerRunner cm_runner(this);
  EXPECT_EQ(call_advanced_spawner(ss.str() + " -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), num_controllers);

  for (size_t i = 0; i < num_controllers; i++)
  {
    auto ctrl = cm_->get_loaded_controllers()[i];
    ASSERT_EQ(ctrl.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
    ASSERT_EQ(ctrl.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
}

TEST_F(TestLoadController, advanced_spawner_parsed_controller_ros_args)
{
  ControllerManagerRunner cm_runner(this);
  cm_->set_parameter(rclcpp::Parameter("ctrl_1.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  cm_->set_parameter(rclcpp::Parameter("ctrl_2.type", test_controller::TEST_CONTROLLER_CLASS_NAME));
  std::stringstream ss;

  EXPECT_EQ(call_advanced_spawner("--controller ctrl_1 -c test_controller_manager"), 0);
  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  // Now as the controller is active, we can call check for the service
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_bool_client");
  auto set_bool_service = node->create_client<example_interfaces::srv::SetBool>("/set_bool");
  ASSERT_FALSE(set_bool_service->wait_for_service(std::chrono::seconds(2)));
  ASSERT_FALSE(set_bool_service->service_is_ready());
  // Now check the service availability in the controller namespace
  auto ctrl_1_set_bool_service =
    node->create_client<example_interfaces::srv::SetBool>("/ctrl_1/set_bool");
  ASSERT_TRUE(ctrl_1_set_bool_service->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(ctrl_1_set_bool_service->service_is_ready());

  // Now test the remapping of the service name with the controller_ros_args
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_2 --controller-ros-args '-r "
      "/ctrl_2/set_bool:=/set_bool' --controller-ros-args '--param "
      "run_cycle:=20 -p test_cycle:=-11.0' -c test_controller_manager"),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  // Now as the controller is active, we can call check for the remapped service
  ASSERT_TRUE(set_bool_service->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(set_bool_service->service_is_ready());
  // Now check the service availability in the controller namespace
  auto ctrl_2_set_bool_service =
    node->create_client<example_interfaces::srv::SetBool>("/ctrl_2/set_bool");
  ASSERT_FALSE(ctrl_2_set_bool_service->wait_for_service(std::chrono::seconds(2)));
  ASSERT_FALSE(ctrl_2_set_bool_service->service_is_ready());

  // Check the parameter run_cycle to have the right value
  ASSERT_EQ("ctrl_2", cm_->get_loaded_controllers()[0].info.name);
  auto ctrl_2 = cm_->get_loaded_controllers()[0].c->get_node();
  if (!ctrl_2->has_parameter("run_cycle"))
  {
    ctrl_2->declare_parameter("run_cycle", -200);
  }
  ASSERT_THAT(ctrl_2->get_parameter("run_cycle").as_int(), 20);
  if (!ctrl_2->has_parameter("test_cycle"))
  {
    ctrl_2->declare_parameter("test_cycle", 1231.0);
  }
  ASSERT_THAT(ctrl_2->get_parameter("test_cycle").as_double(), -11.0);
}

TEST_F(TestLoadController, advanced_spawner_test_with_wildcard_entries_with_no_ctrl_name)
{
  const std::string test_file_path = std::string(PARAMETERS_FILE_PATH) +
                                     std::string("test_controller_spawner_wildcard_entries.yaml");

  ControllerManagerRunner cm_runner(this);
  // Provide controller type via the parsed file
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller wildcard_ctrl_1 --param-file " + test_file_path +
      " --controller wildcard_ctrl_2 --param-file " + test_file_path +
      " -c test_controller_manager --controller-manager-timeout 1.0 --controller wildcard_ctrl_3 "
      "--param-file " +
      test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul);

  auto wildcard_ctrl_3 = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(wildcard_ctrl_3.info.name, "wildcard_ctrl_3");
  ASSERT_EQ(wildcard_ctrl_3.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    wildcard_ctrl_3.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  verify_ctrl_parameter(wildcard_ctrl_3.c->get_node(), true);

  auto wildcard_ctrl_2 = cm_->get_loaded_controllers()[1];
  ASSERT_EQ(wildcard_ctrl_2.info.name, "wildcard_ctrl_2");
  ASSERT_EQ(wildcard_ctrl_2.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    wildcard_ctrl_2.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  verify_ctrl_parameter(wildcard_ctrl_2.c->get_node(), false);

  auto wildcard_ctrl_1 = cm_->get_loaded_controllers()[2];
  ASSERT_EQ(wildcard_ctrl_1.info.name, "wildcard_ctrl_1");
  ASSERT_EQ(wildcard_ctrl_1.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    wildcard_ctrl_1.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  verify_ctrl_parameter(wildcard_ctrl_1.c->get_node(), false);
}

TEST_F(TestLoadController, advanced_spawner_test_with_global_wildcard_entries)
{
  const std::string test_file_path =
    std::string(PARAMETERS_FILE_PATH) +
    std::string("test_controller_spawner_wildcard_entries_global.yaml");

  ControllerManagerRunner cm_runner(this);
  // Provide controller type via the parsed file
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller wildcard_ctrl -c test_controller_manager "
      "--controller-manager-timeout 1.0 "
      "--param-file " +
      test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);

  auto wildcard_ctrl = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(wildcard_ctrl.info.name, "wildcard_ctrl");
  ASSERT_EQ(wildcard_ctrl.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    wildcard_ctrl.c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  verify_ctrl_parameter(wildcard_ctrl.c->get_node(), true);
}

TEST_F(TestLoadController, advanced_spawner_test_failed_activation_of_controllers)
{
  const std::string test_file_path =
    std::string(PARAMETERS_FILE_PATH) + std::string("test_controller_spawner_with_interfaces.yaml");

  ControllerManagerRunner cm_runner(this);
  // Provide controller type via the parsed file
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_joint1_command_interface --param-file " + test_file_path +
      " --controller ctrl_with_joint2_command_interface -c  test_controller_manager "
      "--controller-manager-timeout 1.0 --param-file " +
      test_file_path),
    0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);

  auto ctrl_with_joint2_command_interface = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(ctrl_with_joint2_command_interface.info.name, "ctrl_with_joint2_command_interface");
  ASSERT_EQ(
    ctrl_with_joint2_command_interface.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(
    ctrl_with_joint2_command_interface.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(
    ctrl_with_joint2_command_interface.c->command_interface_configuration().names.size(), 1ul);
  ASSERT_THAT(
    ctrl_with_joint2_command_interface.c->command_interface_configuration().names,
    std::vector<std::string>({"joint2/velocity"}));

  auto ctrl_with_joint1_command_interface = cm_->get_loaded_controllers()[1];
  ASSERT_EQ(ctrl_with_joint1_command_interface.info.name, "ctrl_with_joint1_command_interface");
  ASSERT_EQ(
    ctrl_with_joint1_command_interface.info.type, test_controller::TEST_CONTROLLER_CLASS_NAME);
  EXPECT_EQ(
    ctrl_with_joint1_command_interface.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(
    ctrl_with_joint1_command_interface.c->command_interface_configuration().names.size(), 1ul);
  ASSERT_THAT(
    ctrl_with_joint1_command_interface.c->command_interface_configuration().names,
    std::vector<std::string>({"joint1/position"}));

  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_joint1_and_joint2_command_interfaces -c test_controller_manager "
      "--controller-manager-timeout 1.0 "
      "--param-file " +
      test_file_path),
    256)
    << "Should fail as the ctrl_with_joint1_command_interface and "
       "ctrl_with_joint2_command_interface are active";

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 3ul);

  auto ctrl_with_joint1_and_joint2_command_interfaces = cm_->get_loaded_controllers()[0];
  ASSERT_EQ(
    ctrl_with_joint1_and_joint2_command_interfaces.info.name,
    "ctrl_with_joint1_and_joint2_command_interfaces");
  ASSERT_EQ(
    ctrl_with_joint1_and_joint2_command_interfaces.info.type,
    test_controller::TEST_CONTROLLER_CLASS_NAME);
  ASSERT_EQ(
    ctrl_with_joint1_and_joint2_command_interfaces.c->get_lifecycle_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    ctrl_with_joint1_and_joint2_command_interfaces.c->command_interface_configuration()
      .names.size(),
    2ul);
  ASSERT_THAT(
    ctrl_with_joint1_and_joint2_command_interfaces.c->command_interface_configuration().names,
    std::vector<std::string>({"joint1/position", "joint2/velocity"}));

  EXPECT_EQ(call_unspawner("ctrl_with_joint1_command_interface -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 2ul);
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_joint1_and_joint2_command_interfaces -c test_controller_manager "
      "--controller-manager-timeout 1.0 "
      "--param-file " +
      test_file_path),
    256)
    << "Should fail as the ctrl_with_joint2_command_interface is still active";

  EXPECT_EQ(call_unspawner("ctrl_with_joint2_command_interface -c test_controller_manager"), 0);

  ASSERT_EQ(cm_->get_loaded_controllers().size(), 1ul);
  EXPECT_EQ(
    call_advanced_spawner(
      "--controller ctrl_with_joint1_and_joint2_command_interfaces -c test_controller_manager "
      "--controller-manager-timeout 1.0 "
      "--param-file " +
      test_file_path),
    0)
    << "Should pass as the ctrl_with_joint1_command_interface and "
       "ctrl_with_joint2_command_interface are inactive";
}
