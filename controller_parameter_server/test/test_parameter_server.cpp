// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_parameter_server/parameter_server.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/get_env.h"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

class TestControllerParameterServer : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestControllerParameterServer, init_key_value) {
  auto ps = std::make_shared<controller_parameter_server::ParameterServer>();

  std::map<std::string, std::string> parameters =
  {{
    {"test_controller.joints.joint1", "joint1"},
    {"test_controller.joints.joint2", "joint2"},
    {"test_controller.joints.joint3", "joint3"}
  }};

  for (auto param : parameters) {
    ps->load_parameters(param.first, param.second);
  }
  for (auto param : parameters) {
    EXPECT_STREQ(param.second.c_str(), ps->get_parameter(param.first).as_string().c_str());
  }
}

void
spin(rclcpp::executor::Executor * exe)
{
  exe->spin();
}

TEST_F(TestControllerParameterServer, load_config_file) {
  const char * yaml_file;
  auto ret = rcutils_get_env("config_file", &yaml_file);
  if (ret != RCUTILS_RET_OK) {
    FAIL();
  }

  std::string file_path = std::string(yaml_file);

  auto ps = std::make_shared<controller_parameter_server::ParameterServer>();
  ps->load_parameters(file_path);

  auto client_node = std::make_shared<rclcpp::Node>("test_parameter_client_node");
  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(client_node, ps->get_name());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ps);
  executor.add_node(client_node);
  auto future_handle_ = std::async(std::launch::async, spin, &executor);

  // actually make sure that all services are up
  std::this_thread::sleep_for(1s);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCUTILS_LOG_ERROR("Interrupted while waiting for the service. Exiting.");
      FAIL();
    }
    RCUTILS_LOG_INFO("service not available, waiting again...");
  }

  std::vector<std::string> expected_keys =
  {{
    ".controller_name",
    ".controller_list.0",
    ".controller_list.1",
    ".test_joint_controller.joints",
    ".test_trajectory_controller.joints.0.joint1.name",
    ".test_trajectory_controller.joints.1",
    ".test_trajectory_controller.joints.2"
  }};

  std::vector<std::string> expected_values =
  {{
    "my_controller",
    "my_controller1",
    "my_controller2",
    "my_joint1",
    "my_joint1",
    "my_joint2",
    "my_joint3"
  }};

  auto parameter_future = parameters_client->get_parameters(expected_keys);

  std::future_status status;
  do {
    status = parameter_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::timeout) {
      FAIL();
      executor.cancel();
    }
  } while (status != std::future_status::ready);
  auto parameter = parameter_future.get();

  for (auto i = 0u; i < expected_keys.size(); ++i) {
    EXPECT_STREQ(expected_keys[i].c_str(), parameter[i].get_name().c_str());
    EXPECT_STREQ(expected_values[i].c_str(), parameter[i].value_to_string().c_str());
  }

  executor.cancel();
}
