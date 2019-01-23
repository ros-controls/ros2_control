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

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "controller_parameter_server/yaml_parser.hpp"

#include "rcutils/types.h"
#include "rcutils/get_env.h"

class TestYamlParser : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }
};

TEST_F(TestYamlParser, init_wrong) {
  controller_parameter_server::YamlParser yp;
  EXPECT_ANY_THROW(yp.parse("unknown_file"));
}

TEST_F(TestYamlParser, init) {
  const char * yaml_file;
  auto ret = rcutils_get_env("config_file", &yaml_file);
  if (ret != RCUTILS_RET_OK) {
    FAIL();
  }
  std::string file_path = std::string(yaml_file);

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

  if (expected_keys.size() != expected_values.size()) {
    FAIL();
  }

  controller_parameter_server::YamlParser yp;
  yp.parse(file_path);

  auto key_values = yp.get_key_value_pairs();

  for (size_t i = 0; i < expected_keys.size(); ++i) {
    EXPECT_STREQ(expected_values[i].c_str(), key_values[expected_keys[i]].c_str());
  }
}
