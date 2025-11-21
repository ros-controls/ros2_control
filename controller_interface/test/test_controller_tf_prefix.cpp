// Copyright (c) 2025, ros2_control developers
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

#include "controller_interface/tf_prefix.hpp"
#include "test_controller_tf_prefix.hpp"

TEST_F(TestControllerTFPrefix, EmptyPrefixReturnsEmpty)
{
  EXPECT_EQ(controller_interface::resolve_tf_prefix("", "/ns"), "");
}

TEST_F(TestControllerTFPrefix, ExplicitPrefixUsed)
{
  EXPECT_EQ(controller_interface::resolve_tf_prefix("robot", "/ns"), "robot/");
}

TEST_F(TestControllerTFPrefix, NormalizePrefixSlashes)
{
  EXPECT_EQ(controller_interface::resolve_tf_prefix("/robot1", "/ns"), "robot1/");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("robot2//", "/ns"), "robot2//");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("/robot3/", "/ns"), "robot3/");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("/", "/ns"), "");
}

TEST_F(TestControllerTFPrefix, TildePrefixResolvesToNamespace)
{
  EXPECT_EQ(controller_interface::resolve_tf_prefix("~", "/ns"), "ns/");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("~/", "/ns"), "ns/");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("~/robot", "/ns"), "ns/robot/");
  EXPECT_EQ(controller_interface::resolve_tf_prefix("/~/robot/", "ns"), "ns/robot/");
}
