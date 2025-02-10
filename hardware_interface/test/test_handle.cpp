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
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
using hardware_interface::StateInterface;

namespace
{
constexpr auto JOINT_NAME = "joint_1";
constexpr auto FOO_INTERFACE = "FooInterface";
}  // namespace

TEST(TestHandle, command_interface)
{
  double value = 1.337;
  CommandInterface interface{JOINT_NAME, FOO_INTERFACE, &value};
  EXPECT_DOUBLE_EQ(interface.get_value(), value);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_DOUBLE_EQ(interface.get_value(), 0.0);
}

TEST(TestHandle, state_interface)
{
  double value = 1.337;
  StateInterface interface{JOINT_NAME, FOO_INTERFACE, &value};
  EXPECT_DOUBLE_EQ(interface.get_value(), value);
  // interface.set_value(5);  compiler error, no set_value function
}

TEST(TestHandle, name_getters_work)
{
  StateInterface handle{JOINT_NAME, FOO_INTERFACE};
  EXPECT_EQ(handle.get_name(), std::string(JOINT_NAME) + "/" + std::string(FOO_INTERFACE));
  EXPECT_EQ(handle.get_interface_name(), FOO_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME);
}

TEST(TestHandle, value_methods_throw_for_nullptr)
{
  CommandInterface handle{JOINT_NAME, FOO_INTERFACE};
  EXPECT_ANY_THROW(handle.get_value());
  EXPECT_ANY_THROW(handle.set_value(0.0));
}

TEST(TestHandle, value_methods_work_on_non_nullptr)
{
  double value = 1.337;
  CommandInterface handle{JOINT_NAME, FOO_INTERFACE, &value};
  EXPECT_DOUBLE_EQ(handle.get_value(), value);
  EXPECT_NO_THROW(handle.set_value(0.0));
  EXPECT_DOUBLE_EQ(handle.get_value(), 0.0);
}

TEST(TestHandle, interface_description_state_interface_name_getters_work)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  StateInterface handle{interface_descr};

  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
}

TEST(TestHandle, interface_description_command_interface_name_getters_work)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  CommandInterface handle{interface_descr};

  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
}

TEST(TestHandle, copy_constructor)
{
  {
    double value = 1.337;
    hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
    hardware_interface::Handle copy(handle);
    EXPECT_DOUBLE_EQ(copy.get_value(), value);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
    EXPECT_NO_THROW(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_value(), 0.0);
  }
  {
    double value = 1.337;
    InterfaceInfo info;
    info.name = FOO_INTERFACE;
    info.data_type = "double";
    InterfaceDescription itf_descr{JOINT_NAME, info};
    hardware_interface::Handle handle{itf_descr};
    EXPECT_TRUE(std::isnan(handle.get_value()));
    handle.set_value(value);
    hardware_interface::Handle copy(handle);
    EXPECT_EQ(copy.get_name(), handle.get_name());
    EXPECT_EQ(copy.get_interface_name(), handle.get_interface_name());
    EXPECT_EQ(copy.get_prefix_name(), handle.get_prefix_name());
    EXPECT_DOUBLE_EQ(copy.get_value(), value);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
    EXPECT_NO_THROW(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
    EXPECT_NO_THROW(copy.set_value(0.52));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.52);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
  }
}

TEST(TesHandle, move_constructor)
{
  double value = 1.337;
  hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
  hardware_interface::Handle moved{std::move(handle)};
  EXPECT_DOUBLE_EQ(moved.get_value(), value);
  EXPECT_NO_THROW(moved.set_value(0.0));
  EXPECT_DOUBLE_EQ(moved.get_value(), 0.0);
}

TEST(TestHandle, copy_assignment)
{
  {
    double value_1 = 1.337;
    double value_2 = 2.337;
    hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value_1};
    hardware_interface::Handle copy{JOINT_NAME, "random", &value_2};
    EXPECT_DOUBLE_EQ(copy.get_value(), value_2);
    EXPECT_DOUBLE_EQ(handle.get_value(), value_1);
    copy = handle;
    EXPECT_DOUBLE_EQ(copy.get_value(), value_1);
    EXPECT_DOUBLE_EQ(handle.get_value(), value_1);
    EXPECT_NO_THROW(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_value(), 0.0);
    EXPECT_DOUBLE_EQ(value_1, 0.0);
    EXPECT_DOUBLE_EQ(value_2, 2.337);
  }

  {
    double value = 1.337;
    InterfaceInfo info;
    info.name = FOO_INTERFACE;
    info.data_type = "double";
    InterfaceDescription itf_descr{JOINT_NAME, info};
    hardware_interface::Handle handle{itf_descr};
    EXPECT_TRUE(std::isnan(handle.get_value()));
    handle.set_value(value);
    hardware_interface::Handle copy = handle;
    EXPECT_EQ(copy.get_name(), handle.get_name());
    EXPECT_EQ(copy.get_interface_name(), handle.get_interface_name());
    EXPECT_EQ(copy.get_prefix_name(), handle.get_prefix_name());
    EXPECT_DOUBLE_EQ(copy.get_value(), value);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
    EXPECT_NO_THROW(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
    EXPECT_NO_THROW(copy.set_value(0.52));
    EXPECT_DOUBLE_EQ(copy.get_value(), 0.52);
    EXPECT_DOUBLE_EQ(handle.get_value(), value);
  }
}

TEST(TestHandle, move_assignment)
{
  double value = 1.337;
  double value_2 = 2.337;
  hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
  hardware_interface::Handle moved{JOINT_NAME, "random", &value_2};
  EXPECT_DOUBLE_EQ(moved.get_value(), value_2);
  EXPECT_DOUBLE_EQ(handle.get_value(), value);
  moved = std::move(handle);
  EXPECT_DOUBLE_EQ(moved.get_value(), value);
  EXPECT_NO_THROW(moved.set_value(0.0));
  EXPECT_DOUBLE_EQ(moved.get_value(), 0.0);
}
