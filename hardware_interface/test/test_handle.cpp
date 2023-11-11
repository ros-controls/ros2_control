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

#include <thread>
#include <gmock/gmock.h>
#include "hardware_interface/handle.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::AsyncCommandInterface;
using hardware_interface::AsyncStateInterface;



namespace
{
constexpr auto JOINT_NAME = "joint_1";
constexpr auto FOO_INTERFACE = "FooInterface";
constexpr auto BAR_INTERFACE = "BarInterface";

}  // namespace

TEST(TestHandle, command_interface)
{
  double value = 1.337;
  CommandInterface interface {
    JOINT_NAME, FOO_INTERFACE, &value
  };
  EXPECT_DOUBLE_EQ(interface.get_value(), value);
  EXPECT_NO_THROW(interface.set_value(0.0));
  EXPECT_DOUBLE_EQ(interface.get_value(), 0.0);
}

TEST(TestHandle, state_interface)
{
  double value = 1.337;
  StateInterface interface {
    JOINT_NAME, FOO_INTERFACE, &value
  };
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

TEST(TestHandle, no_data_race_when_accessing_value) // fails when used with regular handles due to thread sanitizer
{
  std::atomic<double> state_if_value = 1.558;
  std::atomic<double> cmd_if_value = 1.337;

  AsyncStateInterface state_handle{JOINT_NAME, FOO_INTERFACE, &state_if_value};
  AsyncCommandInterface command_handle{JOINT_NAME, FOO_INTERFACE, &cmd_if_value};


  std::thread hwif_read = std::thread([&]() {
    state_if_value.store(1.338, std::memory_order_relaxed);
  });

  std::thread controller_update = std::thread([&]() {
    command_handle.set_value(state_handle.get_value() + 0.33);
  });

  std::thread hwif_write = std::thread([&]() {
    double k = command_handle.get_value();
    cmd_if_value.store(2.0, std::memory_order_relaxed);
  });

  
  hwif_read.join();
  controller_update.join();
  hwif_write.join();
  
  EXPECT_TRUE(true);
}