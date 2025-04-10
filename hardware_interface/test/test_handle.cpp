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

#include "gmock/gmock.h"
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
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  EXPECT_NO_THROW({ interface.set_value(0.0); });
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), 0.0);
}

TEST(TestHandle, state_interface)
{
  double value = 1.337;
  StateInterface interface{JOINT_NAME, FOO_INTERFACE, &value};
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
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
  EXPECT_ANY_THROW(handle.get_optional().value());
  EXPECT_ANY_THROW(bool status = handle.set_value(0.0));
}

TEST(TestHandle, value_methods_work_on_non_nullptr)
{
  double value = 1.337;
  CommandInterface handle{JOINT_NAME, FOO_INTERFACE, &value};
  ASSERT_TRUE(handle.get_optional().has_value());
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
  EXPECT_NO_THROW({ handle.set_value(0.0); });
  ASSERT_TRUE(handle.get_optional().has_value());
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), 0.0);
}

TEST(TestHandle, test_command_interface_limiter_on_set)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  CommandInterface handle{interface_descr};
  handle.set_value(1.0);
  EXPECT_DOUBLE_EQ(handle.get_value(), 1.0);
  ASSERT_FALSE(handle.is_limited());

  handle.set_on_set_command_limiter(
    [](double value, bool & is_limited) -> double
    {
      is_limited = false;
      if (value > 10.0)
      {
        is_limited = true;
        return 10.0;
      }
      return value;
    });
  for (int i = 0; i < 10; i++)
  {
    handle.set_limited_value(static_cast<double>(i));
    EXPECT_DOUBLE_EQ(handle.get_value(), i);
    ASSERT_FALSE(handle.is_limited());
  }

  for (int i = 11; i < 20; i++)
  {
    handle.set_limited_value(static_cast<double>(i));
    EXPECT_DOUBLE_EQ(handle.get_value(), 10.0);
    ASSERT_TRUE(handle.is_limited());
  }
}

TEST(TestHandle, test_command_interface_limiter_on_set_different_threads)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  CommandInterface handle{interface_descr};
  handle.set_value(121.0);
  ASSERT_DOUBLE_EQ(handle.get_value(), 121.0);

  handle.set_on_set_command_limiter(
    [](double value, bool & is_limited) -> double
    {
      is_limited = false;
      if (std::abs(value) > 100.0)
      {
        is_limited = true;
        return std::copysign(100.0, value);
      }
      return value;
    });

  handle.set_limited_value(121.0);
  ASSERT_DOUBLE_EQ(handle.get_value(), 100.0);

  std::atomic_bool done(false);
  std::thread checking_thread(
    [&handle, &done]()
    {
      while (!done)
      {
        double value;
        if (handle.get_value(value))
        {
          EXPECT_DOUBLE_EQ(value, 100.0);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    });
  std::thread modifier_thread(
    [&handle, &done]()
    {
      for (int i = 100; i < 100000; i++)
      {
        handle.set_limited_value(static_cast<double>(i));
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      done = true;
    });

  modifier_thread.join();
  done = true;
  checking_thread.join();
  EXPECT_DOUBLE_EQ(handle.get_value(), 100.0);
}

TEST(TestHandle, interface_description_state_interface_name_getters_work)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::DOUBLE, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::DOUBLE, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
  EXPECT_NO_THROW({ handle.get_optional<double>(); });
  ASSERT_TRUE(handle.get_optional<double>().has_value());

  ASSERT_THROW({ handle.get_optional<bool>(); }, std::runtime_error);
  ASSERT_THROW({ handle.set_value(true); }, std::runtime_error);
}

TEST(TestHandle, interface_description_bool_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "bool";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::BOOL, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::BOOL, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ handle.get_optional<bool>(); });
  ASSERT_FALSE(handle.get_optional<bool>().value()) << "Default value should be false";
  EXPECT_NO_THROW({ handle.set_value(true); });
  ASSERT_TRUE(handle.get_optional<bool>().value());
  EXPECT_NO_THROW({ handle.set_value(false); });
  ASSERT_FALSE(handle.get_optional<bool>().value());

  // Test the assertions
  ASSERT_THROW({ handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW({ handle.set_value(0.0); }, std::runtime_error);
  ASSERT_THROW({ handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_unknown_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "unknown";
  InterfaceDescription interface_descr(itf_name, info);

  ASSERT_EQ(hardware_interface::HandleDataType::UNKNOWN, interface_descr.get_data_type());
  EXPECT_ANY_THROW({ StateInterface handle{interface_descr}; }) << "Unknown data type should throw";
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
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    EXPECT_NO_THROW({ copy.set_value(0.0); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), 0.0);
  }
  {
    double value = 1.337;
    InterfaceInfo info;
    info.name = FOO_INTERFACE;
    info.data_type = "double";
    InterfaceDescription itf_descr{JOINT_NAME, info};
    hardware_interface::Handle handle{itf_descr};
    EXPECT_TRUE(std::isnan(handle.get_optional().value()));
    ASSERT_TRUE(handle.set_value(value));
    hardware_interface::Handle copy(handle);
    EXPECT_EQ(copy.get_name(), handle.get_name());
    EXPECT_EQ(copy.get_interface_name(), handle.get_interface_name());
    EXPECT_EQ(copy.get_prefix_name(), handle.get_prefix_name());
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    EXPECT_NO_THROW({ copy.set_value(0.0); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    EXPECT_NO_THROW({ copy.set_value(0.52); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.52);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
  }
}

TEST(TesHandle, move_constructor)
{
  double value = 1.337;
  hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
  hardware_interface::Handle moved{std::move(handle)};
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), value);
  EXPECT_NO_THROW({ moved.set_value(0.0); });
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), 0.0);
}

TEST(TestHandle, copy_assignment)
{
  {
    double value_1 = 1.337;
    double value_2 = 2.337;
    hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value_1};
    hardware_interface::Handle copy{JOINT_NAME, "random", &value_2};
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value_2);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value_1);
    copy = handle;
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value_1);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value_1);
    EXPECT_NO_THROW({ copy.set_value(0.0); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), 0.0);
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
    EXPECT_TRUE(std::isnan(handle.get_optional().value()));
    ASSERT_TRUE(handle.set_value(value));
    hardware_interface::Handle copy = handle;
    EXPECT_EQ(copy.get_name(), handle.get_name());
    EXPECT_EQ(copy.get_interface_name(), handle.get_interface_name());
    EXPECT_EQ(copy.get_prefix_name(), handle.get_prefix_name());
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    EXPECT_NO_THROW({ copy.set_value(0.0); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    EXPECT_NO_THROW({ copy.set_value(0.52); });
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.52);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
  }
}

TEST(TestHandle, move_assignment)
{
  double value = 1.337;
  double value_2 = 2.337;
  hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
  hardware_interface::Handle moved{JOINT_NAME, "random", &value_2};
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), value_2);
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
  moved = std::move(handle);
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), value);
  EXPECT_NO_THROW({ moved.set_value(0.0); });
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), 0.0);
}
