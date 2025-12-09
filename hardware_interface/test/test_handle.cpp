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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/handle.hpp"
#pragma GCC diagnostic pop
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(TestHandle, command_interface)
{
  double value = 1.337;
  CommandInterface interface{JOINT_NAME, FOO_INTERFACE, &value};
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  EXPECT_NO_THROW({ std::ignore = interface.set_value(0.0); });
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), 0.0);
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(TestHandle, state_interface)
{
  double value = 1.337;
  StateInterface interface{JOINT_NAME, FOO_INTERFACE, &value};
  ASSERT_TRUE(interface.get_optional().has_value());
  EXPECT_DOUBLE_EQ(interface.get_optional().value(), value);
  // interface.set_value(5);  compiler error, no set_value function
}
#pragma GCC diagnostic pop

TEST(TestHandle, name_getters_work)
{
  StateInterface handle{JOINT_NAME, FOO_INTERFACE, nullptr};
  EXPECT_EQ(handle.get_name(), std::string(JOINT_NAME) + "/" + std::string(FOO_INTERFACE));
  EXPECT_EQ(handle.get_interface_name(), FOO_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME);
}

TEST(TestHandle, value_methods_throw_for_nullptr)
{
  CommandInterface handle{JOINT_NAME, FOO_INTERFACE, nullptr};
  EXPECT_ANY_THROW(handle.get_optional().value());
  EXPECT_ANY_THROW(std::ignore = handle.set_value(0.0));
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(TestHandle, value_methods_work_on_non_nullptr)
{
  double value = 1.337;
  CommandInterface handle{JOINT_NAME, FOO_INTERFACE, &value};
  ASSERT_TRUE(handle.get_optional().has_value());
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(0.0); });
  ASSERT_TRUE(handle.get_optional().has_value());
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), 0.0);
}
#pragma GCC diagnostic pop

TEST(TestHandle, test_command_interface_limiter_on_set)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  InterfaceInfo info;
  info.name = POSITION_INTERFACE;
  InterfaceDescription interface_descr(JOINT_NAME_1, info);
  CommandInterface handle{interface_descr};
  ASSERT_TRUE(handle.set_value(1.0));
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), 1.0);
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
    ASSERT_TRUE(handle.set_limited_value(static_cast<double>(i)));
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), i);
    ASSERT_FALSE(handle.is_limited());
  }

  for (int i = 11; i < 20; i++)
  {
    ASSERT_TRUE(handle.set_limited_value(static_cast<double>(i)));
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), 10.0);
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
  ASSERT_TRUE(handle.set_value(121.0));
  ASSERT_DOUBLE_EQ(handle.get_optional().value(), 121.0);

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

  ASSERT_TRUE(handle.set_limited_value(121.0));
  ASSERT_DOUBLE_EQ(handle.get_optional().value(), 100.0);

  std::atomic_bool done(false);
  std::thread checking_thread(
    [&handle, &done]()
    {
      while (!done)
      {
        std::optional<double> opt_value = handle.get_optional();

        if (opt_value.has_value())
        {
          EXPECT_DOUBLE_EQ(opt_value.value(), 100.0);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    });
  std::thread modifier_thread(
    [&handle, &done]()
    {
      for (int i = 100; i < 100000; i++)
      {
        std::ignore = handle.set_limited_value(static_cast<double>(i));
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      done = true;
    });

  modifier_thread.join();
  done = true;
  checking_thread.join();
  EXPECT_DOUBLE_EQ(handle.get_optional().value(), 100.0);
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
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
  ASSERT_TRUE(handle.get_optional<double>().has_value());

  ASSERT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(true); }, std::runtime_error);
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
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<bool>(); });
  ASSERT_FALSE(handle.get_optional<bool>().value()) << "Default value should be false";
  ASSERT_TRUE(handle.set_value(true));
  ASSERT_TRUE(handle.get_optional<bool>().value());
  ASSERT_EQ(handle.get_optional(), 1.0);
  ASSERT_TRUE(handle.set_value(false));
  ASSERT_FALSE(handle.get_optional<bool>().value());
  ASSERT_EQ(handle.get_optional(), 0.0);

  info.name = "some_interface";
  interface_descr = InterfaceDescription(itf_name, info);
  StateInterface handle2{interface_descr};
  EXPECT_EQ(handle2.get_name(), itf_name + "/" + "some_interface");
  ASSERT_TRUE(handle2.set_value(false));
  ASSERT_FALSE(handle2.get_optional<bool>().value());
  ASSERT_EQ(handle2.get_optional(), 0.0);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(0.0); }, std::runtime_error);

  EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
}

TEST(TestHandle, handle_constructor_double_data_type)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  StateInterface handle{JOINT_NAME_1, POSITION_INTERFACE, "double", "23.0"};

  ASSERT_EQ(hardware_interface::HandleDataType::DOUBLE, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
  ASSERT_EQ(handle.get_optional<double>().value(), 23.0);
  ASSERT_TRUE(handle.get_optional<double>().has_value());
  ASSERT_TRUE(handle.set_value(0.0));
  ASSERT_EQ(handle.get_optional<double>().value(), 0.0);
  ASSERT_TRUE(handle.set_value(1.0));
  ASSERT_EQ(handle.get_optional<double>().value(), 1.0);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(true); }, std::runtime_error);
  EXPECT_ANY_THROW({ StateInterface bad_itf("joint1", POSITION_INTERFACE, "double", "233NaN0"); })
    << "Invalid double value should throw";
}

TEST(TestHandle, handle_constructor_float_data_type)
{
  const std::string POSITION_INTERFACE = "position";
  const std::string JOINT_NAME_1 = "joint1";
  StateInterface handle{JOINT_NAME_1, POSITION_INTERFACE, "float32", "23.0"};

  ASSERT_EQ(hardware_interface::HandleDataType::FLOAT32, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), JOINT_NAME_1 + "/" + POSITION_INTERFACE);
  EXPECT_EQ(handle.get_interface_name(), POSITION_INTERFACE);
  EXPECT_EQ(handle.get_prefix_name(), JOINT_NAME_1);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<float>(); });
  EXPECT_FLOAT_EQ(handle.get_optional<float>().value(), 23.0f);
  ASSERT_TRUE(handle.get_optional<float>().has_value());
  ASSERT_TRUE(handle.set_value(static_cast<float>(0.0)));
  EXPECT_FLOAT_EQ(handle.get_optional<float>().value(), 0.0f);
  ASSERT_TRUE(handle.set_value(static_cast<float>(1.0)));
  EXPECT_FLOAT_EQ(handle.get_optional<float>().value(), 1.0f);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(true); }, std::runtime_error);
  ASSERT_THROW(
    { std::ignore = handle.set_value(std::numeric_limits<double>::max()); }, std::runtime_error);
  EXPECT_ANY_THROW({ StateInterface bad_itf("joint1", POSITION_INTERFACE, "double", "233NaN0"); })
    << "Invalid double value should throw";

  EXPECT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, handle_constructor_bool_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  StateInterface handle{itf_name, collision_interface, "bool", "true"};

  ASSERT_EQ(hardware_interface::HandleDataType::BOOL, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<bool>(); });
  ASSERT_TRUE(handle.get_optional<bool>().value())
    << "Default value should be true as it is initialized";
  ASSERT_TRUE(handle.set_value(false));
  ASSERT_FALSE(handle.get_optional<bool>().value());
  ASSERT_EQ(handle.get_optional(), 0.0);
  ASSERT_TRUE(handle.set_value(true));
  ASSERT_TRUE(handle.get_optional<bool>().value());
  ASSERT_EQ(handle.get_optional(), 1.0);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(0.0); }, std::runtime_error);

  EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
}

TEST(TestHandle, interface_description_uint8_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "uint8";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::UINT8, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::UINT8, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint8_t>(); });
  ASSERT_EQ(handle.get_optional<uint8_t>().value(), std::numeric_limits<uint8_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint8_t>(255)); });
  ASSERT_EQ(handle.get_optional<uint8_t>().value(), 255u);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint8_t>(0)); });
  ASSERT_EQ(handle.get_optional<uint8_t>().value(), 0u);
  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(256); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_int8_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "int8";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::INT8, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::INT8, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<int8_t>(); });
  ASSERT_EQ(handle.get_optional<int8_t>().value(), std::numeric_limits<int8_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int8_t>(127)); });
  ASSERT_EQ(handle.get_optional<int8_t>().value(), 127);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int8_t>(-128)); });
  ASSERT_EQ(handle.get_optional<int8_t>().value(), -128);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-129); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(128); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_uint16_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "uint16";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::UINT16, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::UINT16, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint16_t>(); });
  ASSERT_EQ(handle.get_optional<uint16_t>().value(), std::numeric_limits<uint16_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint16_t>(65535)); });
  ASSERT_EQ(handle.get_optional<uint16_t>().value(), 65535);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint16_t>(0)); });
  ASSERT_EQ(handle.get_optional<uint16_t>().value(), 0);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(65536); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_int16_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "int16";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::INT16, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::INT16, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<int16_t>(); });
  ASSERT_EQ(handle.get_optional<int16_t>().value(), std::numeric_limits<int16_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int16_t>(32767)); });
  ASSERT_EQ(handle.get_optional<int16_t>().value(), 32767);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int16_t>(-32768)); });
  ASSERT_EQ(handle.get_optional<int16_t>().value(), -32768);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-32769); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.set_value(32768); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_uint32_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "uint32";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::UINT32, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::UINT32, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint32_t>(); });
  ASSERT_EQ(handle.get_optional<uint32_t>().value(), std::numeric_limits<uint32_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint32_t>(4294967295)); });
  ASSERT_EQ(handle.get_optional<uint32_t>().value(), 4294967295);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<uint32_t>(0)); });
  ASSERT_EQ(handle.get_optional<uint32_t>().value(), 0);

  // Test the assertions
  ASSERT_THROW({ std::ignore = handle.set_value(-1.0); }, std::runtime_error);
  ASSERT_THROW(
    { std::ignore = handle.set_value(static_cast<int32_t>(4294967296)); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_int32_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "int32";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::INT32, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::INT32, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<int32_t>(); });
  ASSERT_EQ(handle.get_optional<int32_t>().value(), std::numeric_limits<int32_t>::max());
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int32_t>(2147483647)); });
  ASSERT_EQ(handle.get_optional<int32_t>().value(), 2147483647);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<int32_t>(-2147483648)); });
  ASSERT_EQ(handle.get_optional<int32_t>().value(), -2147483648);

  // Test the assertions
  ASSERT_THROW(
    { std::ignore = handle.set_value(static_cast<uint16_t>(-2147)); }, std::runtime_error);
  ASSERT_THROW(
    { std::ignore = handle.set_value(static_cast<uint16_t>(2147)); }, std::runtime_error);
  ASSERT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
}

TEST(TestHandle, interface_description_double_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "double";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::DOUBLE, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::DOUBLE, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
  ASSERT_TRUE(std::isnan(handle.get_optional<double>().value()));
  EXPECT_NO_THROW({ std::ignore = handle.set_value(1.337); });
  ASSERT_DOUBLE_EQ(handle.get_optional<double>().value(), 1.337);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(0.0); });
  ASSERT_DOUBLE_EQ(handle.get_optional<double>().value(), 0.0);
}

TEST(TestHandle, interface_description_float_data_type)
{
  const std::string collision_interface = "collision";
  const std::string itf_name = "joint1";
  InterfaceInfo info;
  info.name = collision_interface;
  info.data_type = "float32";
  InterfaceDescription interface_descr(itf_name, info);
  StateInterface handle{interface_descr};

  ASSERT_EQ(hardware_interface::HandleDataType::FLOAT32, interface_descr.get_data_type());
  ASSERT_EQ(hardware_interface::HandleDataType::FLOAT32, handle.get_data_type());
  EXPECT_EQ(handle.get_name(), itf_name + "/" + collision_interface);
  EXPECT_EQ(handle.get_interface_name(), collision_interface);
  EXPECT_EQ(handle.get_prefix_name(), itf_name);
  EXPECT_NO_THROW({ std::ignore = handle.get_optional<float>(); });
  ASSERT_TRUE(std::isnan(handle.get_optional<float>().value()));
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<float>(1.337)); });
  ASSERT_FLOAT_EQ(handle.get_optional<float>().value(), 1.337f);
  EXPECT_NO_THROW({ std::ignore = handle.set_value(static_cast<float>(0.0)); });
  ASSERT_FLOAT_EQ(handle.get_optional<float>().value(), 0.0f);

  EXPECT_THROW({ std::ignore = handle.get_optional<double>(); }, std::runtime_error);
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
  EXPECT_ANY_THROW({ StateInterface handle("joint1", "collision", "UNKNOWN"); })
    << "Unknown data type should throw";
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(TestHandle, copy_constructor)
{
  {
    double value = 1.337;
    hardware_interface::Handle handle{JOINT_NAME, FOO_INTERFACE, &value};
    hardware_interface::Handle copy(handle);
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), value);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    ASSERT_TRUE(copy.set_value(0.0));
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
    ASSERT_TRUE(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    ASSERT_TRUE(copy.set_value(0.52));
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
  ASSERT_TRUE(moved.set_value(0.0));
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
    ASSERT_TRUE(copy.set_value(0.0));
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
    ASSERT_TRUE(copy.set_value(0.0));
    EXPECT_DOUBLE_EQ(copy.get_optional().value(), 0.0);
    EXPECT_DOUBLE_EQ(handle.get_optional().value(), value);
    ASSERT_TRUE(copy.set_value(0.52));
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
  ASSERT_TRUE(moved.set_value(0.0));
  EXPECT_DOUBLE_EQ(moved.get_optional().value(), 0.0);
}
#pragma GCC diagnostic pop

class TestableHandle : public hardware_interface::Handle
{
  FRIEND_TEST(TestHandle, handle_castable);
  // Use generation of interface names
  explicit TestableHandle(const InterfaceDescription & interface_description)
  : hardware_interface::Handle(interface_description)
  {
  }
};

// @note Once we add support for more data types, this function should be updated
TEST(TestHandle, handle_castable)
{
  hardware_interface::InterfaceInfo info;
  info.name = "position";
  const std::string JOINT_NAME_1 = "joint1";
  {
    info.data_type = "double";
    info.initial_value = "23.0";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 23.0);
  }
  {
    info.data_type = "float32";
    info.initial_value = "23.0";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 23.0);
  }
  {
    info.data_type = "uint8";
    info.initial_value = "123";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 123.0);
  }
  {
    info.data_type = "int8";
    info.initial_value = "-45";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), -45.0);
  }
  {
    info.data_type = "uint16";
    info.initial_value = "32000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 32000.0);
  }
  {
    info.data_type = "int16";
    info.initial_value = "-16000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), -16000.0);
  }
  {
    info.data_type = "uint32";
    info.initial_value = "2000000000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 2000000000.0);
  }
  {
    info.data_type = "int32";
    info.initial_value = "-1000000000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), -1000000000.0);
  }
  {
    info.data_type = "bool";
    info.initial_value = "false";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    TestableHandle handle{interface_description};

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(handle.is_castable_to_double());
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 0.0);

    handle.value_ = true;
    EXPECT_EQ(handle.data_type_.cast_to_double(handle.value_), 1.0);
  }
  {
    // handle with unsupported datatype can't be created right now
    // extend with more datatypes once supported in Handle
    hardware_interface::HandleDataType dt{"string"};
    EXPECT_FALSE(dt.is_castable_to_double());
    hardware_interface::HANDLE_DATATYPE value = std::monostate{};
    EXPECT_THROW(dt.cast_to_double(value), std::runtime_error);
  }
}

// @note Once we add support for more data types, this function should be updated
TEST(TestHandle, handle_invalid_args)
{
  hardware_interface::InterfaceInfo info;
  info.name = "position";
  const std::string JOINT_NAME_1 = "joint1";
  {
    info.data_type = "double";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "float32";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "uint8";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "int8";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "uint16";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "int16";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "uint32";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "int32";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
  {
    info.data_type = "bool";
    info.initial_value = "wrong_value";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    ASSERT_THROW(hardware_interface::Handle handle{interface_description}, std::invalid_argument);
  }
}

// @note Once we add support for more data types, this function should be updated
TEST(TestHandle, handle_getters)
{
  hardware_interface::InterfaceInfo info;
  info.name = "position";
  const std::string JOINT_NAME_1 = "joint1";
  {
    info.data_type = "double";
    info.initial_value = "23.0";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<double>(); });
    EXPECT_EQ(handle.get_optional<double>().value(), 23.0);
    double val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_DOUBLE_EQ(val, 23.0);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_DOUBLE_EQ(val, 23.0);
  }
  {
    info.data_type = "float32";
    info.initial_value = "23.0";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<float>(); });
    EXPECT_EQ(handle.get_optional<float>().value(), 23.0);
    float val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_FLOAT_EQ(val, 23.0f);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_FLOAT_EQ(val, 23.0f);
  }
  {
    info.data_type = "uint8";
    info.initial_value = "123";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint8_t>(); });
    EXPECT_EQ(handle.get_optional<uint8_t>().value(), 123);
    uint8_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, 123);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, 123);
  }
  {
    info.data_type = "int8";
    info.initial_value = "-45";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<int8_t>(); });
    EXPECT_EQ(handle.get_optional<int8_t>().value(), -45);
    int8_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, -45);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, -45);
  }
  {
    info.data_type = "uint16";
    info.initial_value = "32000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint16_t>(); });
    EXPECT_EQ(handle.get_optional<uint16_t>().value(), 32000);
    uint16_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, 32000);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, 32000);
  }
  {
    info.data_type = "int16";
    info.initial_value = "-16000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<int16_t>(); });
    EXPECT_EQ(handle.get_optional<int16_t>().value(), -16000);
    int16_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, -16000);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, -16000);
  }
  {
    info.data_type = "uint32";
    info.initial_value = "2000000000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<uint32_t>(); });
    EXPECT_EQ(handle.get_optional<uint32_t>().value(), 2000000000);
    uint32_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, 2000000000);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, 2000000000);
  }
  {
    info.data_type = "int32";
    info.initial_value = "-1000000000";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<bool>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<int32_t>(); });
    EXPECT_EQ(handle.get_optional<int32_t>().value(), -1000000000);
    int32_t val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, -1000000000);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, -1000000000);
  }
  {
    info.data_type = "bool";
    info.initial_value = "true";
    hardware_interface::InterfaceDescription interface_description{JOINT_NAME_1, info};
    hardware_interface::Handle handle{interface_description};

    EXPECT_THROW({ std::ignore = handle.get_optional<float>(); }, std::runtime_error);
    EXPECT_NO_THROW({ std::ignore = handle.get_optional<bool>(); });
    EXPECT_EQ(handle.get_optional<bool>().value(), true);
    bool val;
    EXPECT_TRUE(handle.get_value(val, true));
    EXPECT_EQ(val, true);
    EXPECT_TRUE(handle.get_value(val, false));
    EXPECT_EQ(val, true);
  }
}
