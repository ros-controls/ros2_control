// Copyright 2026 ros2_control Developers
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

#include <cstddef>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

namespace
{
constexpr std::size_t kJointCount = 8;
const char * const kHardwareName = "RTAllocationSystem";
const char * const kGroupName = "RTAllocationHardwareGroupWithLongName";

// Longer than the libstdc++ small string buffer on purpose: composing the interface lookup keys
// only allocates when the name does not fit into it.
std::string joint_name(const std::size_t index)
{
  return "shoulder_pan_joint_" + std::to_string(index);
}

std::string make_urdf(const std::size_t number_of_joints)
{
  std::string urdf = R"(<?xml version="1.0"?>
<robot name="rt_allocation_test">
  <link name="base_link"/>
)";
  for (std::size_t i = 0; i < number_of_joints; ++i)
  {
    urdf += "  <link name=\"link_" + std::to_string(i) + "\"/>\n";
  }
  for (std::size_t i = 0; i < number_of_joints; ++i)
  {
    const std::string parent_link = i == 0 ? "base_link" : "link_" + std::to_string(i - 1);
    urdf += "  <joint name=\"" + joint_name(i) + "\" type=\"revolute\">\n";
    urdf += "    <parent link=\"" + parent_link + "\"/>\n";
    urdf += "    <child link=\"link_" + std::to_string(i) + "\"/>\n";
    urdf += "    <limit lower=\"-1.0\" upper=\"1.0\" effort=\"10.0\" velocity=\"1.0\"/>\n";
    urdf += "  </joint>\n";
  }
  urdf += "  <ros2_control name=\"" + std::string(kHardwareName) + "\" type=\"system\">\n";
  urdf += R"(    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
)";
  urdf += "      <group>" + std::string(kGroupName) + "</group>\n";
  urdf += "    </hardware>\n";
  for (std::size_t i = 0; i < number_of_joints; ++i)
  {
    urdf += "    <joint name=\"" + joint_name(i) + "\">\n";
    urdf += R"(      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
)";
  }
  urdf += "  </ros2_control>\n</robot>\n";
  return urdf;
}

// thread_local: the RMW/DDS and executor threads allocate on their own schedule, and a
// process-wide counter would attribute those to whichever measured cycle overlapped them. Only
// the thread running the measured loop is counted here, which is where both loops run.
thread_local bool g_counting{false};
thread_local std::size_t g_allocations{0};
thread_local std::size_t g_bytes{0};
}  // namespace

// Interposing the global allocation functions catches allocations made inside
// libhardware_interface.so, which is where the measured loop runs.
//
// ELF only: on Windows a DLL resolves operator new through its own import table, so this does not
// reach libhardware_interface.dll and the counters stay at zero. A local Windows run therefore
// does not validate the fix.
void * operator new(std::size_t size)
{
  if (g_counting)
  {
    ++g_allocations;
    g_bytes += size;
  }
  void * const ptr = std::malloc(size == 0u ? 1u : size);
  if (ptr == nullptr)
  {
    throw std::bad_alloc();
  }
  return ptr;
}

void * operator new[](std::size_t size) { return ::operator new(size); }

// std::free() is the counterpart of the std::malloc() above. Once the surrounding
// std::make_unique() calls are inlined, GCC pairs the two replacements and reports a mismatched
// delete that is not one.
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

void operator delete(void * ptr) noexcept { std::free(ptr); }

void operator delete[](void * ptr) noexcept { std::free(ptr); }

void operator delete(void * ptr, std::size_t) noexcept { std::free(ptr); }

void operator delete[](void * ptr, std::size_t) noexcept { std::free(ptr); }

#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic pop
#endif

class RTAllocationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string urdf = make_urdf(kJointCount);
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = node_.get_clock();
    params.logger = node_.get_logger();
    params.update_rate = 100u;
    params.activate_all = true;

    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, true);

    // Importing after the components are loaded is enough for enforce_command_limits(), which
    // looks the limiters up by joint name. It does not bind the per-interface limiter callback:
    // bind_command_limiter_to_interface() only runs while the command interfaces are added during
    // load_and_initialize_components(). CommandLimiterBindingTest below uses the ordering the
    // controller manager actually applies.
    resource_manager_->import_joint_limiters(urdf);
    ASSERT_TRUE(resource_manager_->are_components_initialized());

    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      claimed_interfaces_.push_back(
        resource_manager_->claim_command_interface(joint_name(i) + "/position"));
    }
  }

  /// @param command_out_of_range command every joint out of its limits each cycle, so the limiter
  /// clamps and the write-back path runs too.
  void measure_enforce_command_limits(const bool command_out_of_range)
  {
    // The reusable buffers reach their steady-state capacity here; only that is relevant.
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      command_joints_out_of_range(command_out_of_range);
      resource_manager_->enforce_command_limits(period_);
    }

    g_allocations = 0u;
    g_bytes = 0u;
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      command_joints_out_of_range(command_out_of_range);
      g_counting = true;
      resource_manager_->enforce_command_limits(period_);
      g_counting = false;
    }
  }

  void measure_read_write()
  {
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      run_read_write();
    }

    g_allocations = 0u;
    g_bytes = 0u;
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      g_counting = true;
      run_read_write();
      g_counting = false;
    }
  }

  void run_read_write()
  {
    const auto now = node_.get_clock()->now();
    resource_manager_->read(now, period_);
    resource_manager_->write(now, period_);
  }

  /// set_value() runs the bound command limiter, a separate path from enforce_command_limits(), so
  /// counting is paused around it.
  void command_joints_out_of_range(const bool enabled)
  {
    if (!enabled)
    {
      return;
    }
    g_counting = false;
    for (auto & interface : claimed_interfaces_)
    {
      ASSERT_TRUE(interface.set_value(kOutOfRangeCommand));
    }
  }

  static constexpr double kOutOfRangeCommand = 5.0;
  static constexpr std::size_t kWarmUpCycles = 50u;
  static constexpr std::size_t kMeasuredCycles = 200u;

  rclcpp::Node node_{"rt_allocations_test"};
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  std::vector<hardware_interface::LoanedCommandInterface> claimed_interfaces_;
  const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(0.01);
};

// The lookup keys were composed with fmt::format() on every cycle, which allocates for every joint
// whose name does not fit into the std::string small string buffer.
TEST_F(RTAllocationTest, enforce_command_limits_does_not_allocate)
{
  measure_enforce_command_limits(false);

  EXPECT_EQ(g_allocations, 0u)
    << "enforce_command_limits() performed " << g_allocations << " heap allocations over "
    << kMeasuredCycles << " cycles (" << g_bytes << " bytes) with " << kJointCount
    << " long-named joints. This path runs in the real-time update loop and must not touch the "
    << "heap in steady state.";
}

// Same, with the limiter clamping and the command write-back path taken.
TEST_F(RTAllocationTest, enforce_command_limits_does_not_allocate_when_clamping)
{
  measure_enforce_command_limits(true);

  EXPECT_EQ(g_allocations, 0u)
    << "enforce_command_limits() performed " << g_allocations << " heap allocations over "
    << kMeasuredCycles << " clamping cycles (" << g_bytes << " bytes) with " << kJointCount
    << " long-named joints. This path runs in the real-time update loop and must not touch the "
    << "heap in steady state.";
}

// The component and group names were copied out of the accessors into a local std::string on every
// cycle, one heap allocation per component per call once the name passed the small string buffer.
TEST_F(RTAllocationTest, read_and_write_do_not_allocate)
{
  measure_read_write();

  EXPECT_EQ(g_allocations, 0u)
    << "read()/write() performed " << g_allocations << " heap allocations over " << kMeasuredCycles
    << " cycles (" << g_bytes << " bytes). This path runs in the "
    << "real-time update loop and must not touch the heap in steady state.";
}

/// The ordering the controller manager uses, the reverse of the fixture above: limiters imported
/// before the hardware is loaded, so bind_command_limiter_to_interface() runs over the real command
/// interfaces and set_value() goes through the limiter.
class CommandLimiterBindingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string urdf = make_urdf(kJointCount);
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = node_.get_clock();
    params.logger = node_.get_logger();
    params.update_rate = 100u;
    params.activate_all = false;

    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    resource_manager_->import_joint_limiters(urdf);
    ASSERT_TRUE(resource_manager_->load_and_initialize_components(params));

    rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    ASSERT_EQ(
      resource_manager_->set_component_state(kHardwareName, active),
      hardware_interface::return_type::OK);

    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      claimed_interfaces_.push_back(
        resource_manager_->claim_command_interface(joint_name(i) + "/position"));
    }

    // Proves the limiter callback is bound; otherwise the test would measure a pass-through
    // set_value() and pass for the wrong reason.
    ASSERT_TRUE(claimed_interfaces_.front().set_value(kOutOfRangeCommand));
    ASSERT_LT(claimed_interfaces_.front().get_optional().value(), kOutOfRangeCommand);
  }

  void measure_set_value()
  {
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      for (auto & interface : claimed_interfaces_)
      {
        ASSERT_TRUE(interface.set_value(kInRangeCommand));
      }
    }

    g_allocations = 0u;
    g_bytes = 0u;
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      g_counting = true;
      for (auto & interface : claimed_interfaces_)
      {
        ASSERT_TRUE(interface.set_value(kInRangeCommand));
      }
      g_counting = false;
    }
  }

  static constexpr double kOutOfRangeCommand = 5.0;
  static constexpr double kInRangeCommand = 0.5;
  static constexpr std::size_t kWarmUpCycles = 50u;
  static constexpr std::size_t kMeasuredCycles = 200u;

  rclcpp::Node node_{"command_limiter_binding_test"};
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  std::vector<hardware_interface::LoanedCommandInterface> claimed_interfaces_;
};

// set_value() on the controller thread reaches the same limit enforcement code. It used to build a
// fresh JointInterfacesCommandLimiterData per call, whose five std::string members each allocated
// for a name past the small string buffer.
TEST_F(CommandLimiterBindingTest, set_value_with_bound_limiter_does_not_allocate)
{
  measure_set_value();

  EXPECT_EQ(g_allocations, 0u)
    << "LoanedCommandInterface::set_value() performed " << g_allocations
    << " heap allocations over " << kMeasuredCycles << " cycles (" << g_bytes << " bytes) with "
    << kJointCount
    << " long-named joints and the command limiter bound to the interfaces. The limiter callback "
    << "runs on the controller thread and must not touch the heap in steady state.";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
