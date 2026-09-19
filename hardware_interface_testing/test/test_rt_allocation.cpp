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

#include <atomic>
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
/// Number of joints used by the allocation test.
constexpr std::size_t kJointCount = 8;

/// Name of the hardware component declared in the generated URDF.
const char * const kHardwareName = "RTAllocationSystem";

/// Name of the hardware group. Also longer than the small string buffer, see above.
const char * const kGroupName = "RTAllocationHardwareGroupWithLongName";

/// Joint names are longer than the 15 characters of the libstdc++ small string buffer.
std::string joint_name(const std::size_t index)
{
  return "shoulder_pan_joint_" + std::to_string(index);
}

/// Build a URDF with long-named revolute joints that declare position limits.
/**
 * The joint names are deliberately longer than the libstdc++ small string optimization buffer, so
 * that composing "<joint_name>/<interface_type>" for the interface lookups cannot stay on the
 * stack. This is what turns a lookup that looks allocation-free into a heap allocation per call.
 */
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
  // Both the hardware name and the group name exceed the small string buffer, so read()/write()
  // allocate when they copy them into a local std::string instead of binding to the returned
  // const reference.
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

/// Heap allocations observed through the replaced global operator new.
///
/// These are declared at namespace scope and the operator new below reads them directly rather than
/// through accessors, because they are also used by the alternate fixture further down.
std::atomic<bool> g_counting{false};
std::atomic<std::size_t> g_allocations{0};
std::atomic<std::size_t> g_bytes{0};
}  // namespace

// Replacing the global allocation functions lets the test observe every allocation made through
// them, no matter which shared library performs it. This matters here because the measured loop
// runs inside libhardware_interface.so and not in the test binary itself.
void * operator new(std::size_t size)
{
  if (g_counting.load(std::memory_order_relaxed))
  {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
    g_bytes.fetch_add(size, std::memory_order_relaxed);
  }
  void * const ptr = std::malloc(size == 0u ? 1u : size);
  if (ptr == nullptr)
  {
    throw std::bad_alloc();
  }
  return ptr;
}

void * operator new[](std::size_t size) { return ::operator new(size); }

// The replaced deallocation functions below hand the pointers back to std::free(), which is the
// counterpart of the std::malloc() in operator new above. Once the surrounding std::make_unique()
// calls are inlined, GCC pairs the two replacements and reports the std::free() as a mismatched
// delete, which it is not. Silence that single false positive instead of restructuring the test.
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

    // `load = true` so that the URDF passed through params is loaded and initialized here.
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, true);
    // The controller manager does this when 'enforce_command_limits' is enabled, which is the
    // default. Without imported joint limiters this test would measure an empty loop.
    //
    // Note the ordering: importing after the components are loaded is enough for
    // enforce_command_limits(), which looks the limiters up by joint name, but it does *not* bind
    // the per-interface limiter callback, because bind_command_limiter_to_interface() only runs
    // while the command interfaces are added during load_and_initialize_components(). See the
    // CommandLimiterBindingTest below for the ordering the controller manager actually uses.
    resource_manager_->import_joint_limiters(urdf);
    ASSERT_TRUE(resource_manager_->are_components_initialized());

    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      claimed_interfaces_.push_back(
        resource_manager_->claim_command_interface(joint_name(i) + "/position"));
    }
  }

  /// Run the real-time limit enforcement loop and return how much heap memory it used.
  /**
   * @param command_out_of_range when true, every joint is commanded out of its limits before each
   * cycle, so that the limiter clamps the command and the write-back path is exercised as well.
   */
  void measure_enforce_command_limits(const bool command_out_of_range)
  {
    // Warm up: the first cycles may allocate one-off state, and the reusable buffers only reach
    // their steady-state capacity here. Only the steady state is relevant for real-time.
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      command_joints_out_of_range(command_out_of_range);
      resource_manager_->enforce_command_limits(period_);
    }

    g_allocations.store(0u, std::memory_order_relaxed);
    g_bytes.store(0u, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      command_joints_out_of_range(command_out_of_range);
      g_counting.store(true, std::memory_order_relaxed);
      resource_manager_->enforce_command_limits(period_);
      g_counting.store(false, std::memory_order_relaxed);
    }
  }

  /// Run the real-time read()/write() cycle and return how much heap memory it used.
  void measure_read_write()
  {
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      run_read_write();
    }

    g_allocations.store(0u, std::memory_order_relaxed);
    g_bytes.store(0u, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      g_counting.store(true, std::memory_order_relaxed);
      run_read_write();
      g_counting.store(false, std::memory_order_relaxed);
    }
  }

  void run_read_write()
  {
    const auto now = node_.get_clock()->now();
    resource_manager_->read(now, period_);
    resource_manager_->write(now, period_);
  }

  /// Write an out-of-range command to every joint, or nothing when @p enabled is false.
  /**
   * Setting a command interface runs the command limiter that is bound to it, which is a separate
   * code path from enforce_command_limits(). Counting is paused around it so that the measurement
   * only attributes the allocations of enforce_command_limits() itself.
   */
  void command_joints_out_of_range(const bool enabled)
  {
    if (!enabled)
    {
      return;
    }
    g_counting.store(false, std::memory_order_relaxed);
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

// enforce_command_limits() runs on every single real-time control cycle, so it must not allocate.
// Before this was fixed, update_joint_limiters_data() and update_joint_limiters_commands()
// composed the "<joint_name>/<interface_type>" lookup keys with fmt::format(), which allocates for
// every stored joint whose name does not fit into the std::string small string buffer.
TEST_F(RTAllocationTest, enforce_command_limits_does_not_allocate)
{
  measure_enforce_command_limits(false);

  EXPECT_EQ(g_allocations.load(), 0u)
    << "enforce_command_limits() performed " << g_allocations.load() << " heap allocations over "
    << kMeasuredCycles << " cycles (" << g_bytes.load() << " bytes) with " << kJointCount
    << " long-named joints. This path runs in the real-time update loop and must not touch the "
    << "heap in steady state.";
}

// Same as above, but with the commands out of range so that the limiter clamps them and the
// command write-back path (update_joint_limiters_commands()) is taken on every cycle.
TEST_F(RTAllocationTest, enforce_command_limits_does_not_allocate_when_clamping)
{
  measure_enforce_command_limits(true);

  EXPECT_EQ(g_allocations.load(), 0u)
    << "enforce_command_limits() performed " << g_allocations.load() << " heap allocations over "
    << kMeasuredCycles << " clamping cycles (" << g_bytes.load() << " bytes) with " << kJointCount
    << " long-named joints. This path runs in the real-time update loop and must not touch the "
    << "heap in steady state.";
}

// read() and write() run in the real-time update loop as well. They copied the component name and
// the component group name out of the accessors, which return a const reference, into a local
// std::string on every cycle. With a name longer than the small string buffer that is one heap
// allocation per component per cycle for read() and one more for write().
TEST_F(RTAllocationTest, read_and_write_do_not_allocate)
{
  measure_read_write();

  EXPECT_EQ(g_allocations.load(), 0u)
    << "read()/write() performed " << g_allocations.load() << " heap allocations over "
    << kMeasuredCycles << " cycles (" << g_bytes.load() << " bytes). This path runs in the "
    << "real-time update loop and must not touch the heap in steady state.";
}

/// Reproduces the ordering the controller manager uses, which is the opposite of the fixture
/// above: the joint limiters are imported *before* the hardware is loaded, so that
/// bind_command_limiter_to_interface() runs over the real command interfaces and
/// LoanedCommandInterface::set_value() goes through the limiter.
/**
 * \sa ControllerManager::init_resource_manager(), which calls import_joint_limiters() and only
 *     afterwards load_and_initialize_components().
 */
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
    // The controller manager activates the components itself, after loading them, so the
    // ResourceManager must not do it here.
    params.activate_all = false;

    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    // Same order as the controller manager: limiters first, then load and activate.
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

    // Setting an out-of-range command must come back clamped, which proves the limiter callback is
    // actually bound. Without this the test would measure a pass-through set_value() and pass for
    // the wrong reason.
    ASSERT_TRUE(claimed_interfaces_.front().set_value(kOutOfRangeCommand));
    ASSERT_LT(claimed_interfaces_.front().get_optional().value(), kOutOfRangeCommand);
  }

  /// Call set_value() on every claimed interface and return how much heap memory it used.
  void measure_set_value()
  {
    // Warm up: the reusable scratch buffer inside the limiter callback reaches its steady-state
    // capacity here.
    for (std::size_t i = 0; i < kWarmUpCycles; ++i)
    {
      for (auto & interface : claimed_interfaces_)
      {
        ASSERT_TRUE(interface.set_value(kInRangeCommand));
      }
    }

    g_allocations.store(0u, std::memory_order_relaxed);
    g_bytes.store(0u, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kMeasuredCycles; ++i)
    {
      g_counting.store(true, std::memory_order_relaxed);
      for (auto & interface : claimed_interfaces_)
      {
        ASSERT_TRUE(interface.set_value(kInRangeCommand));
      }
      g_counting.store(false, std::memory_order_relaxed);
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

// LoanedCommandInterface::set_value() runs on the controller thread, but with the limiter bound it
// still reaches the same real-time limit enforcement code. It used to build a fresh
// JointInterfacesCommandLimiterData on every call, and set_joint_name() writes the joint name into
// five std::string members, so a name longer than the small string buffer cost five heap
// allocations per joint and call.
TEST_F(CommandLimiterBindingTest, set_value_with_bound_limiter_does_not_allocate)
{
  measure_set_value();

  EXPECT_EQ(g_allocations.load(), 0u)
    << "LoanedCommandInterface::set_value() performed " << g_allocations.load()
    << " heap allocations over " << kMeasuredCycles << " cycles (" << g_bytes.load()
    << " bytes) with " << kJointCount
    << " long-named joints and the command limiter bound to the interfaces. The limiter callback "
    << "runs on the controller thread and must not touch the heap in steady state.";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
