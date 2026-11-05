// Copyright 2026 ros2_control Development Team
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

// Lives in its own executable because it replaces the global operator new to inject an
// allocation failure.

#include <gmock/gmock.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

namespace
{
// When set, the next allocation throws std::bad_alloc and the flag is cleared
std::atomic_bool fail_next_allocation{false};
}  // namespace

void * operator new(std::size_t size)
{
  if (fail_next_allocation.exchange(false))
  {
    throw std::bad_alloc();
  }
  if (void * ptr = std::malloc(size == 0 ? 1 : size))
  {
    return ptr;
  }
  throw std::bad_alloc();
}

void operator delete(void * ptr) noexcept { std::free(ptr); }
void operator delete(void * ptr, std::size_t) noexcept { std::free(ptr); }

TEST(
  ResourceManagerAllocFailure, availability_stays_consistent_when_adding_to_available_list_throws)
{
  hardware_interface::ResourceManager rm(
    std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test_resource_manager_alloc_failure"));

  // Names longer than the small string buffer, so copying one into the available list allocates
  const std::string controller_name = "controller_with_a_long_name";
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces = {
    std::make_shared<hardware_interface::CommandInterface>(
      controller_name, "first_reference_interface"),
    std::make_shared<hardware_interface::CommandInterface>(
      controller_name, "second_reference_interface")};
  rm.import_controller_reference_interfaces(controller_name, reference_interfaces);
  const auto names = rm.get_controller_reference_interface_names(controller_name);

  // The only allocation in this call is the name copy into the available list
  fail_next_allocation = true;
  EXPECT_THROW(rm.make_controller_reference_interfaces_available(controller_name), std::bad_alloc);
  fail_next_allocation = false;

  // An interface must be reported as available only if it is in the available list
  const auto available = rm.available_command_interfaces();
  for (const auto & name : names)
  {
    const bool in_list = std::find(available.begin(), available.end(), name) != available.end();
    EXPECT_EQ(rm.command_interface_is_available(name), in_list) << name;
  }

  // Recovers once allocation works again, and making the interfaces unavailable is safe
  rm.make_controller_reference_interfaces_available(controller_name);
  for (const auto & name : names)
  {
    EXPECT_TRUE(rm.command_interface_is_available(name)) << name;
  }
  rm.make_controller_reference_interfaces_unavailable(controller_name);
  for (const auto & name : names)
  {
    EXPECT_FALSE(rm.command_interface_is_available(name)) << name;
  }
  EXPECT_TRUE(rm.available_command_interfaces().empty());
}
