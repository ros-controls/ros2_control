// Copyright 2026 Dennis Lanov
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

#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/range_sensor.hpp"
#include "sensor_msgs/msg/range.hpp"

using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;
using testing::ElementsAre;

namespace
{
constexpr char SENSOR_NAME[] = "test_range_sensor";
constexpr char RANGE_INTERFACE[] = "range";
}  // namespace

class TestableRangeSensor : public semantic_components::RangeSensor
{
public:
  using RangeSensor::RangeSensor;

  std::size_t assigned_interface_count() const { return state_interfaces_.size(); }
};

class LockableStateInterface : public StateInterface
{
public:
  using StateInterface::StateInterface;

  std::unique_lock<std::shared_mutex> lock_for_test()
  {
    return std::unique_lock<std::shared_mutex>(get_mutex());
  }
};

TEST(RangeSensorTest, reports_range_value_and_message)
{
  TestableRangeSensor range_sensor(SENSOR_NAME);

  EXPECT_THAT(
    range_sensor.get_state_interface_names(),
    ElementsAre(std::string(SENSOR_NAME) + "/" + RANGE_INTERFACE));

  double range_value = 1.25;
  auto range_state =
    std::make_shared<LockableStateInterface>(SENSOR_NAME, RANGE_INTERFACE, &range_value);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(range_state);

  ASSERT_TRUE(range_sensor.assign_loaned_state_interfaces(state_interfaces));
  ASSERT_EQ(range_sensor.assigned_interface_count(), 1u);

  EXPECT_FLOAT_EQ(range_sensor.get_range(), static_cast<float>(range_value));

  sensor_msgs::msg::Range message;
  ASSERT_TRUE(range_sensor.get_values_as_message(message));
  EXPECT_FLOAT_EQ(message.range, static_cast<float>(range_value));

  range_sensor.release_interfaces();
  EXPECT_EQ(range_sensor.assigned_interface_count(), 0u);
}

TEST(RangeSensorTest, reports_nan_when_value_lock_is_unavailable)
{
  TestableRangeSensor range_sensor(SENSOR_NAME);

  double range_value = 1.25;
  auto range_state =
    std::make_shared<LockableStateInterface>(SENSOR_NAME, RANGE_INTERFACE, &range_value);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(range_state);

  ASSERT_TRUE(range_sensor.assign_loaned_state_interfaces(state_interfaces));

  auto exclusive_lock = range_state->lock_for_test();
  ASSERT_TRUE(exclusive_lock.owns_lock());

  EXPECT_TRUE(std::isnan(range_sensor.get_range()));

  sensor_msgs::msg::Range message;
  ASSERT_TRUE(range_sensor.get_values_as_message(message));
  EXPECT_TRUE(std::isnan(message.range));
}
