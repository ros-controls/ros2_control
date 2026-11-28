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

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "hardware_interface/types/statistics_types.hpp"

using ros2_control::CompletedExecutionRecord;
using std::chrono::nanoseconds;

TEST(TestCompletedExecutionRecord, nothing_to_consume_before_a_publish)
{
  CompletedExecutionRecord record;
  EXPECT_FALSE(record.consume_new().has_value());
}

TEST(TestCompletedExecutionRecord, each_execution_is_consumed_once)
{
  CompletedExecutionRecord record;
  record.publish(nanoseconds(5));
  const auto execution = record.consume_new();
  ASSERT_TRUE(execution.has_value());
  EXPECT_EQ(execution->sequence, 1u);
  EXPECT_EQ(execution->duration, nanoseconds(5));
  EXPECT_FALSE(record.consume_new().has_value());

  record.publish(nanoseconds(7));
  ASSERT_TRUE(record.consume_new().has_value());
  EXPECT_FALSE(record.consume_new().has_value());
}

TEST(TestCompletedExecutionRecord, only_the_latest_execution_is_consumed)
{
  CompletedExecutionRecord record;
  record.publish(nanoseconds(1));
  record.publish(nanoseconds(2));
  const auto execution = record.consume_new();
  ASSERT_TRUE(execution.has_value());
  EXPECT_EQ(execution->sequence, 2u);
  EXPECT_EQ(execution->duration, nanoseconds(2));
  EXPECT_FALSE(record.consume_new().has_value());
}

TEST(TestCompletedExecutionRecord, discard_drops_the_unconsumed_execution)
{
  CompletedExecutionRecord record;
  record.publish(nanoseconds(3));
  record.discard();
  EXPECT_FALSE(record.consume_new().has_value());
  record.publish(nanoseconds(4));
  ASSERT_TRUE(record.consume_new().has_value());
}

TEST(TestCompletedExecutionRecord, a_consumed_duration_always_belongs_to_its_execution)
{
  // The n-th execution lasts n ns: a duration paired with another execution's sequence would show
  CompletedExecutionRecord record;
  constexpr uint64_t executions = 200000;
  std::atomic<bool> done{false};
  std::thread publisher(
    [&]()
    {
      for (uint64_t n = 1; n <= executions; ++n)
      {
        record.publish(nanoseconds(n));
      }
      done = true;
    });
  uint64_t consumed = 0;
  uint64_t last_sequence = 0;
  // Non-fatal checks and a break, so that the publisher is always joined: a fatal one would return
  // with the thread still joinable, and its destructor would abort the whole test executable
  while (!done || consumed == 0)
  {
    if (const auto execution = record.consume_new())
    {
      const bool coherent =
        static_cast<uint64_t>(execution->duration.count()) == execution->sequence &&
        execution->sequence > last_sequence;
      EXPECT_TRUE(coherent) << "execution " << execution->sequence << " reported with "
                            << execution->duration.count() << " ns after execution "
                            << last_sequence;
      if (!coherent)
      {
        break;
      }
      last_sequence = execution->sequence;
      ++consumed;
    }
  }
  publisher.join();
  EXPECT_GT(consumed, 0u);
}
