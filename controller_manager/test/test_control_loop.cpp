// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "controller_manager/control_loop.hpp"

#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

using namespace std::literals;

namespace
{
rclcpp::Time now()
{
  thread_local auto clock = rclcpp::Clock{};
  return clock.now();
}

void sleep_until(rclcpp::Time time)
{
  std::this_thread::sleep_until(
    std::chrono::system_clock::time_point(std::chrono::nanoseconds(time.nanoseconds())));
}

}  // namespace

TEST(TestControlLoop, ExitsWhenNotOk)
{
  auto work_count = 0;
  auto do_work = [&work_count](rclcpp::Time, rclcpp::Duration) { work_count++; };

  // GIVEN ok function that returns false
  auto ok = []() { return false; };

  // WHEN we run the ControlLoop
  ControlLoop(1000ns, now, ok, do_work, sleep_until);

  // THEN we expect do_work was never called, so the work_count should be 0
  EXPECT_EQ(work_count, 0);
}

TEST(TestControlLoop, AverageReportedDurationMatchesPeriod)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 10;
  };

  // GIVEN do_work function that records duration values
  std::vector<rclcpp::Duration> durations;
  auto do_work = [&durations](rclcpp::Time, rclcpp::Duration duration) {
    durations.push_back(duration);
  };

  // WHEN we run the ControlLoop
  ControlLoop(10'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the average duration to be close to the period
  // accurate to 0.1ms. The period we set was 10ms.
  auto const average_duration = std::chrono::nanoseconds{
    std::reduce(durations.cbegin() + 1, durations.cend(), rclcpp::Duration{0, 0}).nanoseconds() /
    (durations.size() - 1)};
  EXPECT_NEAR(average_duration.count(), 10'000'000, 100'000);
}

TEST(TestControlLoop, FirstDurationIsSmall)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 2;
  };

  // GIVEN do_work function that records duration values
  std::vector<rclcpp::Duration> durations;
  auto do_work = [&durations](rclcpp::Time, rclcpp::Duration duration) {
    durations.push_back(duration);
  };

  // WHEN we run the ControlLoop
  ControlLoop(10'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the first duration to be near 0
  EXPECT_NEAR(durations.at(0).nanoseconds(), 0, 1'000);
}

TEST(TestControlLoop, DoWorkTimeGreaterThanPeriod)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes longer than the period
  std::vector<rclcpp::Duration> durations;
  auto do_work = [&durations](rclcpp::Time, rclcpp::Duration duration) {
    durations.push_back(duration);
    std::this_thread::sleep_for(20'000'000ns);
  };

  // WHEN we run the ControlLoop
  ControlLoop(10'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the second duration to be near the time the work function took
  EXPECT_NEAR(durations.at(1).nanoseconds(), 20'000'000, 1'000'000);
}

TEST(TestControlLoop, WorkTimeAtOrAfterLoopTime)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes less time than the period
  std::vector<rclcpp::Time> loop_times;
  std::vector<rclcpp::Time> work_times;
  auto do_work = [&loop_times, &work_times](rclcpp::Time time, rclcpp::Duration) {
    loop_times.push_back(time);
    work_times.push_back(now());
  };

  // WHEN we run the ControlLoop
  ControlLoop(10'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the work times to be after the loop times
  EXPECT_GT(work_times.at(0), loop_times.at(0));
  EXPECT_GT(work_times.at(1), loop_times.at(1));
}

TEST(TestControlLoop, ReportedTimesMatchPeriod)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes less time than the period
  std::vector<rclcpp::Time> times;
  auto do_work = [&times](rclcpp::Time time, rclcpp::Duration) { times.push_back(time); };

  // WHEN we run the ControlLoop
  ControlLoop(100'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the time between the loop times to be near the period
  auto const duration = times.at(1) - times.at(0);
  EXPECT_NEAR(duration.nanoseconds(), 100'000'000, 1'000'000);
}

TEST(TestControlLoop, ReportedTimesMatchReportedDuration)
{
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes less time than the period
  std::vector<rclcpp::Time> times;
  std::vector<rclcpp::Duration> durations;
  auto do_work = [&times, &durations](rclcpp::Time time, rclcpp::Duration duration) {
    times.push_back(time);
    durations.push_back(duration);
  };

  // WHEN we run the ControlLoop
  ControlLoop(10'000'000ns, now, ok, do_work, sleep_until);

  // THEN we expect the time duration to be near the reported duration
  auto const time_duration = times.at(1) - times.at(0);
  EXPECT_NEAR(time_duration.nanoseconds(), durations.at(1).nanoseconds(), 100'000);
}
