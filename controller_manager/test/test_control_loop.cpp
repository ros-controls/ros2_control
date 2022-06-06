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

#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "controller_manager/control_loop.hpp"

using namespace std::literals;

namespace
{
// Fake clock is used to inject a counter based clock into these tests
// The advantage of this over using system clock with real sleeps and now
// calls is that the test passing does not depend on the performance of the
// system it is running on. Also, as there are no actual sleeps in these
// tests (only simulated ones), the test runs blazingly fast.
class FakeClock
{
  rclcpp::Time state_ = rclcpp::Time{0};

public:
  rclcpp::Time now() { return state_; }
  void sleep_until(rclcpp::Time time)
  {
    if (time > state_)
    {
      state_ = time;
    }
  }
  void sleep_for(rclcpp::Duration duration) { state_ += duration; }
};

}  // namespace

TEST(TestControlLoop, ExitsWhenNotOk)
{
  auto fake_clock = FakeClock{};
  auto work_count = 0;
  auto do_work = [&work_count](rclcpp::Time, rclcpp::Duration) { work_count++; };

  // GIVEN ok function that returns false
  auto ok = []() { return false; };
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect do_work was never called, so the work_count should be 0
  EXPECT_EQ(work_count, 0);
}

TEST(TestControlLoop, AverageReportedDurationMatchesPeriod)
{
  auto fake_clock = FakeClock{};
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
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the average duration to be the period
  // Note that we exclude the first duration from the average as it is 0
  auto const average_duration = std::chrono::nanoseconds{
    std::reduce(durations.cbegin() + 1, durations.cend(), rclcpp::Duration{0, 0}).nanoseconds() /
    (durations.size() - 1)};
  EXPECT_EQ(average_duration.count(), 10);
}

TEST(TestControlLoop, FirstDurationIsSmall)
{
  auto fake_clock = FakeClock{};
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
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the first duration to be 0
  EXPECT_EQ(durations.at(0).nanoseconds(), 0);
}

TEST(TestControlLoop, DoWorkTimeGreaterThanPeriod)
{
  auto fake_clock = FakeClock{};
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes longer than the period
  std::vector<rclcpp::Duration> durations;
  auto do_work = [&durations, &fake_clock](rclcpp::Time, rclcpp::Duration duration) {
    durations.push_back(duration);
    fake_clock.sleep_for(rclcpp::Duration{20ns});
  };
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the second duration to be the time the work function took
  EXPECT_EQ(durations.at(1).nanoseconds(), 20);
}

TEST(TestControlLoop, WorkTimeAtOrAfterLoopTime)
{
  auto fake_clock = FakeClock{};
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes less time than the period
  std::vector<rclcpp::Time> loop_times;
  std::vector<rclcpp::Time> work_times;
  auto do_work = [&loop_times, &work_times, &fake_clock](rclcpp::Time time, rclcpp::Duration) {
    loop_times.push_back(time);
    fake_clock.sleep_for(rclcpp::Duration{2ns});
    work_times.push_back(fake_clock.now());
  };
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the work times to be after the loop times
  EXPECT_GT(work_times.at(0).nanoseconds(), loop_times.at(0).nanoseconds());
  EXPECT_GT(work_times.at(1).nanoseconds(), loop_times.at(1).nanoseconds());
}

TEST(TestControlLoop, ReportedTimesMatchPeriod)
{
  auto fake_clock = FakeClock{};
  auto loop_iterations = 0;
  auto ok = [&loop_iterations]() {
    loop_iterations++;
    return loop_iterations < 3;
  };

  // GIVEN do_work function takes less time than the period
  std::vector<rclcpp::Time> times;
  auto do_work = [&times](rclcpp::Time time, rclcpp::Duration) { times.push_back(time); };
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the time between the loop times to be the period
  auto const duration = times.at(1) - times.at(0);
  EXPECT_EQ(duration.nanoseconds(), 10);
}

TEST(TestControlLoop, ReportedTimesMatchReportedDuration)
{
  auto fake_clock = FakeClock{};
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
  auto now = [&fake_clock]() { return fake_clock.now(); };
  auto sleep_until = [&fake_clock](auto time) { fake_clock.sleep_until(time); };

  // WHEN we run the ControlLoop
  ControlLoop(10ns, now, ok, do_work, sleep_until);

  // THEN we expect the time duration to be the reported duration
  auto const time_duration = times.at(1) - times.at(0);
  EXPECT_EQ(time_duration.nanoseconds(), durations.at(1).nanoseconds());
}
