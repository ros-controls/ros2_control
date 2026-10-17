// Copyright 2026 Felix Exner
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
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "controller_manager/sleeping_policies.hpp"
#include "controller_manager_test_common.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kMinimumCycleTimeS = 0.001;
constexpr int kUpdateRateHz = 100;
constexpr auto kTimingTolerance = 500us;

template <typename Func>
std::chrono::nanoseconds measure_execution(Func && func)
{
  const auto start = std::chrono::steady_clock::now();
  func();
  return std::chrono::steady_clock::now() - start;
}

class SleepingPoliciesTest : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  SleepingPoliciesTest()
  : ControllerManagerFixture(
      ros2_control_test_assets::minimal_robot_urdf, "",
      {rclcpp::Parameter("update_rate", kUpdateRateHz)})
  {
  }

protected:
  void SetUp() override
  {
    ControllerManagerFixture::SetUp();
    pass_robot_description_to_cm_and_rm();
    executor_->add_node(cm_);
    executor_spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
    std::this_thread::sleep_for(50ms);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_spin_future_.valid())
    {
      executor_spin_future_.wait();
    }
    executor_->remove_node(cm_->get_node_base_interface());
    ControllerManagerFixture::TearDown();
  }

  std::future<void> executor_spin_future_;
};

}  // namespace

TEST_F(SleepingPoliciesTest, sleep_for_blocking_read_write_does_not_sleep_when_cycle_is_long_enough)
{
  const controller_manager::ControlLoopTimingConfig config{
    .expect_blocking_read_write = true, .minimum_cycle_time = kMinimumCycleTimeS};
  controller_manager::ControlLoopState state;
  state.previous_time = rclcpp::Time(1, 0);
  state.cycle_end_time = state.previous_time + rclcpp::Duration::from_seconds(kMinimumCycleTimeS);

  const auto elapsed =
    measure_execution([&]() { sleep_for_blocking_read_write(cm_, config, state); });

  EXPECT_LT(elapsed, kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_blocking_read_write_sleeps_when_cycle_is_too_short)
{
  const controller_manager::ControlLoopTimingConfig config{
    .expect_blocking_read_write = true, .minimum_cycle_time = kMinimumCycleTimeS};
  controller_manager::ControlLoopState state;
  state.previous_time = rclcpp::Time(1, 0);
  state.cycle_end_time =
    state.previous_time + rclcpp::Duration::from_seconds(kMinimumCycleTimeS / 2);

  const auto elapsed =
    measure_execution([&]() { sleep_for_blocking_read_write(cm_, config, state); });

  EXPECT_GE(
    elapsed, std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::duration<double>(kMinimumCycleTimeS)) -
               kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_periodic_cycle_sleeps_until_next_iteration)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = true};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  state.next_iteration_time = std::chrono::steady_clock::now();
  auto expected_next_iteration_time = state.next_iteration_time + state.period;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_GE(elapsed, state.period - kTimingTolerance);
  EXPECT_LE(elapsed, state.period + kTimingTolerance);
  EXPECT_GE(state.next_iteration_time, expected_next_iteration_time - kTimingTolerance);
  EXPECT_LE(state.next_iteration_time, expected_next_iteration_time + kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_periodic_cycle_recovers_from_overrun)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = true};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  // "next iteration time" is more or less the expected time at which THIS control cycle should
  // have been started. Since we overran, this is in the past
  state.next_iteration_time = std::chrono::steady_clock::now() - 200ms;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_LT(elapsed, 100ms);
  EXPECT_GE(state.next_iteration_time, std::chrono::steady_clock::now() - kTimingTolerance);
}

TEST_F(
  SleepingPoliciesTest, sleep_for_periodic_cycle_does_not_recover_when_overrun_management_disabled)
{
  controller_manager::ControlLoopTimingConfig config{.manage_overruns = false};
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(1'000'000'000 / kUpdateRateHz);
  // "next iteration time" is more or less the expected time at which THIS control cycle should
  // have been started. Since we overran, this is in the past
  state.next_iteration_time = std::chrono::steady_clock::now() - 200ms;

  auto expected_next_iteration_time = state.next_iteration_time + state.period;

  const auto elapsed = measure_execution([&]() { sleep_for_periodic_cycle(cm_, config, state); });

  EXPECT_LT(elapsed, kTimingTolerance);  // Since the next iteration time is in the past, we should
                                         // not sleep at all

  EXPECT_LE(state.next_iteration_time, expected_next_iteration_time + kTimingTolerance);
  EXPECT_GE(state.next_iteration_time, expected_next_iteration_time - kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_sim_time_returns_true_and_sleeps_for_period)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::milliseconds(20);
  state.previous_time = cm_->get_clock()->now();

  bool sleep_result = false;
  const auto elapsed = measure_execution([&]() { sleep_result = sleep_for_sim_time(cm_, state); });

  EXPECT_TRUE(sleep_result);
  EXPECT_LE(elapsed, state.period + kTimingTolerance);
  EXPECT_GE(elapsed, state.period - kTimingTolerance);
}

TEST_F(SleepingPoliciesTest, sleep_for_sim_time_returns_false_when_clock_types_mismatch)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::milliseconds(20);
  state.previous_time = cm_->get_trigger_clock()->now();

  EXPECT_FALSE(sleep_for_sim_time(cm_, state));
}

TEST_F(SleepingPoliciesTest, update_time_base_reports_measured_period)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(10'000'000);
  state.previous_time = cm_->get_trigger_clock()->now();

  const auto current_time = state.previous_time + rclcpp::Duration::from_seconds(0.01);
  const auto measured_period = update_time_base_and_get_period(state, current_time, false);

  EXPECT_EQ(measured_period.nanoseconds(), 10'000'000);
  EXPECT_EQ(state.previous_time, current_time);
}

TEST_F(SleepingPoliciesTest, update_time_base_reports_nominal_period_after_backward_jump)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(10'000'000);
  state.previous_time = cm_->get_trigger_clock()->now();

  // Simulation time was reset, so the time base is ahead of the current time. The difference
  // would be negative and must not be handed out as a period.
  const auto current_time = state.previous_time - rclcpp::Duration::from_seconds(5.0);
  const auto measured_period = update_time_base_and_get_period(state, current_time, true);

  EXPECT_EQ(measured_period.nanoseconds(), state.period.count());
  EXPECT_EQ(state.previous_time, current_time);
}

TEST_F(SleepingPoliciesTest, sim_time_jump_handler_detects_backward_jump)
{
  controller_manager::ControlLoopState state;
  state.period = std::chrono::nanoseconds(10'000'000);

  const auto clock = cm_->get_clock();
  auto jump_handler = std::make_unique<controller_manager::SimTimeJumpHandler>(clock);
  EXPECT_FALSE(jump_handler->consume_backward_jump());

  // Move the clock backwards, as a simulator does when it resets its world.
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock->get_clock_handle()));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 10'000'000'000));
  ASSERT_TRUE(clock->ros_time_is_active());
  EXPECT_FALSE(jump_handler->consume_backward_jump());

  // The jump is reported exactly once, no matter how often it is queried.
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 1'000'000'000));
  EXPECT_TRUE(jump_handler->consume_backward_jump());
  EXPECT_FALSE(jump_handler->consume_backward_jump());

  // A forward jump must not be reported.
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 2'000'000'000));
  EXPECT_FALSE(jump_handler->consume_backward_jump());

  ASSERT_EQ(RCL_RET_OK, rcl_disable_ros_time_override(clock->get_clock_handle()));
}

// Reproduces ros-controls/ros2_control#3697: with simulation time, a sleep waiting for a deadline
// of the previous timeline must end when the simulator resets its clock backwards, instead of
// blocking until simulation time catches up with the old deadline again.
TEST_F(SleepingPoliciesTest, sleep_for_sim_time_recovers_from_backward_jump)
{
  const auto clock = cm_->get_clock();
  // The clock that sleep_for_sim_time() sleeps on is the one that has to report the jumps.
  controller_manager::SimTimeJumpHandler jump_handler(clock);

  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock->get_clock_handle()));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 10'000'000'000));
  ASSERT_TRUE(clock->ros_time_is_active());

  controller_manager::ControlLoopState state;
  state.period = std::chrono::milliseconds(100);
  state.previous_time = clock->now();

  std::atomic<bool> sleep_returned{false};
  std::thread sleeper(
    [&]()
    {
      // Blocks on a deadline of the current timeline, as the control loop does between cycles.
      sleep_for_sim_time(cm_, state);
      sleep_returned.store(true);
    });

  // Make sure the sleeper is actually blocked before resetting the clock.
  std::this_thread::sleep_for(100ms);
  ASSERT_FALSE(sleep_returned.load()) << "The sleep should be pending before the jump";

  // Reset simulation time backwards by 9 s and keep it advancing afterwards, as a simulator does
  // for `reset: {time_only: true}` or a timeline restart.
  const auto jump_start = std::chrono::steady_clock::now();
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 1'000'000'000));

  while (!sleep_returned.load() && std::chrono::steady_clock::now() - jump_start < 2s)
  {
    std::this_thread::sleep_for(5ms);
  }

  const bool returned = sleep_returned.load();
  if (returned)
  {
    sleeper.join();
  }
  else
  {
    // The sleep is stuck on the deadline of the previous timeline; advancing simulation time past
    // it lets the thread finish so the test can report the failure instead of hanging.
    EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock->get_clock_handle(), 20'000'000'000));
    sleeper.join();
  }

  EXPECT_TRUE(returned)
    << "sleep_for_sim_time() did not return after the simulation time jumped backwards";

  // The loop must also learn about the jump, so that it can re-base its time base instead of
  // handing a negative period to the controllers.
  EXPECT_TRUE(jump_handler.consume_backward_jump());

  ASSERT_EQ(RCL_RET_OK, rcl_disable_ros_time_override(clock->get_clock_handle()));
}
