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
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/control_loop.hpp"
#include "controller_manager_test_common.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kDiagnosticsTopic = "/diagnostics";
constexpr auto kControllerManagerActivityDiagName = "Controller Manager Activity";
constexpr auto kPeriodicityAverageKey = "periodicity.average";
constexpr auto kMeasurementDuration = 3s;
constexpr double kPeriodicityToleranceRatio = 0.25;

testing::AssertionResult expect_periodicity_near(double expected_hz, double measured_hz)
{
  if (std::abs(measured_hz - expected_hz) <= expected_hz * kPeriodicityToleranceRatio)
  {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure()
         << "Expected cycle time of " << (1.0 / expected_hz) << " s (" << expected_hz
         << " Hz), but measured " << measured_hz << " Hz";
}

class ControlLoopRunner
{
public:
  ControlLoopRunner(
    ControllerManagerFixture<controller_manager::ControllerManager> * fixture,
    const controller_manager::ControlLoopTimingConfig & config)
  : fixture_(fixture), config_(config)
  {
    fixture_->executor_->add_node(fixture_->cm_);
    executor_spin_future_ =
      std::async(std::launch::async, [this]() { fixture_->executor_->spin(); });

    stop_loop_ = false;
    loop_thread_ = std::thread([this]() { this->control_loop(); });
    std::this_thread::sleep_for(50ms);
  }

  ~ControlLoopRunner()
  {
    stop_loop_ = true;
    if (loop_thread_.joinable())
    {
      loop_thread_.join();
    }
    fixture_->executor_->cancel();
    if (executor_spin_future_.valid())
    {
      executor_spin_future_.wait();
    }
    fixture_->executor_->remove_node(fixture_->cm_->get_node_base_interface());
  }

private:
  void control_loop()
  {
    controller_manager::ControlLoopState state;
    controller_manager::initialize_control_loop_timing(*fixture_->cm_, state);
    while (rclcpp::ok() && !stop_loop_)
    {
      if (!controller_manager::run_control_loop_cycle(*fixture_->cm_, config_, state))
      {
        break;
      }
    }
  }

  ControllerManagerFixture<controller_manager::ControllerManager> * fixture_;
  controller_manager::ControlLoopTimingConfig config_;
  std::atomic_bool stop_loop_{false};
  std::thread loop_thread_;
  std::future<void> executor_spin_future_;
};

std::optional<double> extract_periodicity_average(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  for (const auto & status : msg.status)
  {
    if (status.name.find(kControllerManagerActivityDiagName) == std::string::npos)
    {
      continue;
    }
    for (const auto & value : status.values)
    {
      if (value.key == kPeriodicityAverageKey)
      {
        return std::stod(value.value);
      }
    }
  }
  return std::nullopt;
}

double measure_controller_manager_periodicity_hz(
  ControllerManagerFixture<controller_manager::ControllerManager> * fixture,
  const controller_manager::ControlLoopTimingConfig & loop_config)
{
  std::mutex samples_mutex;
  std::vector<double> periodicity_samples;

  rclcpp::Node diag_node("diag_subscriber");
  auto subscription = diag_node.create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    kDiagnosticsTopic, rclcpp::QoS(10),
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
      const auto periodicity = extract_periodicity_average(*msg);
      if (!periodicity.has_value())
      {
        return;
      }
      std::lock_guard<std::mutex> lock(samples_mutex);
      periodicity_samples.push_back(*periodicity);
    });

  rclcpp::executors::SingleThreadedExecutor diag_executor;
  diag_executor.add_node(diag_node.get_node_base_interface());

  {
    ControlLoopRunner loop_runner(fixture, loop_config);
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < kMeasurementDuration)
    {
      diag_executor.spin_some(50ms);
    }
  }

  diag_executor.remove_node(diag_node.get_node_base_interface());

  std::lock_guard<std::mutex> lock(samples_mutex);
  EXPECT_GE(periodicity_samples.size(), 2u)
    << "Expected at least two diagnostics samples on /diagnostics";
  if (periodicity_samples.size() < 2)
  {
    return 0.0;
  }

  return (periodicity_samples[periodicity_samples.size() - 2] +
          periodicity_samples[periodicity_samples.size() - 1]) /
         2.0;
}

class TestRos2ControlNodeHardwareSync
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  explicit TestRos2ControlNodeHardwareSync(
    const std::string & robot_description, std::vector<rclcpp::Parameter> cm_parameters)
  : ControllerManagerFixture(robot_description, "", std::move(cm_parameters))
  {
  }

protected:
  void SetUp() override
  {
    ControllerManagerFixture::SetUp();
    pass_robot_description_to_cm_and_rm();
  }
};

class BlockingReadHardwareSyncTest : public TestRos2ControlNodeHardwareSync
{
public:
  BlockingReadHardwareSyncTest()
  : TestRos2ControlNodeHardwareSync(
      ros2_control_test_assets::minimal_robot_single_blocking_read_system_urdf,
      {rclcpp::Parameter("update_rate", 100), rclcpp::Parameter("diagnostic_updater.period", 0.3)})
  {
  }
};

class NonBlockingHardwareSyncTest : public TestRos2ControlNodeHardwareSync
{
public:
  NonBlockingHardwareSyncTest()
  : TestRos2ControlNodeHardwareSync(
      ros2_control_test_assets::minimal_robot_single_non_blocking_system_urdf,
      {rclcpp::Parameter("update_rate", 100), rclcpp::Parameter("diagnostic_updater.period", 0.3)})
  {
  }
};

class RateLimitedControlLoopTest : public TestRos2ControlNodeHardwareSync
{
public:
  RateLimitedControlLoopTest()
  : TestRos2ControlNodeHardwareSync(
      ros2_control_test_assets::minimal_robot_single_non_blocking_system_urdf,
      {rclcpp::Parameter("update_rate", 50), rclcpp::Parameter("diagnostic_updater.period", 0.3)})
  {
  }
};

}  // namespace

TEST_F(BlockingReadHardwareSyncTest, cycle_time_matches_blocking_read_duration)
{
  const auto measured_periodicity = measure_controller_manager_periodicity_hz(
    this, controller_manager::ControlLoopTimingConfig{.expect_blocking_read_write = true});

  EXPECT_PRED2(
    expect_periodicity_near, 1.0 / ros2_control_test_assets::TEST_BLOCKING_READ_DURATION_S,
    measured_periodicity);
}

TEST_F(NonBlockingHardwareSyncTest, cycle_time_matches_minimum_cycle_time)
{
  const auto measured_periodicity = measure_controller_manager_periodicity_hz(
    this,
    controller_manager::ControlLoopTimingConfig{
      .expect_blocking_read_write = true,
      .minimum_cycle_time = ros2_control_test_assets::TEST_HARDWARE_SYNC_MINIMUM_CYCLE_TIME_S});

  EXPECT_PRED2(
    expect_periodicity_near,
    1.0 / ros2_control_test_assets::TEST_HARDWARE_SYNC_MINIMUM_CYCLE_TIME_S, measured_periodicity);
}

TEST_F(RateLimitedControlLoopTest, cycle_time_matches_update_rate_without_hardware_sync)
{
  const auto measured_periodicity = measure_controller_manager_periodicity_hz(
    this, controller_manager::ControlLoopTimingConfig{.expect_blocking_read_write = false});

  EXPECT_PRED2(expect_periodicity_near, 50.0, measured_periodicity);
}
