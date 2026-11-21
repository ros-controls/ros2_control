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

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "gmock/gmock.h"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
constexpr int UPDATE_RATE = 100;
constexpr int64_t WINDOW_SIZE = 50;
const char CM_ACTIVITY[] = "Controller Manager Activity";
}  // namespace

class TestPeriodicityDiagnostics
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  explicit TestPeriodicityDiagnostics(int64_t window_size = 0)
  : ControllerManagerFixture<controller_manager::ControllerManager>(
      ros2_control_test_assets::minimal_robot_urdf, "",
      {rclcpp::Parameter("update_rate", UPDATE_RATE),
       rclcpp::Parameter("diagnostics.controller_manager.periodicity.window_size", window_size),
       rclcpp::Parameter("diagnostic_updater.period", 0.05)})
  {
  }

  void run_slow_start_then_steady(int steady_cycles)
  {
    cm_->read(time_, rclcpp::Duration::from_seconds(0.125));
    cm_->read(time_, rclcpp::Duration::from_seconds(0.0001));
    for (int i = 0; i < steady_cycles; ++i)
    {
      cm_->read(time_, rclcpp::Duration::from_seconds(1.0 / UPDATE_RATE));
    }
  }

  std::optional<diagnostic_msgs::msg::DiagnosticStatus> next_cm_activity()
  {
    auto listener = std::make_shared<rclcpp::Node>("periodicity_diagnostics_listener");
    std::optional<diagnostic_msgs::msg::DiagnosticStatus> received;
    auto sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS(),
      [&received](const diagnostic_msgs::msg::DiagnosticArray & msg)
      {
        for (const auto & status : msg.status)
        {
          if (status.name.find(CM_ACTIVITY) != std::string::npos)
          {
            received = status;
          }
        }
      });
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(cm_);
    exec.add_node(listener);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline)
    {
      exec.spin_some(std::chrono::milliseconds(10));
    }
    exec.remove_node(cm_);
    return received;
  }
};

class TestPeriodicityDiagnosticsWithWindow : public TestPeriodicityDiagnostics
{
public:
  TestPeriodicityDiagnosticsWithWindow() : TestPeriodicityDiagnostics(WINDOW_SIZE) {}
};

std::map<std::string, std::string> values(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  std::map<std::string, std::string> out;
  for (const auto & kv : status.values)
  {
    out[kv.key] = kv.value;
  }
  return out;
}

TEST_F(TestPeriodicityDiagnostics, without_window_a_startup_stall_keeps_the_diagnostic_in_error)
{
  run_slow_start_then_steady(1000);

  const auto status = next_cm_activity();
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(values(*status).count("periodicity.recent.standard_deviation"), 0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(TestPeriodicityDiagnosticsWithWindow, with_window_the_level_recovers_after_a_startup_stall)
{
  run_slow_start_then_steady(1000);

  const auto status = next_cm_activity();
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;

  const auto v = values(*status);
  EXPECT_EQ(v.at("periodicity.recent.window_size"), std::to_string(WINDOW_SIZE));
  EXPECT_EQ(v.at("periodicity.recent.sample_count"), std::to_string(WINDOW_SIZE));
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.average")), UPDATE_RATE, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.standard_deviation")), 0.0, 1e-3);

  EXPECT_NEAR(std::stod(v.at("periodicity.min")), 8.0, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.max")), 10000.0, 1e-3);
  EXPECT_GT(std::stod(v.at("periodicity.standard_deviation")), 10.0);
}

TEST_F(TestPeriodicityDiagnosticsWithWindow, with_window_a_recent_stall_is_still_an_error)
{
  run_slow_start_then_steady(WINDOW_SIZE / 2);

  const auto status = next_cm_activity();
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
}
