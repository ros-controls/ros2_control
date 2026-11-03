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
#include "test_controller/test_controller.hpp"

namespace
{
constexpr int UPDATE_RATE = 100;
constexpr double WINDOW_DURATION = 1.0;
const char CM_ACTIVITY[] = "Controller Manager Activity";
const char CONTROLLERS_ACTIVITY[] = "Controllers Activity";
constexpr int CONTROLLER_UPDATE_RATE = UPDATE_RATE / 2;

std::map<std::string, std::string> values(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  std::map<std::string, std::string> out;
  for (const auto & kv : status.values)
  {
    out[kv.key] = kv.value;
  }
  return out;
}
}  // namespace

class TestPeriodicityDiagnostics
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  explicit TestPeriodicityDiagnostics(double window_duration = -1.0)
  : ControllerManagerFixture<controller_manager::ControllerManager>(
      ros2_control_test_assets::minimal_robot_urdf, "",
      {rclcpp::Parameter("update_rate", UPDATE_RATE),
       rclcpp::Parameter(
         "diagnostics.controller_manager.periodicity.window_duration", window_duration),
       rclcpp::Parameter("diagnostics.controllers.periodicity.window_duration", window_duration),
       rclcpp::Parameter("diagnostic_updater.period", 0.05)})
  {
  }

  /// Run a read cycle that took @p period, advancing the time accordingly
  void read_cycle(double period)
  {
    const auto duration = rclcpp::Duration::from_seconds(period);
    time_ += duration;
    cm_->read(time_, duration);
  }

  /// A slow and a fast cycle, as when the system is still bringing up other processes
  void stall()
  {
    read_cycle(0.125);
    read_cycle(0.0001);
  }

  void steady_cycles(int cycles)
  {
    for (int i = 0; i < cycles; ++i)
    {
      read_cycle(1.0 / UPDATE_RATE);
    }
  }

  /// Load, configure and activate a test controller running at half the CM rate, so that its
  /// periodicity is diagnosed
  controller_manager::ControllerSpec add_active_controller()
  {
    auto controller = std::make_shared<test_controller::TestController>();
    cm_->add_controller(
      controller, test_controller::TEST_CONTROLLER_NAME,
      test_controller::TEST_CONTROLLER_CLASS_NAME);
    controller->get_node()->set_parameter({"update_rate", CONTROLLER_UPDATE_RATE});
    {
      ControllerManagerRunner cm_runner(this);
      EXPECT_EQ(
        controller_interface::return_type::OK,
        cm_->configure_controller(test_controller::TEST_CONTROLLER_NAME));
      EXPECT_EQ(
        controller_interface::return_type::OK,
        cm_->switch_controller(
          {test_controller::TEST_CONTROLLER_NAME}, {},
          controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
          rclcpp::Duration(0, 0)));
    }
    return cm_->get_loaded_controllers().at(0);
  }

  /// Feed periodicity statistics with a stall at bring-up followed by 3 s of steady cycles
  template <typename AddMeasurement>
  static void stall_then_steady(int rate, AddMeasurement add_measurement)
  {
    double t = 0.0;
    add_measurement(8.0, t);
    t += 0.125;
    add_measurement(10000.0, t);
    t += 0.0001;
    for (int i = 0; i < 3 * rate; ++i)
    {
      t += 1.0 / rate;
      add_measurement(static_cast<double>(rate), t);
    }
  }

  /// The periodicity of the controller goes through a stall at bring-up and is steady since
  static void controller_stall_then_steady(const controller_manager::ControllerSpec & spec)
  {
    spec.periodicity_statistics->reset();
    spec.recent_periodicity_statistics->reset();
    stall_then_steady(
      CONTROLLER_UPDATE_RATE,
      [&spec](double value, double t)
      {
        spec.periodicity_statistics->add_measurement(value);
        spec.recent_periodicity_statistics->add_measurement(value, t);
      });
  }

  std::optional<diagnostic_msgs::msg::DiagnosticStatus> next_status(const std::string & name)
  {
    auto listener = std::make_shared<rclcpp::Node>("periodicity_diagnostics_listener");
    std::optional<diagnostic_msgs::msg::DiagnosticStatus> received;
    auto sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS(),
      [&received, &name](const diagnostic_msgs::msg::DiagnosticArray & msg)
      {
        for (const auto & status : msg.status)
        {
          if (status.name.find(name) != std::string::npos)
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
  TestPeriodicityDiagnosticsWithWindow() : TestPeriodicityDiagnostics(WINDOW_DURATION) {}
};

TEST_F(TestPeriodicityDiagnostics, cm_without_window_a_startup_stall_keeps_the_diagnostic_in_error)
{
  stall();
  steady_cycles(3 * UPDATE_RATE);

  const auto status = next_status(CM_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(values(*status).count("periodicity.recent.standard_deviation"), 0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(
  TestPeriodicityDiagnosticsWithWindow, cm_with_window_the_level_recovers_after_a_startup_stall)
{
  stall();
  steady_cycles(3 * UPDATE_RATE);

  const auto status = next_status(CM_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;

  const auto v = values(*status);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.window_duration")), WINDOW_DURATION, 1e-9);
  EXPECT_GT(std::stoul(v.at("periodicity.recent.sample_count")), 0u);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.average")), UPDATE_RATE, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.standard_deviation")), 0.0, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.min")), UPDATE_RATE, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.recent.max")), UPDATE_RATE, 1e-3);

  // The since-startup statistics still show the stall
  EXPECT_NEAR(std::stod(v.at("periodicity.min")), 8.0, 1e-3);
  EXPECT_NEAR(std::stod(v.at("periodicity.max")), 10000.0, 1e-3);
  EXPECT_GT(std::stod(v.at("periodicity.standard_deviation")), 10.0);
}

TEST_F(TestPeriodicityDiagnosticsWithWindow, cm_with_window_a_recent_stall_is_still_an_error)
{
  steady_cycles(3 * UPDATE_RATE);
  stall();
  steady_cycles(UPDATE_RATE / 4);

  const auto status = next_status(CM_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controller Manager Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
}

TEST_F(
  TestPeriodicityDiagnostics,
  controller_without_window_a_startup_stall_keeps_the_diagnostic_in_error)
{
  const auto spec = add_active_controller();
  EXPECT_FALSE(spec.recent_periodicity_statistics->is_enabled());
  controller_stall_then_steady(spec);

  const auto status = next_status(CONTROLLERS_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controllers Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(
    values(*status).count(
      std::string(test_controller::TEST_CONTROLLER_NAME) + ".periodicity.recent"),
    0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(TestPeriodicityDiagnosticsWithWindow, controller_update_feeds_the_recent_periodicity)
{
  const auto spec = add_active_controller();
  ASSERT_TRUE(spec.recent_periodicity_statistics->is_enabled());
  EXPECT_DOUBLE_EQ(spec.recent_periodicity_statistics->get_window_duration(), WINDOW_DURATION);
  spec.periodicity_statistics->reset();
  spec.recent_periodicity_statistics->reset();

  // The controller periodicity is measured on the trigger clock, so let real time pass
  for (int i = 0; i < 10; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / CONTROLLER_UPDATE_RATE));
    time_ += rclcpp::Duration::from_seconds(1.0 / CONTROLLER_UPDATE_RATE);
    cm_->update(time_, rclcpp::Duration::from_seconds(1.0 / CONTROLLER_UPDATE_RATE));
  }
  // All the cycles are well within the window
  EXPECT_GT(spec.periodicity_statistics->get_count(), 0u);
  EXPECT_EQ(
    spec.recent_periodicity_statistics->get_statistics().sample_count,
    spec.periodicity_statistics->get_count());
}

TEST_F(
  TestPeriodicityDiagnosticsWithWindow,
  controller_with_window_the_level_recovers_after_a_startup_stall)
{
  const auto spec = add_active_controller();
  controller_stall_then_steady(spec);

  const auto status = next_status(CONTROLLERS_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controllers Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;
  const auto v = values(*status);
  const std::string name = test_controller::TEST_CONTROLLER_NAME;
  ASSERT_EQ(v.count(name + ".periodicity.recent"), 1u);
  EXPECT_THAT(v.at(name + ".periodicity.recent"), testing::HasSubstr("Avg: 50.00 [50.00 - 50.00]"));
  // The since-startup statistics still show the stall
  EXPECT_THAT(v.at(name + ".periodicity"), testing::HasSubstr("[8.00 - 10000.00]"));
}
