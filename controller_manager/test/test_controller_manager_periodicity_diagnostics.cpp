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
const char HARDWARE_ACTIVITY[] = "Hardware Components Activity";
// Two actuators at half the CM rate, so that their periodicity is diagnosed and loading the second
// one moves the first one within its container
const char HARDWARE_NAME[] = "TestActuatorHardware";
const char SECOND_HARDWARE_NAME[] = "TestActuatorHardware2";
constexpr int HARDWARE_RW_RATE = 50;
std::string actuator(const std::string & name, const std::string & joint)
{
  return R"(
  <ros2_control name=")" +
         name + R"(" type="actuator" rw_rate="50">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name=")" +
         joint + R"(">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <command_interface name="max_velocity" />
    </joint>
  </ros2_control>
)";
}
const auto HARDWARE_URDF =
  std::string(ros2_control_test_assets::urdf_head) + actuator(HARDWARE_NAME, "joint1") +
  actuator(SECOND_HARDWARE_NAME, "joint2") + ros2_control_test_assets::urdf_tail;
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

/// Gives the tests access to the hardware components statistics
class PeriodicityTestControllerManager : public controller_manager::ControllerManager
{
public:
  using controller_manager::ControllerManager::ControllerManager;

  const hardware_interface::HardwareComponentInfo & get_hardware_info(const std::string & name)
  {
    return resource_manager_->get_components_status().at(name);
  }
};
}  // namespace

class TestPeriodicityDiagnostics : public ControllerManagerFixture<PeriodicityTestControllerManager>
{
public:
  explicit TestPeriodicityDiagnostics(
    double window_duration = -1.0,
    const std::string & urdf = ros2_control_test_assets::minimal_robot_urdf)
  : ControllerManagerFixture<PeriodicityTestControllerManager>(
      urdf, "",
      {rclcpp::Parameter("update_rate", UPDATE_RATE),
       rclcpp::Parameter("diagnostics.controller_manager.window_duration", window_duration),
       rclcpp::Parameter("diagnostics.controllers.window_duration", window_duration),
       rclcpp::Parameter("diagnostics.hardware_components.window_duration", window_duration),
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

  /// Feed execution time statistics (in us) with a slow first cycle followed by 3 s of steady
  /// cycles, as when the first cycle warms up caches or initializes things lazily
  template <typename AddMeasurement>
  static void slow_first_cycle_then_steady(int rate, AddMeasurement add_measurement)
  {
    double t = 0.0;
    add_measurement(50000.0, t);
    for (int i = 0; i < 3 * rate; ++i)
    {
      t += 1.0 / rate;
      add_measurement(100.0, t);
    }
  }

  /// The execution time of the controller goes through a slow first update and is steady since
  static void controller_slow_first_update(const controller_manager::ControllerSpec & spec)
  {
    spec.periodicity_statistics->reset();
    spec.recent_periodicity_statistics->reset();
    spec.execution_time_statistics->reset();
    spec.recent_execution_time_statistics->reset();
    slow_first_cycle_then_steady(
      CONTROLLER_UPDATE_RATE,
      [&spec](double value, double t)
      {
        spec.execution_time_statistics->add_measurement(value);
        spec.recent_execution_time_statistics->add_measurement(value, t);
        // A steady periodicity, so that only the execution time can raise the level
        spec.periodicity_statistics->add_measurement(CONTROLLER_UPDATE_RATE);
        spec.recent_periodicity_statistics->add_measurement(CONTROLLER_UPDATE_RATE, t);
      });
  }

  /// The read cycle periodicity of the hardware goes through a stall at bring-up and is steady
  /// since
  void hardware_read_stall_then_steady()
  {
    const auto & statistics = cm_->get_hardware_info(HARDWARE_NAME).read_statistics;
    auto since_startup = std::make_shared<ros2_control::MovingAverageStatistics>();
    since_startup->reset();
    statistics->recent_periodicity->reset();
    stall_then_steady(
      HARDWARE_RW_RATE,
      [&](double value, double t)
      {
        since_startup->add_measurement(value);
        statistics->recent_periodicity->add_measurement(value, t);
      });
    statistics->periodicity.update_statistics(since_startup);
  }

  /// The read cycle execution time of the hardware goes through a slow first read and is steady
  /// since
  void hardware_slow_first_read()
  {
    const auto & statistics = cm_->get_hardware_info(HARDWARE_NAME).read_statistics;
    auto exec_time_since_startup = std::make_shared<ros2_control::MovingAverageStatistics>();
    auto periodicity_since_startup = std::make_shared<ros2_control::MovingAverageStatistics>();
    exec_time_since_startup->reset();
    periodicity_since_startup->reset();
    statistics->recent_execution_time->reset();
    statistics->recent_periodicity->reset();
    slow_first_cycle_then_steady(
      HARDWARE_RW_RATE,
      [&](double value, double t)
      {
        exec_time_since_startup->add_measurement(value);
        statistics->recent_execution_time->add_measurement(value, t);
        // A steady periodicity, so that only the execution time can raise the level
        periodicity_since_startup->add_measurement(HARDWARE_RW_RATE);
        statistics->recent_periodicity->add_measurement(HARDWARE_RW_RATE, t);
      });
    statistics->execution_time.update_statistics(exec_time_since_startup);
    statistics->periodicity.update_statistics(periodicity_since_startup);
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

class TestHardwarePeriodicityDiagnostics : public TestPeriodicityDiagnostics
{
public:
  TestHardwarePeriodicityDiagnostics() : TestPeriodicityDiagnostics(-1.0, HARDWARE_URDF) {}
};

class TestHardwarePeriodicityDiagnosticsWithWindow : public TestPeriodicityDiagnostics
{
public:
  TestHardwarePeriodicityDiagnosticsWithWindow()
  : TestPeriodicityDiagnostics(WINDOW_DURATION, HARDWARE_URDF)
  {
  }
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

TEST_F(
  TestPeriodicityDiagnostics,
  controller_without_window_a_slow_first_update_keeps_the_execution_time_in_error)
{
  const auto spec = add_active_controller();
  EXPECT_FALSE(spec.recent_execution_time_statistics->is_enabled());
  controller_slow_first_update(spec);

  const auto status = next_status(CONTROLLERS_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controllers Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(
    values(*status).count(
      std::string(test_controller::TEST_CONTROLLER_NAME) + ".execution_time.recent"),
    0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(
  TestPeriodicityDiagnosticsWithWindow,
  controller_with_window_the_execution_time_level_recovers_after_a_slow_first_update)
{
  const auto spec = add_active_controller();
  controller_slow_first_update(spec);

  const auto status = next_status(CONTROLLERS_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Controllers Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;
  const auto v = values(*status);
  const std::string name = test_controller::TEST_CONTROLLER_NAME;
  ASSERT_EQ(v.count(name + ".execution_time.recent"), 1u);
  EXPECT_THAT(
    v.at(name + ".execution_time.recent"), testing::HasSubstr("Avg: 100.00 [100.00 - 100.00] us"));
  // The since-activation statistics still show the slow first update
  EXPECT_THAT(v.at(name + ".execution_time"), testing::HasSubstr("[100.00 - 50000.00]"));
}

TEST_F(TestPeriodicityDiagnosticsWithWindow, controller_update_feeds_the_recent_statistics)
{
  const auto spec = add_active_controller();
  ASSERT_TRUE(spec.recent_periodicity_statistics->is_enabled());
  EXPECT_DOUBLE_EQ(spec.recent_periodicity_statistics->get_window_duration(), WINDOW_DURATION);
  ASSERT_TRUE(spec.recent_execution_time_statistics->is_enabled());
  EXPECT_DOUBLE_EQ(spec.recent_execution_time_statistics->get_window_duration(), WINDOW_DURATION);
  spec.periodicity_statistics->reset();
  spec.recent_periodicity_statistics->reset();
  spec.execution_time_statistics->reset();
  spec.recent_execution_time_statistics->reset();

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
  EXPECT_GT(spec.execution_time_statistics->get_count(), 0u);
  EXPECT_EQ(
    spec.recent_execution_time_statistics->get_statistics().sample_count,
    spec.execution_time_statistics->get_count());
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

TEST_F(
  TestHardwarePeriodicityDiagnostics,
  hardware_without_window_a_startup_stall_keeps_the_diagnostic_in_error)
{
  const auto & info = cm_->get_hardware_info(HARDWARE_NAME);
  ASSERT_EQ(info.rw_rate, static_cast<unsigned int>(HARDWARE_RW_RATE));
  ASSERT_TRUE(info.read_statistics->recent_periodicity);
  EXPECT_FALSE(info.read_statistics->recent_periodicity->is_enabled());
  hardware_read_stall_then_steady();

  const auto status = next_status(HARDWARE_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Hardware Components Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(
    values(*status).count(std::string(HARDWARE_NAME) + ".read_cycle.periodicity.recent"), 0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(
  TestHardwarePeriodicityDiagnostics,
  hardware_without_window_a_slow_first_read_keeps_the_execution_time_in_error)
{
  const auto & info = cm_->get_hardware_info(HARDWARE_NAME);
  ASSERT_TRUE(info.read_statistics->recent_execution_time);
  EXPECT_FALSE(info.read_statistics->recent_execution_time->is_enabled());
  hardware_slow_first_read();

  const auto status = next_status(HARDWARE_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Hardware Components Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << status->message;
  EXPECT_EQ(
    values(*status).count(std::string(HARDWARE_NAME) + ".read_cycle.execution_time.recent"), 0u)
    << "no recent statistics are reported when the window is disabled";
}

TEST_F(
  TestHardwarePeriodicityDiagnosticsWithWindow,
  hardware_with_window_the_execution_time_level_recovers_after_a_slow_first_read)
{
  hardware_slow_first_read();

  const auto status = next_status(HARDWARE_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Hardware Components Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;
  const auto v = values(*status);
  const std::string key = std::string(HARDWARE_NAME) + ".read_cycle.execution_time";
  ASSERT_EQ(v.count(key + ".recent"), 1u);
  EXPECT_THAT(v.at(key + ".recent"), testing::HasSubstr("Avg: 100.00 [100.00 - 100.00] us"));
  // The since-activation statistics still show the slow first read
  EXPECT_THAT(v.at(key), testing::HasSubstr("[100.00 - 50000.00]"));
}

TEST_F(
  TestHardwarePeriodicityDiagnosticsWithWindow, hardware_read_and_write_feed_the_recent_statistics)
{
  // The resource manager runs the hardware cycles on its own clock, so let real time pass
  const auto period = rclcpp::Duration::from_seconds(1.0 / UPDATE_RATE);
  for (int i = 0; i < UPDATE_RATE / 4; ++i)
  {
    std::this_thread::sleep_for(period.to_chrono<std::chrono::nanoseconds>());
    time_ += period;
    cm_->read(time_, period);
    cm_->write(time_, period);
  }
  // A quarter of a second of cycles, all within the window
  for (const std::string name : {HARDWARE_NAME, SECOND_HARDWARE_NAME})
  {
    const auto & info = cm_->get_hardware_info(name);
    for (const auto & statistics : {info.read_statistics, info.write_statistics})
    {
      ASSERT_TRUE(statistics->recent_periodicity) << name;
      EXPECT_DOUBLE_EQ(statistics->recent_periodicity->get_window_duration(), WINDOW_DURATION)
        << name;
      const auto recent = statistics->recent_periodicity->get_statistics();
      EXPECT_GT(recent.sample_count, 0u) << name;
      EXPECT_EQ(recent.sample_count, statistics->periodicity.get_statistics().sample_count) << name;

      ASSERT_TRUE(statistics->recent_execution_time) << name;
      EXPECT_DOUBLE_EQ(statistics->recent_execution_time->get_window_duration(), WINDOW_DURATION)
        << name;
      const auto recent_exec_time = statistics->recent_execution_time->get_statistics();
      EXPECT_GT(recent_exec_time.sample_count, 0u) << name;
      EXPECT_EQ(
        recent_exec_time.sample_count, statistics->execution_time.get_statistics().sample_count)
        << name;
    }
  }
}

TEST_F(
  TestHardwarePeriodicityDiagnosticsWithWindow,
  hardware_with_window_the_level_recovers_after_a_startup_stall)
{
  hardware_read_stall_then_steady();

  const auto status = next_status(HARDWARE_ACTIVITY);
  ASSERT_TRUE(status.has_value()) << "no Hardware Components Activity diagnostic was published";
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << status->message;
  const auto v = values(*status);
  const std::string key = std::string(HARDWARE_NAME) + ".read_cycle.periodicity";
  ASSERT_EQ(v.count(key + ".recent"), 1u);
  EXPECT_THAT(v.at(key + ".recent"), testing::HasSubstr("Avg: 50.00 [50.00 - 50.00]"));
  // The since-activation statistics still show the stall
  EXPECT_THAT(v.at(key), testing::HasSubstr("[8.00 - 10000.00]"));
}
