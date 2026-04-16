// Copyright 2024 ros2_control Development Team
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

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::ResourceManager;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

class ResourceManagerExceptionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("ResourceManagerExceptionTestNode");
  }

  std::shared_ptr<rclcpp::Node> node_;

  std::string get_exception_robot_urdf(const std::string & param_name)
  {
    std::string urdf = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="ExceptionRobot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="0.2"/>
  </joint>
  <link name="link1"/>
  <joint name="joint2" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="0.2"/>
  </joint>
  <link name="link2"/>
  <ros2_control name="ExceptionSystem" type="system">
    <hardware>
      <plugin>test_system</plugin>
      <param name=")" + param_name +
                       R"(">true</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="velocity"/>
      <command_interface name="max_acceleration"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
  </ros2_control>
</robot>
)";
    return urdf;
  }
};

TEST_F(ResourceManagerExceptionTest, catch_read_exception)
{
  std::string urdf = get_exception_robot_urdf("throw_on_read");

  // Use the 5-argument constructor with node interfaces to ensure parsing success
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSystem", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionSystem", active_state), return_type::OK);

  // Read should throw, be caught, and transition component to UNCONFIGURED
  auto [result, failed_hardware] =
    rm.read(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));

  EXPECT_EQ(result, return_type::ERROR);
  ASSERT_EQ(failed_hardware.size(), 1u);
  EXPECT_EQ(failed_hardware[0], "ExceptionSystem");

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSystem"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerExceptionTest, catch_write_exception)
{
  std::string urdf = get_exception_robot_urdf("throw_on_write");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSystem", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionSystem", active_state), return_type::OK);

  // Write should throw, be caught, and transition component to UNCONFIGURED
  auto [result, failed_hardware] =
    rm.write(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));

  EXPECT_EQ(result, return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSystem"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerExceptionTest, catch_configure_exception)
{
  std::string urdf = get_exception_robot_urdf("throw_on_configure");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);

  // on_configure throws, caught by RM → component stays UNCONFIGURED
  EXPECT_EQ(rm.set_component_state("ExceptionSystem", inactive_state), return_type::ERROR);

  auto status = rm.get_components_status();
  // After configure exception the state is not advanced — stays UNCONFIGURED
  EXPECT_EQ(
    status["ExceptionSystem"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerExceptionTest, catch_activate_exception)
{
  std::string urdf = get_exception_robot_urdf("throw_on_activate");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSystem", inactive_state), return_type::OK);

  // on_activate throws, caught by activate_hardware in RM. The exception propagates before
  // HardwareComponent::activate() can record the new state, so the component stays INACTIVE.
  EXPECT_EQ(rm.set_component_state("ExceptionSystem", active_state), return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSystem"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

class ResourceManagerActuatorExceptionTest : public ResourceManagerExceptionTest
{
protected:
  std::string get_exception_actuator_urdf(const std::string & param_name)
  {
    std::string urdf = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="ExceptionRobot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="0.2"/>
  </joint>
  <link name="link1"/>
  <ros2_control name="ExceptionActuator" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
      <param name=")" + param_name +
                       R"(">true</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";
    return urdf;
  }
};

TEST_F(ResourceManagerActuatorExceptionTest, catch_read_exception)
{
  std::string urdf = get_exception_actuator_urdf("throw_on_read");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionActuator", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionActuator", active_state), return_type::OK);

  auto [result, failed_hardware] =
    rm.read(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));

  EXPECT_EQ(result, return_type::ERROR);
  ASSERT_EQ(failed_hardware.size(), 1u);
  EXPECT_EQ(failed_hardware[0], "ExceptionActuator");

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionActuator"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerActuatorExceptionTest, catch_write_exception)
{
  std::string urdf = get_exception_actuator_urdf("throw_on_write");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionActuator", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionActuator", active_state), return_type::OK);

  auto [result, failed_hardware] =
    rm.write(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));

  EXPECT_EQ(result, return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionActuator"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerActuatorExceptionTest, catch_configure_exception)
{
  std::string urdf = get_exception_actuator_urdf("throw_on_configure");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionActuator", inactive_state), return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionActuator"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerActuatorExceptionTest, catch_activate_exception)
{
  std::string urdf = get_exception_actuator_urdf("throw_on_activate");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionActuator", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionActuator", active_state), return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionActuator"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

class ResourceManagerSensorExceptionTest : public ResourceManagerExceptionTest
{
protected:
  std::string get_exception_sensor_urdf(const std::string & param_name)
  {
    std::string urdf = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="ExceptionRobot">
  <link name="base_link"/>
  <ros2_control name="ExceptionSensor" type="sensor">
    <hardware>
      <plugin>test_sensor</plugin>
      <param name=")" + param_name +
                       R"(">true</param>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="velocity"/>
    </sensor>
  </ros2_control>
</robot>
)";
    return urdf;
  }
};

TEST_F(ResourceManagerSensorExceptionTest, catch_read_exception)
{
  std::string urdf = get_exception_sensor_urdf("throw_on_read");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSensor", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionSensor", active_state), return_type::OK);

  auto [result, failed_hardware] =
    rm.read(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));

  EXPECT_EQ(result, return_type::ERROR);
  ASSERT_EQ(failed_hardware.size(), 1u);
  EXPECT_EQ(failed_hardware[0], "ExceptionSensor");

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSensor"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerSensorExceptionTest, catch_configure_exception)
{
  std::string urdf = get_exception_sensor_urdf("throw_on_configure");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSensor", inactive_state), return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSensor"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

TEST_F(ResourceManagerSensorExceptionTest, catch_activate_exception)
{
  std::string urdf = get_exception_sensor_urdf("throw_on_activate");
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  EXPECT_EQ(rm.set_component_state("ExceptionSensor", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("ExceptionSensor", active_state), return_type::ERROR);

  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["ExceptionSensor"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);
}

// --- Nominal Usage Tests to reach 100% coverage and verify non-interference ---

class ResourceManagerNominalAllComponentsTest : public ResourceManagerExceptionTest
{
protected:
  std::string get_nominal_robot_all_components_urdf()
  {
    return R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="NominalRobot">
  <link name="base_link"/>
  <joint name="sys_joint1" type="revolute">
    <parent link="base_link"/><child link="sys_link1"/><limit effort="1" lower="-1" upper="1" velocity="1"/>
  </joint>
  <link name="sys_link1"/>
  <joint name="sys_joint2" type="revolute">
    <parent link="sys_link1"/><child link="sys_link2"/><limit effort="1" lower="-1" upper="1" velocity="1"/>
  </joint>
  <link name="sys_link2"/>
  <joint name="act_joint1" type="revolute">
    <parent link="base_link"/><child link="act_link1"/><limit effort="1" lower="-1" upper="1" velocity="1"/>
  </joint>
  <link name="act_link1"/>

  <ros2_control name="NominalSystem" type="system">
    <hardware><plugin>test_system</plugin></hardware>
    <joint name="sys_joint1">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="acceleration"/>
    </joint>
    <joint name="sys_joint2">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="acceleration"/>
    </joint>
  </ros2_control>

  <ros2_control name="NominalActuator" type="actuator">
    <hardware><plugin>test_actuator</plugin></hardware>
    <joint name="act_joint1">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <ros2_control name="NominalSensor" type="sensor">
    <hardware><plugin>test_sensor</plugin></hardware>
    <sensor name="sensor1"><state_interface name="velocity"/></sensor>
  </ros2_control>
</robot>
)";
  }
};

TEST_F(ResourceManagerNominalAllComponentsTest, validate_nominal_behavior)
{
  std::string urdf = get_nominal_robot_all_components_urdf();
  ResourceManager rm(
    urdf, node_->get_node_clock_interface(), node_->get_node_logging_interface(), true, 100);

  rclcpp_lifecycle::State inactive_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Configure and Activate all
  EXPECT_EQ(rm.set_component_state("NominalSystem", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("NominalSystem", active_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("NominalActuator", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("NominalActuator", active_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("NominalSensor", inactive_state), return_type::OK);
  EXPECT_EQ(rm.set_component_state("NominalSensor", active_state), return_type::OK);

  // Perform read/write cycle
  {
    auto [result, failed_hardware] =
      rm.read(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));
    EXPECT_EQ(result, return_type::OK);
  }
  {
    auto [result, failed_hardware] =
      rm.write(node_->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(10)));
    EXPECT_EQ(result, return_type::OK);
  }

  // Verify all are ACTIVE
  auto status = rm.get_components_status();
  EXPECT_EQ(
    status["NominalSystem"].state.label(), hardware_interface::lifecycle_state_names::ACTIVE);
  EXPECT_EQ(
    status["NominalActuator"].state.label(), hardware_interface::lifecycle_state_names::ACTIVE);
  EXPECT_EQ(
    status["NominalSensor"].state.label(), hardware_interface::lifecycle_state_names::ACTIVE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
