// Copyright 2020 ros2_control development team
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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/joint.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

using namespace ::testing;  // NOLINT

namespace hardware_interface
{

namespace hardware_interfaces_components_test
{

class DummyPositionJoint : public Joint
{
public:
  return_type configure(const ComponentInfo & joint_info) override
  {
    info = joint_info;
    if (info.command_interfaces.size() > 1 || info.state_interfaces.size() > 1) {
      return return_type::ERROR;
    }
    if (info.command_interfaces.size() == 0) {
      info.command_interfaces.push_back("position");
    }
    if (info.state_interfaces.size() == 0) {
      info.state_interfaces.push_back("position");
    }
    max_position = stod(info.parameters["max_position"]);
    min_position = stod(info.parameters["min_position"]);
    return return_type::OK;
  }

  std::vector<std::string> get_command_interfaces() const override
  {
    return info.command_interfaces;
  }

  std::vector<std::string> get_state_interfaces() const override
  {
    return info.state_interfaces;
  }

  return_type set_command(
    const std::vector<double> command,
    std::vector<std::string> interfaces = std::vector<std::string>()) override
  {
    if (interfaces.size() != 0) {
      return return_type::ERROR;
    }
    position_command = command[0];
    return return_type::OK;
  }

  return_type get_command(
    std::vector<double> & command,
    std::vector<std::string> & interfaces) const override
  {
    command.push_back(position_command);
    interfaces = info.command_interfaces;
    return return_type::OK;
  }

  return_type set_state(
    const std::vector<double> & state,
    std::vector<std::string> interfaces = std::vector<std::string>()) override
  {
    return_type ret = return_type::OK;
    if (interfaces.size() != 0) {
      ret = return_type::ERROR;
    }
    if (state[0] > min_position && state[0] < max_position) {
      position_state = state[0];
    } else {
      ret = return_type::ERROR;
    }
    return ret;
  }

  return_type get_state(
    std::vector<double> & state,
    std::vector<std::string> & interfaces) const override
  {
    state.push_back(position_state);
    interfaces = info.command_interfaces;
    return return_type::OK;
  }

private:
  ComponentInfo info;
  double position_command;
  double position_state;
  double max_position, min_position;
};

class DummyForceTorqueSensor : public Sensor
{
public:
  return_type configure(const ComponentInfo & sensor_info) override
  {
    info = sensor_info;
    if (info.parameters["frame_id"] == "") {
      return return_type::ERROR;
    }
    if (info.state_interfaces.size() == 0) {
      info.state_interfaces.push_back("force_x");
      info.state_interfaces.push_back("force_y");
      info.state_interfaces.push_back("force_z");
      info.state_interfaces.push_back("torque_x");
      info.state_interfaces.push_back("torque_y");
      info.state_interfaces.push_back("torque_z");
    }
    ft_values.resize(6);
    ft_values = {1.34, 5.67, 8.21, 5.63, 5.99, 4.32};
    return return_type::OK;
  }

  std::vector<std::string> get_state_interfaces() const override
  {
    return info.state_interfaces;
  }

  return_type get_state(
    std::vector<double> & state,
    std::vector<std::string> & interfaces) const override
  {
    interfaces = info.state_interfaces;
    state = ft_values;
    return return_type::OK;
  }

  return_type set_state(
    const std::vector<double> & state,
    std::vector<std::string> interfaces) override
  {
    if (interfaces.size() != 0) {
      return return_type::ERROR;
    }
    ft_values = state;
    return return_type::OK;
  }

private:
  ComponentInfo info;
  std::vector<double> ft_values;
};

class DummyActuatorHardware : public ActuatorHardwareInterface
{
  return_type configure(const HardwareInfo & actuator_info) override
  {
    info = actuator_info;
    hw_read_time = stod(info.hardware_parameters["example_param_read_for_sec"]);
    hw_write_time = stod(info.hardware_parameters["example_param_write_for_sec"]);
    status = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status == hardware_interface_status::CONFIGURED ||
      status == hardware_interface_status::STOPPED)
    {
      status = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status == hardware_interface_status::STARTED) {
      status = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status;
  }

  return_type read_joint(Joint & joint) const override
  {
    return joint.set_state(hw_values);
  }

  return_type write_joint(const Joint & joint) override
  {
    return joint.get_command(hw_values, interfaces);
  }

private:
  HardwareInfo info;
  hardware_interface_status status = hardware_interface_status::UNKNOWN;
  std::vector<double> hw_values = {1.2};
  std::vector<std::string> interfaces;
  double hw_read_time, hw_write_time;
};

class DummySensorHardware : public SensorHardwareInterface
{
  return_type configure(const HardwareInfo & sensor_info) override
  {
    info = sensor_info;
    binary_to_voltage_factor = stod(info.hardware_parameters["binary_to_voltage_factor"]);
    status = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status == hardware_interface_status::CONFIGURED ||
      status == hardware_interface_status::STOPPED)
    {
      status = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status == hardware_interface_status::STARTED) {
      status = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status;
  }

  return_type read_sensors(const std::vector<std::shared_ptr<Sensor>> & sensors) const override
  {
    return_type ret = return_type::OK;
    for (const auto & sensor : sensors) {
      ret = sensor->set_state(ft_hw_values);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

private:
  HardwareInfo info;
  hardware_interface_status status = hardware_interface_status::UNKNOWN;
  double binary_to_voltage_factor;
  std::vector<double> ft_hw_values = {1, -1.0, 3.4, 7.9, 5.5, 4.4};
};

class DummySystemHardware : public SystemHardwareInterface
{
  return_type configure(const HardwareInfo & system_info) override
  {
    info = system_info;
    api_version = stod(info.hardware_parameters["example_api_version"]);
    hw_read_time = stod(info.hardware_parameters["example_param_read_for_sec"]);
    hw_write_time = stod(info.hardware_parameters["example_param_write_for_sec"]);
    status = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status == hardware_interface_status::CONFIGURED ||
      status == hardware_interface_status::STOPPED)
    {
      status = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status == hardware_interface_status::STARTED) {
      status = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status;
  }

  return_type read_sensors(std::vector<std::shared_ptr<Sensor>> & sensors) const override
  {
    return_type ret = return_type::OK;
    for (const auto & sensor : sensors) {
      ret = sensor->set_state(ft_hw_values);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

  return_type read_joints(std::vector<std::shared_ptr<Joint>> & joints) const override
  {
    return_type ret = return_type::OK;
    std::vector<double> joint_values;
    for (uint i = 0; i < joints.size(); i++) {
      joint_values.clear();
      joint_values.push_back(joints_hw_values[i]);
      ret = joints[i]->set_state(joint_values);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

  return_type write_joints(const std::vector<std::shared_ptr<Joint>> & joints) override
  {
    return_type ret = return_type::OK;
    for (const auto & joint : joints) {
      std::vector<double> values;
      std::vector<std::string> interfaces;
      ret = joint->set_state(values, interfaces);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

private:
  HardwareInfo info;
  hardware_interface_status status;
  double hw_write_time, hw_read_time, api_version;
  std::vector<double> ft_hw_values = {-3.5, -2.1, -8.7, -5.4, -9.0, -11.2};
  std::vector<double> joints_hw_values = {-1.575, -0.7543};
};

}  // namespace hardware_interfaces_components_test
}  // namespace hardware_interface

using hardware_interface::return_type;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::ActuatorHardware;
using hardware_interface::ActuatorHardwareInterface;
using hardware_interface::Joint;
using hardware_interface::Sensor;
using hardware_interface::SensorHardware;
using hardware_interface::SensorHardwareInterface;
using hardware_interface::SystemHardware;
using hardware_interface::SystemHardwareInterface;
using hardware_interface::hardware_interface_status;

using hardware_interface::hardware_interfaces_components_test::DummyPositionJoint;
using hardware_interface::hardware_interfaces_components_test::DummyForceTorqueSensor;

using hardware_interface::hardware_interfaces_components_test::DummyActuatorHardware;
using hardware_interface::hardware_interfaces_components_test::DummySensorHardware;
using hardware_interface::hardware_interfaces_components_test::DummySystemHardware;

class TestComponentInterfaces : public Test
{
protected:
  ComponentInfo joint_info;
  ComponentInfo sensor_info;

  void SetUp() override
  {
    joint_info.name = "DummyPositionJoint";
    joint_info.parameters["max_position"] = "1.742";
    joint_info.parameters["min_position"] = "-1.742";

    sensor_info.name = "DummyForceTorqueSensor";
    sensor_info.parameters["frame_id"] = "tcp_link";
  }
};

TEST_F(TestComponentInterfaces, joint_example_component_works)
{
  DummyPositionJoint joint;

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
  ASSERT_THAT(joint.get_command_interfaces(), SizeIs(1));
  EXPECT_EQ(joint.get_command_interfaces()[0], "position");
  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(1));
  EXPECT_EQ(joint.get_state_interfaces()[0], "position");
  std::vector<std::string> interfaces;
  std::vector<double> input;
  input.push_back(2.1);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::OK);
  std::vector<double> output;
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 2.1);
  ASSERT_THAT(interfaces, SizeIs(1));
  EXPECT_EQ(interfaces[0], "position");

  joint_info.command_interfaces.push_back("position");
  joint_info.command_interfaces.push_back("velocity");
  EXPECT_EQ(joint.configure(joint_info), return_type::ERROR);
}

TEST_F(TestComponentInterfaces, sensor_example_component_works)
{
  DummyForceTorqueSensor sensor;

  EXPECT_EQ(sensor.configure(sensor_info), return_type::OK);
  ASSERT_THAT(sensor.get_state_interfaces(), SizeIs(6));
  EXPECT_EQ(sensor.get_state_interfaces()[0], "force_x");
  EXPECT_EQ(sensor.get_state_interfaces()[5], "torque_z");
  std::vector<double> input = {5, 6.7, 2.5, 3.8, 8.9, 12.3};
  std::vector<double> output;
  std::vector<std::string> interfaces;
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::OK);
  EXPECT_EQ(output[1], 5.67);
  ASSERT_THAT(interfaces, SizeIs(6));
  EXPECT_EQ(interfaces[0], "force_x");
  interfaces.clear();
  EXPECT_EQ(sensor.set_state(input, interfaces), return_type::OK);
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::OK);
  EXPECT_EQ(output[5], 12.3);

  sensor_info.parameters.clear();
  EXPECT_EQ(sensor.configure(sensor_info), return_type::ERROR);
}

TEST_F(TestComponentInterfaces, actuator_hardware_interface_works)
{
  ActuatorHardware actuator_hw(std::make_unique<DummyActuatorHardware>());
  DummyPositionJoint joint;

  HardwareInfo actuator_hw_info;
  actuator_hw_info.name = "DummyActuatorHardware";
  actuator_hw_info.hardware_parameters["example_param_write_for_sec"] = "2";
  actuator_hw_info.hardware_parameters["example_param_read_for_sec"] = "3";

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);

  EXPECT_EQ(actuator_hw.configure(actuator_hw_info), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::CONFIGURED);
  EXPECT_EQ(actuator_hw.start(), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::STARTED);
  EXPECT_EQ(actuator_hw.read_joint(joint), return_type::OK);
  std::vector<std::string> interfaces;
  std::vector<double> output;
  EXPECT_EQ(joint.get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.2);
  EXPECT_EQ(interfaces[0], "position");
  EXPECT_EQ(actuator_hw.write_joint(joint), return_type::OK);
  EXPECT_EQ(actuator_hw.stop(), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::STOPPED);
}

TEST_F(TestComponentInterfaces, sensor_interface_with_hardware_works)
{
  SensorHardware sensor_hw(std::make_unique<DummySensorHardware>());
  std::shared_ptr<DummyForceTorqueSensor> sensor(new DummyForceTorqueSensor);

  HardwareInfo sensor_hw_info;
  sensor_hw_info.name = "DummySensor";
  sensor_hw_info.hardware_parameters["binary_to_voltage_factor"] = "0.0048828125";

  EXPECT_EQ(sensor->configure(sensor_info), return_type::OK);

  EXPECT_EQ(sensor_hw.configure(sensor_hw_info), return_type::OK);
  EXPECT_EQ(sensor_hw.get_status(), hardware_interface_status::CONFIGURED);
  EXPECT_EQ(sensor_hw.start(), return_type::OK);
  EXPECT_EQ(sensor_hw.get_status(), hardware_interface_status::STARTED);
  std::vector<std::shared_ptr<Sensor>> sensors;
  sensors.push_back(sensor);
  EXPECT_EQ(sensor_hw.read_sensors(sensors), return_type::OK);
  std::vector<double> output;
  std::vector<std::string> interfaces;
  EXPECT_EQ(sensor->get_state(output, interfaces), return_type::OK);
  EXPECT_EQ(output[2], 3.4);
  ASSERT_THAT(interfaces, SizeIs(6));
  EXPECT_EQ(interfaces[1], "force_y");
  EXPECT_EQ(sensor_hw.stop(), return_type::OK);
  EXPECT_EQ(sensor_hw.get_status(), hardware_interface_status::STOPPED);
  EXPECT_EQ(sensor_hw.start(), return_type::OK);
}

TEST_F(TestComponentInterfaces, system_interface_with_hardware_works)
{
  SystemHardware system(std::make_unique<DummySystemHardware>());
  std::shared_ptr<DummyPositionJoint> joint1(new DummyPositionJoint());
  std::shared_ptr<DummyPositionJoint> joint2(new DummyPositionJoint());
  std::vector<std::shared_ptr<Joint>> joints;
  joints.push_back(joint1);
  joints.push_back(joint2);

  std::shared_ptr<DummyForceTorqueSensor> sensor(new DummyForceTorqueSensor);
  std::vector<std::shared_ptr<Sensor>> sensors;
  sensors.push_back(sensor);

  EXPECT_EQ(joint1->configure(joint_info), return_type::OK);
  EXPECT_EQ(joint2->configure(joint_info), return_type::OK);
  EXPECT_EQ(sensor->configure(sensor_info), return_type::OK);

  HardwareInfo system_hw_info;
  system_hw_info.name = "DummyActuatorHardware";
  system_hw_info.hardware_parameters["example_api_version"] = "1.1";
  system_hw_info.hardware_parameters["example_param_write_for_sec"] = "2";
  system_hw_info.hardware_parameters["example_param_read_for_sec"] = "3";

  EXPECT_EQ(system.configure(system_hw_info), return_type::OK);
  EXPECT_EQ(system.get_status(), hardware_interface_status::CONFIGURED);
  EXPECT_EQ(system.start(), return_type::OK);
  EXPECT_EQ(system.get_status(), hardware_interface_status::STARTED);

  EXPECT_EQ(system.read_sensors(sensors), return_type::OK);
  std::vector<double> output;
  std::vector<std::string> interfaces;
  EXPECT_EQ(sensor->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(6));
  EXPECT_EQ(output[2], -8.7);
  ASSERT_THAT(interfaces, SizeIs(6));
  EXPECT_EQ(interfaces[4], "torque_y");
  output.clear();
  interfaces.clear();

  EXPECT_EQ(system.read_joints(joints), return_type::OK);
  EXPECT_EQ(joint1->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], -1.575);
  ASSERT_THAT(interfaces, SizeIs(1));
  EXPECT_EQ(interfaces[0], "position");
  output.clear();
  interfaces.clear();
  EXPECT_EQ(joint2->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], -0.7543);
  ASSERT_THAT(interfaces, SizeIs(1));
  EXPECT_EQ(interfaces[0], "position");

  EXPECT_EQ(system.stop(), return_type::OK);
  EXPECT_EQ(system.get_status(), hardware_interface_status::STOPPED);
}
