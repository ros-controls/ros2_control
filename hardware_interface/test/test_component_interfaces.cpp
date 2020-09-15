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
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ::testing;  // NOLINT

namespace hardware_interface
{

namespace hardware_interfaces_components_test
{

class DummyPositionJoint : public components::Joint
{
public:
  return_type configure(const components::ComponentInfo & joint_info)
  {
    if (Joint::configure(joint_info) != return_type::OK) {
      return return_type::ERROR;
    }

    if (info_.command_interfaces.size() > 1 || info_.state_interfaces.size() > 1) {
      return return_type::ERROR;
    }
    if (info_.command_interfaces.size() == 0) {
      info_.command_interfaces.push_back(HW_IF_POSITION);
      commands_.resize(1);
    }
    if (info_.state_interfaces.size() == 0) {
      info_.state_interfaces.push_back(HW_IF_POSITION);
      states_.resize(1);
    }

    max_position_ = stod(info_.parameters["max_position"]);
    min_position_ = stod(info_.parameters["min_position"]);
    return return_type::OK;
  }

private:
  double max_position_, min_position_;
};

class DummyMultiJoint : public components::Joint
{
public:
  return_type configure(const components::ComponentInfo & joint_info)
  {
    if (Joint::configure(joint_info) != return_type::OK) {
      return return_type::ERROR;
    }

    if (info_.command_interfaces.size() < 2) {
      return return_type::ERROR;
    }

    max_position_ = stod(info_.parameters["max_position"]);
    min_position_ = stod(info_.parameters["min_position"]);
    max_velocity_ = stod(info_.parameters["max_velocity"]);
    min_velocity_ = stod(info_.parameters["min_velocity"]);
    return return_type::OK;
  }

private:
  double max_position_, min_position_;
  double max_velocity_, min_velocity_;
};

class DummyForceTorqueSensor : public components::Sensor
{
public:
  return_type configure(const components::ComponentInfo & sensor_info)
  {
    if (Sensor::configure(sensor_info) != return_type::OK) {
      return return_type::ERROR;
    }

    if (info_.parameters["frame_id"] == "") {
      return return_type::ERROR;
    }
    if (info_.state_interfaces.size() == 0) {
      info_.state_interfaces.push_back("force_x");
      info_.state_interfaces.push_back("force_y");
      info_.state_interfaces.push_back("force_z");
      info_.state_interfaces.push_back("torque_x");
      info_.state_interfaces.push_back("torque_y");
      info_.state_interfaces.push_back("torque_z");
    }
    states_ = {1.34, 5.67, 8.21, 5.63, 5.99, 4.32};
    return return_type::OK;
  }
};

class DummyActuatorHardware : public ActuatorHardwareInterface
{
  return_type configure(const HardwareInfo & actuator_info) override
  {
    info_ = actuator_info;
    hw_read_time_ = stod(info_.hardware_parameters["example_param_read_for_sec"]);
    hw_write_time_ = stod(info_.hardware_parameters["example_param_write_for_sec"]);
    status_ = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status_ == hardware_interface_status::CONFIGURED ||
      status_ == hardware_interface_status::STOPPED)
    {
      status_ = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status_ == hardware_interface_status::STARTED) {
      status_ = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status_;
  }

  return_type read_joint(std::shared_ptr<components::Joint> joint) const override
  {
    std::vector<std::string> interfaces = joint->get_state_interfaces();
    return joint->set_state(hw_values_, interfaces);
  }

  return_type write_joint(const std::shared_ptr<components::Joint> joint) override
  {
    std::vector<std::string> interfaces = joint->get_command_interfaces();
    return joint->get_command(hw_values_, interfaces);
  }

private:
  HardwareInfo info_;
  hardware_interface_status status_ = hardware_interface_status::UNKNOWN;
  std::vector<double> hw_values_ = {1.2};
  double hw_read_time_, hw_write_time_;
};

class DummySensorHardware : public SensorHardwareInterface
{
  return_type configure(const HardwareInfo & sensor_info) override
  {
    info_ = sensor_info;
    binary_to_voltage_factor_ = stod(info_.hardware_parameters["binary_to_voltage_factor"]);
    status_ = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status_ == hardware_interface_status::CONFIGURED ||
      status_ == hardware_interface_status::STOPPED)
    {
      status_ = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status_ == hardware_interface_status::STARTED) {
      status_ = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status_;
  }

  return_type read_sensors(const std::vector<std::shared_ptr<components::Sensor>> & sensors) const
  override
  {
    return_type ret = return_type::OK;
    for (const auto & sensor : sensors) {
      ret = sensor->set_state(ft_hw_values_);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

private:
  HardwareInfo info_;
  hardware_interface_status status_ = hardware_interface_status::UNKNOWN;
  double binary_to_voltage_factor_;
  std::vector<double> ft_hw_values_ = {1, -1.0, 3.4, 7.9, 5.5, 4.4};
};

class DummySystemHardware : public SystemHardwareInterface
{
  return_type configure(const HardwareInfo & system_info) override
  {
    info_ = system_info;
    api_version_ = stod(info_.hardware_parameters["example_api_version"]);
    hw_read_time_ = stod(info_.hardware_parameters["example_param_read_for_sec"]);
    hw_write_time_ = stod(info_.hardware_parameters["example_param_write_for_sec"]);
    status_ = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  return_type start() override
  {
    if (status_ == hardware_interface_status::CONFIGURED ||
      status_ == hardware_interface_status::STOPPED)
    {
      status_ = hardware_interface_status::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (status_ == hardware_interface_status::STARTED) {
      status_ = hardware_interface_status::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  hardware_interface_status get_status() const override
  {
    return status_;
  }

  return_type read_sensors(std::vector<std::shared_ptr<components::Sensor>> & sensors) const
  override
  {
    return_type ret = return_type::OK;
    for (const auto & sensor : sensors) {
      ret = sensor->set_state(ft_hw_values_);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

  return_type read_joints(std::vector<std::shared_ptr<components::Joint>> & joints) const override
  {
    return_type ret = return_type::OK;
    std::vector<std::string> interfaces;
    std::vector<double> joint_values;
    for (uint i = 0; i < joints.size(); i++) {
      joint_values.clear();
      joint_values.push_back(joints_hw_values_[i]);
      interfaces = joints[i]->get_state_interfaces();
      ret = joints[i]->set_state(joint_values, interfaces);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

  return_type write_joints(const std::vector<std::shared_ptr<components::Joint>> & joints) override
  {
    return_type ret = return_type::OK;
    for (const auto & joint : joints) {
      std::vector<double> values;
      std::vector<std::string> interfaces = joint->get_command_interfaces();
      ret = joint->get_command(values, interfaces);
      if (ret != return_type::OK) {
        break;
      }
    }
    return ret;
  }

private:
  HardwareInfo info_;
  hardware_interface_status status_;
  double hw_write_time_, hw_read_time_, api_version_;
  std::vector<double> ft_hw_values_ = {-3.5, -2.1, -8.7, -5.4, -9.0, -11.2};
  std::vector<double> joints_hw_values_ = {-1.575, -0.7543};
};

}  // namespace hardware_interfaces_components_test
}  // namespace hardware_interface

using hardware_interface::components::ComponentInfo;
using hardware_interface::components::Joint;
using hardware_interface::components::Sensor;
using hardware_interface::ActuatorHardware;
using hardware_interface::ActuatorHardwareInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::hardware_interface_status;
using hardware_interface::return_type;
using hardware_interface::SensorHardware;
using hardware_interface::SensorHardwareInterface;
using hardware_interface::SystemHardware;
using hardware_interface::SystemHardwareInterface;
using hardware_interface::hardware_interfaces_components_test::DummyForceTorqueSensor;
using hardware_interface::hardware_interfaces_components_test::DummyMultiJoint;
using hardware_interface::hardware_interfaces_components_test::DummyPositionJoint;

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
    joint_info.parameters["max_position"] = "3.14";
    joint_info.parameters["min_position"] = "-3.14";

    sensor_info.name = "DummyForceTorqueSensor";
    sensor_info.parameters["frame_id"] = "tcp_link";
  }
};

TEST_F(TestComponentInterfaces, joint_example_component_works)
{
  DummyPositionJoint joint;

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
  ASSERT_THAT(joint.get_command_interfaces(), SizeIs(1));
  EXPECT_EQ(joint.get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(1));
  EXPECT_EQ(joint.get_state_interfaces()[0], hardware_interface::HW_IF_POSITION);

  // Command getters and setters
  std::vector<std::string> interfaces;
  std::vector<double> input;
  input.push_back(2.1);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  input.clear();
  input.push_back(1.2);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::OK);

  std::vector<double> output;
  interfaces.clear();
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.2);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::INTERFACE_NOT_FOUND);

  input.clear();
  EXPECT_EQ(joint.set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input.push_back(2.1);
  EXPECT_EQ(joint.set_command(input), return_type::OK);

  EXPECT_EQ(joint.get_command(output), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 2.1);

  // State getters and setters
  interfaces.clear();
  input.clear();
  input.push_back(2.1);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  input.clear();
  input.push_back(1.2);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::OK);

  output.clear();
  interfaces.clear();
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint.get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.2);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.get_state(output, interfaces), return_type::INTERFACE_NOT_FOUND);

  input.clear();
  EXPECT_EQ(joint.set_state(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input.push_back(2.1);
  EXPECT_EQ(joint.set_state(input), return_type::OK);

  EXPECT_EQ(joint.get_state(output), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 2.1);

  // Test DummyPositionJoint
  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.configure(joint_info), return_type::ERROR);
}

TEST_F(TestComponentInterfaces, multi_joint_example_component_works)
{
  DummyMultiJoint joint;

  joint_info.name = "DummyMultiJoint";
  joint_info.parameters["max_position"] = "3.14";
  joint_info.parameters["min_position"] = "-3.14";
  joint_info.parameters["max_velocity"] = "1.14";
  joint_info.parameters["min_velocity"] = "-1.14";

  EXPECT_EQ(joint.configure(joint_info), return_type::ERROR);

  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  joint_info.command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);

  ASSERT_THAT(joint.get_command_interfaces(), SizeIs(2));
  EXPECT_EQ(joint.get_command_interfaces()[0], hardware_interface::HW_IF_POSITION);
  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(0));

  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_POSITION);
  joint_info.state_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
  ASSERT_THAT(joint.get_state_interfaces(), SizeIs(2));
  EXPECT_EQ(joint.get_command_interfaces()[1], hardware_interface::HW_IF_VELOCITY);

  // Command getters and setters
  std::vector<std::string> interfaces;
  std::vector<double> input;
  input.push_back(2.1);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  input.clear();
  input.push_back(1.02);
  EXPECT_EQ(joint.set_command(input, interfaces), return_type::OK);

  std::vector<double> output;
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.02);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(joint.get_command(output, interfaces), return_type::INTERFACE_NOT_FOUND);

  input.clear();
  EXPECT_EQ(joint.set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input.push_back(5.77);
  EXPECT_EQ(joint.set_command(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input.clear();
  input.push_back(1.2);
  input.push_back(0.4);
  EXPECT_EQ(joint.set_command(input), return_type::OK);

  EXPECT_EQ(joint.get_command(output), return_type::OK);
  ASSERT_THAT(output, SizeIs(2));
  EXPECT_EQ(output[1], 0.4);

  // State getters and setters
  interfaces.clear();
  input.clear();
  input.push_back(2.1);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_POSITION);
  input.clear();
  input.push_back(1.2);
  EXPECT_EQ(joint.set_state(input, interfaces), return_type::OK);

  output.clear();
  EXPECT_EQ(joint.get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.2);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  EXPECT_EQ(joint.get_state(output, interfaces), return_type::INTERFACE_NOT_FOUND);

  input.clear();
  EXPECT_EQ(joint.set_state(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input.push_back(2.1);
  input.push_back(1.02);
  EXPECT_EQ(joint.set_state(input), return_type::OK);

  EXPECT_EQ(joint.get_state(output), return_type::OK);
  ASSERT_THAT(output, SizeIs(2));
  EXPECT_EQ(output[0], 2.1);
}

TEST_F(TestComponentInterfaces, sensor_example_component_works)
{
  DummyForceTorqueSensor sensor;

  EXPECT_EQ(sensor.configure(sensor_info), return_type::OK);
  ASSERT_THAT(sensor.get_state_interfaces(), SizeIs(6));
  EXPECT_EQ(sensor.get_state_interfaces()[0], "force_x");
  EXPECT_EQ(sensor.get_state_interfaces()[5], "torque_z");
  std::vector<double> input = {5.23, 6.7, 2.5, 3.8, 8.9, 12.3};
  std::vector<double> output;
  std::vector<std::string> interfaces;
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back("force_y");
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 5.67);

  // State getters and setters
  interfaces.clear();
  EXPECT_EQ(sensor.set_state(input, interfaces), return_type::INTERFACE_NOT_PROVIDED);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(sensor.set_state(input, interfaces), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(sensor.set_state(input, interfaces), return_type::INTERFACE_NOT_FOUND);
  interfaces.clear();
  interfaces = sensor.get_state_interfaces();
  EXPECT_EQ(sensor.set_state(input, interfaces), return_type::OK);

  output.clear();
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(6));
  EXPECT_EQ(output[0], 5.23);
  interfaces.clear();
  interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(sensor.get_state(output, interfaces), return_type::INTERFACE_NOT_FOUND);

  input.clear();
  EXPECT_EQ(sensor.set_state(input), return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL);
  input = {5.23, 6.7, 2.5, 3.8, 8.9, 12.3};
  EXPECT_EQ(sensor.set_state(input), return_type::OK);

  EXPECT_EQ(sensor.get_state(output), return_type::OK);
  ASSERT_THAT(output, SizeIs(6));
  EXPECT_EQ(output[5], 12.3);

  sensor_info.parameters.clear();
  EXPECT_EQ(sensor.configure(sensor_info), return_type::ERROR);
}

TEST_F(TestComponentInterfaces, actuator_hardware_interface_works)
{
  ActuatorHardware actuator_hw(std::make_unique<DummyActuatorHardware>());
  auto joint = std::make_shared<DummyPositionJoint>();

  HardwareInfo actuator_hw_info;
  actuator_hw_info.name = "DummyActuatorHardware";
  actuator_hw_info.hardware_parameters["example_param_write_for_sec"] = "2";
  actuator_hw_info.hardware_parameters["example_param_read_for_sec"] = "3";

  EXPECT_EQ(joint->configure(joint_info), return_type::OK);

  EXPECT_EQ(actuator_hw.configure(actuator_hw_info), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::CONFIGURED);
  EXPECT_EQ(actuator_hw.start(), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::STARTED);
  EXPECT_EQ(actuator_hw.read_joint(joint), return_type::OK);
  std::vector<std::string> interfaces = joint->get_state_interfaces();
  std::vector<double> output;
  EXPECT_EQ(joint->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], 1.2);
  EXPECT_EQ(interfaces[0], hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(actuator_hw.write_joint(joint), return_type::OK);
  EXPECT_EQ(actuator_hw.stop(), return_type::OK);
  EXPECT_EQ(actuator_hw.get_status(), hardware_interface_status::STOPPED);
}

TEST_F(TestComponentInterfaces, sensor_interface_with_hardware_works)
{
  SensorHardware sensor_hw(std::make_unique<DummySensorHardware>());
  auto sensor = std::make_shared<DummyForceTorqueSensor>();

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
  std::vector<std::string> interfaces = sensor->get_state_interfaces();
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
  auto joint1 = std::make_shared<DummyPositionJoint>();
  auto joint2 = std::make_shared<DummyPositionJoint>();
  std::vector<std::shared_ptr<Joint>> joints;
  joints.push_back(joint1);
  joints.push_back(joint2);

  std::shared_ptr<DummyForceTorqueSensor> sensor(std::make_shared<DummyForceTorqueSensor>());
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
  std::vector<std::string> interfaces = sensor->get_state_interfaces();
  EXPECT_EQ(sensor->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(6));
  EXPECT_EQ(output[2], -8.7);
  ASSERT_THAT(interfaces, SizeIs(6));
  EXPECT_EQ(interfaces[4], "torque_y");
  output.clear();
  interfaces.clear();

  EXPECT_EQ(system.read_joints(joints), return_type::OK);
  interfaces = joint1->get_command_interfaces();
  EXPECT_EQ(joint1->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], -1.575);
  ASSERT_THAT(interfaces, SizeIs(1));
  EXPECT_EQ(interfaces[0], hardware_interface::HW_IF_POSITION);
  output.clear();
  interfaces.clear();
  interfaces = joint2->get_state_interfaces();
  EXPECT_EQ(joint2->get_state(output, interfaces), return_type::OK);
  ASSERT_THAT(output, SizeIs(1));
  EXPECT_EQ(output[0], -0.7543);
  ASSERT_THAT(interfaces, SizeIs(1));
  EXPECT_EQ(interfaces[0], hardware_interface::HW_IF_POSITION);

  EXPECT_EQ(system.write_joints(joints), return_type::OK);

  EXPECT_EQ(system.stop(), return_type::OK);
  EXPECT_EQ(system.get_status(), hardware_interface_status::STOPPED);
}
