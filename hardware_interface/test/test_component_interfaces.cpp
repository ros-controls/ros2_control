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
#include "hardware_interface/component_info.hpp"
#include "hardware_interface/component_interfaces/joint_interface.hpp"
#include "hardware_interface/component_interfaces/sensor_interface.hpp"
#include "hardware_interface/component_interfaces/system_interface.hpp"
#include "hardware_interface/joint.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_state_values.hpp"

namespace hardware_interface
{
namespace component_interfaces_test
{
class DummyJoint : public JointInterface
{
  return_type configure(const ComponentInfo & joint_info) override
  {
    info = joint_info;
    min_value = stod(info.parameters["min_value"]);
    max_value = stod(info.parameters["max_value"]);
    state = component_state::CONFIGURED;
    return return_type::OK;
  }

  std::string get_interface_name() const override
  {
    return info.interface_names.front();
  }

  return_type start() override
  {
    if (state == component_state::CONFIGURED || state == component_state::STOPPED) {
      state = component_state::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (state == component_state::STARTED) {
      state = component_state::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  component_state get_state() const override
  {
    return state;
  }

  return_type read(double & data) override
  {
    data = joint_value;
    return return_type::OK;
  }

  return_type write(const double & data) override
  {
    if (data > min_value && data < max_value) {
      joint_value = data;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

private:
  ComponentInfo info;
  component_state state = component_state::UNKNOWN;
  double min_value, max_value;
  double joint_value;
};

class DummySensorHardware
{
public:
  DummySensorHardware() = default;

  ~DummySensorHardware() = default;

  return_type configure_hardware(std::unordered_map<std::string, std::string> & parameters)
  {
    if (parameters.count("start_value") > 0) {
      value = stoi(parameters["start_value"]);
      return return_type::OK;
    }
    return return_type::ERROR;
  }

  return_type start_hardware()
  {
    return return_type::OK;
  }

  return_type stop_hardware()
  {
    return return_type::OK;
  }

  return_type read_from_hardware(int & data)
  {
    data = value + 128;
    return return_type::OK;
  }

private:
  int value = 0;
};

class DummySensor : public SensorInterface
{
  return_type configure(const ComponentInfo & sensor_info) override
  {
    return_type ret;
    info = sensor_info;
    binary_to_voltage_factor = stod(info.parameters["binary_to_voltage_factor"]);

    // Normaly crate dynamically SensorHardware Object from sensor hardware_class_type
    if (hardware.configure_hardware(info.hardware_parameters) == return_type::OK) {
      state = component_state::CONFIGURED;
      ret = return_type::OK;
    } else {
      state = component_state::UNKNOWN;
      ret = return_type::ERROR;
    }
    return ret;
  }

  std::string get_interface_name() const override
  {
    return info.interface_names.front();
  }

  return_type start() override
  {
    if (state == component_state::CONFIGURED || state == component_state::STOPPED) {
      if (hardware.start_hardware() == return_type::OK) {
        state = component_state::STARTED;
      }
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (state == component_state::STARTED) {
      if (hardware.stop_hardware() == return_type::OK) {
        state = component_state::STOPPED;
      }
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  component_state get_state() const override
  {
    return state;
  }

  return_type read(double & data) override
  {
    int hw_data;
    hardware.read_from_hardware(hw_data);
    data = hw_data * binary_to_voltage_factor;
    return return_type::OK;
  }

private:
  ComponentInfo info;
  component_state state = component_state::UNKNOWN;
  DummySensorHardware hardware;
  double binary_to_voltage_factor;
};

class DummySystem : public SystemInterface
{
  return_type configure(const ComponentInfo & system_info) override
  {
    info = system_info;
    n_dof = info.joints.size();
    min_value.resize(n_dof);
    max_value.resize(n_dof);
    std::size_t current, previous = 0;
    const char delim = ',';
    for (uint i = 0; i < n_dof; i++) {
      current = info.parameters["min_value"].find(delim, previous);
      min_value[i] = stod(info.parameters["min_value"].substr(previous, current - previous));
      previous = current + 1;
    }
    for (uint i = 0; i < n_dof; i++) {
      current = info.parameters["max_value"].find(delim, previous);
      max_value[i] = stod(info.parameters["max_value"].substr(previous, current - previous));
      previous = current + 1;
    }
    joint_value.resize(n_dof);
    state = component_state::CONFIGURED;
    return return_type::OK;
  }

  std::vector<std::string> get_interface_names() const override
  {
    return info.interface_names;
  }

  return_type start() override
  {
    if (state == component_state::CONFIGURED || state == component_state::STOPPED) {
      state = component_state::STARTED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (state == component_state::STARTED) {
      state = component_state::STOPPED;
    } else {
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  component_state get_state() const override
  {
    return state;
  }

  return_type read(std::vector<double> & data) override
  {
    data = joint_value;
    return return_type::OK;
  }

  return_type write(const std::vector<double> & data) override
  {
    return_type ret = return_type::OK;
    if (data.size() == n_dof) {
      for (uint i = 0; i < n_dof; i++) {
        if (data[i] > min_value[i] && data[i] < max_value[i]) {
          joint_value[i] = data[i] / 2;
        } else {
          ret = return_type::ERROR;
        }
      }
    } else {
      ret = return_type::ERROR;
    }
    return ret;
  }

private:
  ComponentInfo info;
  component_state state;
  uint n_dof;
  std::vector<double> min_value, max_value;
  std::vector<double> joint_value;
};

}  // namespace component_interfaces_test
}  // namespace hardware_interface

using hardware_interface::return_type;
using hardware_interface::ComponentInfo;
using hardware_interface::Joint;
using hardware_interface::JointInterface;
using hardware_interface::Sensor;
using hardware_interface::SensorInterface;
using hardware_interface::System;
using hardware_interface::SystemInterface;

using hardware_interface::component_interfaces_test::DummyJoint;
using hardware_interface::component_interfaces_test::DummySensor;
using hardware_interface::component_interfaces_test::DummySystem;
using hardware_interface::component_state;

TEST(TestJointInterface, joint_interface_works)
{
  Joint joint(std::make_unique<DummyJoint>());

  ComponentInfo joint_info;
  joint_info.name = "DummyJoint";
  joint_info.interface_names.push_back("dummy");
  joint_info.parameters["min_value"] = "-1";
  joint_info.parameters["max_value"] = "1";

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
  EXPECT_EQ(joint.get_interface_name(), "dummy");
  EXPECT_EQ(joint.get_state(), component_state::CONFIGURED);
  EXPECT_EQ(joint.start(), return_type::OK);
  EXPECT_EQ(joint.get_state(), component_state::STARTED);
  EXPECT_EQ(joint.write(2), return_type::ERROR);
  EXPECT_EQ(joint.write(0.5), return_type::OK);
  double output = 0.0;
  EXPECT_EQ(joint.read(output), return_type::OK);
  EXPECT_DOUBLE_EQ(output, 0.5);
  EXPECT_EQ(joint.stop(), return_type::OK);
  EXPECT_EQ(joint.get_state(), component_state::STOPPED);
}

TEST(TestSensorInterfaceWithHardware, sensor_interface_with_hardware_works)
{
  Sensor sensor(std::make_unique<DummySensor>());

  ComponentInfo sensor_info;
  sensor_info.name = "DummySensor";
  sensor_info.interface_names.push_back("dummySensor");
  sensor_info.parameters["binary_to_voltage_factor"] = "0.0048828125";
  sensor_info.hardware_class_type = "DummySensorHardware";

  EXPECT_EQ(sensor.configure(sensor_info), return_type::ERROR);
  EXPECT_EQ(sensor.get_state(), component_state::UNKNOWN);

  sensor_info.hardware_parameters["start_value"] = "128";

  EXPECT_EQ(sensor.configure(sensor_info), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::CONFIGURED);
  EXPECT_EQ(sensor.get_interface_name(), "dummySensor");
  EXPECT_EQ(sensor.start(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STARTED);
  EXPECT_EQ(sensor.stop(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STOPPED);
  EXPECT_EQ(sensor.start(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STARTED);
  double output = 0.0;
  EXPECT_EQ(sensor.read(output), return_type::OK);
  EXPECT_DOUBLE_EQ(output, 0.0048828125 * (128 + 128));
}

TEST(TestJointInterface, system_interface_works)
{
  System system(std::make_unique<DummySystem>());

  ComponentInfo system_info;
  system_info.name = "DummyJoint";
  system_info.interface_names.push_back("dummy_position");
  system_info.interface_names.push_back("dummy_velocity");
  system_info.joints.push_back("joint1");
  system_info.joints.push_back("joint2");
  system_info.parameters["min_value"] = "0, 0";
  system_info.parameters["max_value"] = "4, 4";

  EXPECT_EQ(system.configure(system_info), return_type::OK);
  EXPECT_EQ(system.get_interface_names()[0], "dummy_position");
  EXPECT_EQ(system.get_interface_names()[1], "dummy_velocity");
  EXPECT_EQ(system.get_state(), component_state::CONFIGURED);
  EXPECT_EQ(system.start(), return_type::OK);
  EXPECT_EQ(system.get_state(), component_state::STARTED);
  std::vector<double> input;
  input.push_back(2);
  EXPECT_EQ(system.write(input), return_type::ERROR);
  input.push_back(3);
  EXPECT_EQ(system.write(input), return_type::OK);
  std::vector<double> output;
  output.push_back(0.0);
  output.push_back(0.0);
  EXPECT_EQ(system.read(output), return_type::OK);
  EXPECT_DOUBLE_EQ(output[0], 1.0);
  EXPECT_DOUBLE_EQ(output[1], 1.5);
  EXPECT_EQ(system.stop(), return_type::OK);
  EXPECT_EQ(system.get_state(), component_state::STOPPED);
}
