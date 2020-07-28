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
    return return_type::OK;
  }

  std::string get_interface_name() const override
  {
    return info.interface_names.front();
  }

  return_type start() override
  {
    started = true;
    return return_type::OK;
  }

  return_type stop() override
  {
    started = false;
    return return_type::OK;
  }

  bool is_started() const override
  {
    return started;
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

  double min_value, max_value;

  double joint_value;
  bool started;
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

  return_type initalize_hardware(bool auto_start)
  {
    (void) auto_start;
    return return_type::OK;
  }

  return_type recover_hardware(bool auto_start)
  {
    (void) auto_start;
    return return_type::OK;
  }

  return_type start_hardware()
  {
    return return_type::OK;
  }

  return_type stop_hardware()
  {
    return return_type::OK;
  }

  return_type halt_hardware()
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

  return_type initalize(bool auto_start) override
  {
    if (hardware.initalize_hardware(auto_start) == return_type::OK) {
      if (auto_start) {
        start();
      } else {
        state = component_state::INITIALIZED;
      }
    }
    return return_type::OK;
  }

  return_type recover(bool auto_start) override
  {
    if (hardware.recover_hardware(auto_start) == return_type::OK) {
      if (auto_start) {
        start();
      } else {
        state = component_state::INITIALIZED;
      }
    }
    return return_type::OK;
  }

  return_type start() override
  {
    if (hardware.start_hardware() == return_type::OK) {
      state = component_state::STARTED;
    }
    return return_type::OK;
  }

  return_type stop() override
  {
    if (hardware.stop_hardware() == return_type::OK) {
      state = component_state::STOPPED;
    }
    return return_type::OK;
  }

  return_type halt() override
  {
    if (hardware.halt_hardware() == return_type::OK) {
      state = component_state::HALTED;
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
};

}  // namespace component_interfaces_test
}  // namespace hardware_interface

using hardware_interface::return_type;
using hardware_interface::ComponentInfo;
using hardware_interface::Joint;
using hardware_interface::JointInterface;
using hardware_interface::Sensor;
using hardware_interface::SensorInterface;

using hardware_interface::component_interfaces_test::DummyJoint;
using hardware_interface::component_interfaces_test::DummySensor;
using hardware_interface::component_state;

TEST(TestJointInterface, joint_interfce_works)
{
  std::unique_ptr<JointInterface> joint_interface(new DummyJoint);
  Joint joint(joint_interface);

  ComponentInfo joint_info;
  joint_info.name = "DummyJoint";
  joint_info.interface_names.push_back("dummy");
  joint_info.parameters["min_value"] = "-1";
  joint_info.parameters["max_value"] = "1";

  EXPECT_EQ(joint.configure(joint_info), return_type::OK);
  EXPECT_EQ(joint.get_interface_name(), "dummy");
  EXPECT_EQ(joint.is_started(), false);
  EXPECT_EQ(joint.start(), return_type::OK);
  EXPECT_EQ(joint.is_started(), true);
  EXPECT_EQ(joint.stop(), return_type::OK);
  EXPECT_EQ(joint.write(2), return_type::ERROR);
  EXPECT_EQ(joint.write(0.5), return_type::OK);
  double output;
  EXPECT_EQ(joint.read(output), return_type::OK);
  EXPECT_DOUBLE_EQ(output, 0.5);
}

TEST(TestSensorInterfaceWithHardware, sensor_interface_with_hardware_works)
{
  std::unique_ptr<SensorInterface> sensor_interface(new DummySensor);
  Sensor sensor(sensor_interface);

  ComponentInfo sensor_info;
  sensor_info.name = "DummySensor";
  sensor_info.interface_names.push_back("dummyS");
  sensor_info.parameters["binary_to_voltage_factor"] = "0.0048828125";
  sensor_info.hardware_class_type = "DummySensorHardware";

  EXPECT_EQ(sensor.configure(sensor_info), return_type::ERROR);
  EXPECT_EQ(sensor.get_state(), component_state::UNKNOWN);

  sensor_info.hardware_parameters["start_value"] = "128";

  EXPECT_EQ(sensor.configure(sensor_info), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::CONFIGURED);
  EXPECT_EQ(sensor.get_interface_name(), "dummyS");
  EXPECT_EQ(sensor.initalize(false), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::INITIALIZED);
  EXPECT_EQ(sensor.initalize(true), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STARTED);
  EXPECT_EQ(sensor.stop(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STOPPED);
  EXPECT_EQ(sensor.start(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STARTED);
  EXPECT_EQ(sensor.halt(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::HALTED);
  EXPECT_EQ(sensor.recover(false), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::INITIALIZED);
  EXPECT_EQ(sensor.start(), return_type::OK);
  EXPECT_EQ(sensor.get_state(), component_state::STARTED);
  double output;
  EXPECT_EQ(sensor.read(output), return_type::OK);
  EXPECT_DOUBLE_EQ(output, 0.0048828125 * (128 + 128));
}
