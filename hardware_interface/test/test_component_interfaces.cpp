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
#include "hardware_interface/component_info.hpp"
#include "hardware_interface/component_interfaces/joint_interface.hpp"
#include "hardware_interface/component_interfaces/sensor_interface.hpp"
#include "hardware_interface/component_interfaces/system_interface.hpp"
#include "hardware_interface/joint.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

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

class DummySensor : public SensorInterface
{
};

class DummySystem : public SystemInterface
{
};

}  // namespace component_interfaces_test
}  // namespace hardware_interface

using hardware_interface::ComponentInfo;
using hardware_interface::Joint;
using hardware_interface::JointInterface;
using hardware_interface::component_interfaces_test::DummyJoint;
using hardware_interface::return_type;

TEST(TestJointInterface, joint_interfce_works)
{
  std::unique_ptr<JointInterface> joint_interface(new DummyJoint);
  Joint joint(joint_interface);

  ComponentInfo joint_info;
  joint_info.name = "DummyJoint";
  joint_info.interface_names.push_back("dummy");
  joint_info.parameters["min_value"] = "-1";
  joint_info.parameters["max_value"] = "1";

  joint.configure(joint_info);
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
