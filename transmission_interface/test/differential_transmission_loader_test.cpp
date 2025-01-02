// Copyright 2022 PAL Robotics S.L.
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
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "transmission_interface/differential_transmission.hpp"
#include "transmission_interface/transmission_loader.hpp"

using testing::DoubleNear;
using testing::SizeIs;

// Floating-point value comparison threshold
const double EPS = 1e-5;

class TransmissionPluginLoader
{
public:
  std::shared_ptr<transmission_interface::TransmissionLoader> create(const std::string & type)
  {
    try
    {
      return class_loader_.createUniqueInstance(type);
    }
    catch (std::exception & ex)
    {
      std::cerr << ex.what() << std::endl;
      return std::shared_ptr<transmission_interface::TransmissionLoader>();
    }
  }

private:
  // must keep it alive because instance destroyers need it
  pluginlib::ClassLoader<transmission_interface::TransmissionLoader> class_loader_ = {
    "transmission_interface", "transmission_interface::TransmissionLoader"};
};

TEST(DifferentialTransmissionLoaderTest, FullSpec)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>0.5</offset>
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>-0.5</offset>
            <mechanical_reduction>-2.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  transmission_interface::DifferentialTransmission * differential_transmission =
    dynamic_cast<transmission_interface::DifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != differential_transmission);

  const std::vector<double> & actuator_reduction =
    differential_transmission->get_actuator_reduction();
  EXPECT_THAT(50.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(-50.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction = differential_transmission->get_joint_reduction();
  EXPECT_THAT(2.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(-2.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = differential_transmission->get_joint_offset();
  EXPECT_THAT(0.5, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(-0.5, DoubleNear(joint_offset[1], EPS));
}

TEST(DifferentialTransmissionLoaderTest, only_mech_red_specified)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  transmission_interface::DifferentialTransmission * differential_transmission =
    dynamic_cast<transmission_interface::DifferentialTransmission *>(transmission.get());

  const std::vector<double> & actuator_reduction =
    differential_transmission->get_actuator_reduction();
  EXPECT_THAT(50.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(-50.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction = differential_transmission->get_joint_reduction();
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));
}

TEST(SimpleTransmissionLoaderTest, offset_and_mech_red_not_specified)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1"/>
          <actuator name="joint2_motor" role="actuator2"/>
          <joint name="joint1" role="joint1"/>
          <joint name="joint2" role="joint2"/>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  transmission_interface::DifferentialTransmission * differential_transmission =
    dynamic_cast<transmission_interface::DifferentialTransmission *>(transmission.get());

  const std::vector<double> & actuator_reduction =
    differential_transmission->get_actuator_reduction();
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction = differential_transmission->get_joint_reduction();
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));
}

TEST(DifferentialTransmissionLoaderTest, mechanical_reduction_not_a_number)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>one</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>two</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <mechanical_reduction>three</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <mechanical_reduction>four</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  transmission_interface::DifferentialTransmission * differential_transmission =
    dynamic_cast<transmission_interface::DifferentialTransmission *>(transmission.get());

  // default kicks in for ill-defined values
  const std::vector<double> & actuator_reduction =
    differential_transmission->get_actuator_reduction();
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction = differential_transmission->get_joint_reduction();
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));
}

TEST(DifferentialTransmissionLoaderTest, offset_ill_defined)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>two</offset>  <!-- Not a number -->
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>three</offset>  <!-- Not a number -->
            <mechanical_reduction>-2.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  transmission_interface::DifferentialTransmission * differential_transmission =
    dynamic_cast<transmission_interface::DifferentialTransmission *>(transmission.get());

  // default kicks in for ill-defined values
  const std::vector<double> & actuator_reduction =
    differential_transmission->get_actuator_reduction();
  EXPECT_THAT(50.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(-50.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction = differential_transmission->get_joint_reduction();
  EXPECT_THAT(2.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(-2.0, DoubleNear(joint_reduction[1], EPS));

  // default kicks in for ill-defined values
  const std::vector<double> & joint_offset = differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));
}

TEST(DifferentialTransmissionLoaderTest, mech_red_invalid_value)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) + R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>transmission_interface/DifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>0</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>2</offset>
            <mechanical_reduction>0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>3</offset>
            <mechanical_reduction>0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(nullptr == transmission);
}
