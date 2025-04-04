// Copyright 2021 ros2_control development team
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

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/gps_sensor.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

struct GPSSensorTest : public testing::Test
{
  GPSSensorTest()
  {
    std::transform(
      gps_interface_names.cbegin(), gps_interface_names.cend(),
      std::back_inserter(full_interface_names),
      [this](const auto & interface_name) { return gps_sensor_name + '/' + interface_name; });
    state_interface.emplace_back(gps_state);
    state_interface.emplace_back(gps_service);
    state_interface.emplace_back(latitude);
    state_interface.emplace_back(longitude);
    state_interface.emplace_back(altitude);
  }

  const std::string gps_sensor_name{"gps_sensor"};
  const std::array<std::string, 5> gps_interface_names{
    {"status", "service", "latitude", "longitude", "altitude"}};
  std::array<double, 5> gps_states{};
  semantic_components::GPSSensor<semantic_components::GPSSensorOption::WithoutCovariance> sut{
    gps_sensor_name};
  std::vector<std::string> full_interface_names;

  hardware_interface::StateInterface gps_state{
    gps_sensor_name, gps_interface_names.at(0), &gps_states.at(0)};
  hardware_interface::StateInterface gps_service{
    gps_sensor_name, gps_interface_names.at(1), &gps_states.at(1)};
  hardware_interface::StateInterface latitude{
    gps_sensor_name, gps_interface_names.at(2), &gps_states.at(2)};
  hardware_interface::StateInterface longitude{
    gps_sensor_name, gps_interface_names.at(3), &gps_states.at(3)};
  hardware_interface::StateInterface altitude{
    gps_sensor_name, gps_interface_names.at(4), &gps_states.at(4)};
  std::vector<hardware_interface::LoanedStateInterface> state_interface;
};

TEST_F(
  GPSSensorTest,
  interfaces_name_should_contain_status_latitude_longitude_altitude_and_sensor_name_prefix)
{
  const auto senors_interfaces_name = sut.get_state_interface_names();
  EXPECT_THAT(senors_interfaces_name, testing::ElementsAreArray(full_interface_names));
}

TEST_F(
  GPSSensorTest,
  status_latitude_longitude_altitude_should_be_equal_to_corresponding_values_in_state_interface)
{
  EXPECT_TRUE(sut.assign_loaned_state_interfaces(state_interface));
  EXPECT_EQ(gps_states.at(0), sut.get_status());
  EXPECT_EQ(gps_states.at(1), sut.get_service());
  EXPECT_DOUBLE_EQ(gps_states.at(2), sut.get_latitude());
  EXPECT_DOUBLE_EQ(gps_states.at(3), sut.get_longitude());
  EXPECT_DOUBLE_EQ(gps_states.at(4), sut.get_altitude());

  gps_states.at(0) = 1.0;
  gps_states.at(1) = 3.0;
  gps_states.at(2) = 2.0;
  gps_states.at(3) = 3.0;
  gps_states.at(4) = 4.0;

  EXPECT_EQ(gps_states.at(0), sut.get_status());
  EXPECT_EQ(gps_states.at(1), sut.get_service());
  EXPECT_DOUBLE_EQ(gps_states.at(2), sut.get_latitude());
  EXPECT_DOUBLE_EQ(gps_states.at(3), sut.get_longitude());
  EXPECT_DOUBLE_EQ(gps_states.at(4), sut.get_altitude());
}

TEST_F(GPSSensorTest, should_fill_gps_nav_sat_fix_msg_with_value_from_state_interface)
{
  EXPECT_TRUE(sut.assign_loaned_state_interfaces(state_interface));

  sensor_msgs::msg::NavSatFix message;
  EXPECT_TRUE(sut.get_values_as_message(message));
  EXPECT_EQ(gps_states.at(0), message.status.status);
  EXPECT_EQ(gps_states.at(1), message.status.service);
  EXPECT_DOUBLE_EQ(gps_states.at(2), message.latitude);
  EXPECT_DOUBLE_EQ(gps_states.at(3), message.longitude);
  EXPECT_DOUBLE_EQ(gps_states.at(4), message.altitude);

  gps_states.at(0) = 1.0;
  gps_states.at(1) = 3.0;
  gps_states.at(2) = 2.0;
  gps_states.at(3) = 3.0;
  gps_states.at(4) = 4.0;

  EXPECT_TRUE(sut.get_values_as_message(message));
  EXPECT_EQ(gps_states.at(0), message.status.status);
  EXPECT_EQ(gps_states.at(1), message.status.service);
  EXPECT_DOUBLE_EQ(gps_states.at(2), message.latitude);
  EXPECT_DOUBLE_EQ(gps_states.at(3), message.longitude);
  EXPECT_DOUBLE_EQ(gps_states.at(4), message.altitude);
}

struct GPSSensorWithCovarianceTest : public testing::Test
{
  GPSSensorWithCovarianceTest()
  {
    std::transform(
      gps_interface_names.cbegin(), gps_interface_names.cend(),
      std::back_inserter(full_interface_names),
      [this](const auto & interface_name) { return gps_sensor_name + '/' + interface_name; });
    state_interface.emplace_back(gps_state);
    state_interface.emplace_back(gps_service);
    state_interface.emplace_back(latitude);
    state_interface.emplace_back(longitude);
    state_interface.emplace_back(altitude);
    state_interface.emplace_back(latitude_covariance);
    state_interface.emplace_back(longitude_covariance);
    state_interface.emplace_back(altitude_covariance);
  }

  const std::string gps_sensor_name{"gps_sensor"};
  const std::array<std::string, 8> gps_interface_names{
    {"status", "service", "latitude", "longitude", "altitude", "latitude_covariance",
     "longitude_covariance", "altitude_covariance"}};
  std::array<double, 8> gps_states{};
  semantic_components::GPSSensor<semantic_components::GPSSensorOption::WithCovariance> sut{
    gps_sensor_name};
  std::vector<std::string> full_interface_names;

  hardware_interface::StateInterface gps_state{
    gps_sensor_name, gps_interface_names.at(0), &gps_states.at(0)};
  hardware_interface::StateInterface gps_service{
    gps_sensor_name, gps_interface_names.at(1), &gps_states.at(1)};
  hardware_interface::StateInterface latitude{
    gps_sensor_name, gps_interface_names.at(2), &gps_states.at(2)};
  hardware_interface::StateInterface longitude{
    gps_sensor_name, gps_interface_names.at(3), &gps_states.at(3)};
  hardware_interface::StateInterface altitude{
    gps_sensor_name, gps_interface_names.at(4), &gps_states.at(4)};
  hardware_interface::StateInterface latitude_covariance{
    gps_sensor_name, gps_interface_names.at(5), &gps_states.at(5)};
  hardware_interface::StateInterface longitude_covariance{
    gps_sensor_name, gps_interface_names.at(6), &gps_states.at(6)};
  hardware_interface::StateInterface altitude_covariance{
    gps_sensor_name, gps_interface_names.at(7), &gps_states.at(7)};
  std::vector<hardware_interface::LoanedStateInterface> state_interface;
};

TEST_F(
  GPSSensorWithCovarianceTest,
  interfaces_name_should_contain_status_latitude_longitude_altitude_and_sensor_name_prefix)
{
  const auto senors_interfaces_name = sut.get_state_interface_names();

  EXPECT_EQ(senors_interfaces_name.size(), full_interface_names.size());
  EXPECT_THAT(senors_interfaces_name, testing::ElementsAreArray(full_interface_names));
}

TEST_F(
  GPSSensorWithCovarianceTest,
  status_latitude_longitude_altitude_should_be_equal_to_corresponding_values_in_state_interface)
{
  EXPECT_TRUE(sut.assign_loaned_state_interfaces(state_interface));
  EXPECT_EQ(gps_states.at(0), sut.get_status());
  EXPECT_EQ(gps_states.at(1), sut.get_service());
  EXPECT_DOUBLE_EQ(gps_states.at(2), sut.get_latitude());
  EXPECT_DOUBLE_EQ(gps_states.at(3), sut.get_longitude());
  EXPECT_DOUBLE_EQ(gps_states.at(4), sut.get_altitude());
  EXPECT_THAT(
    sut.get_covariance(), testing::ElementsAreArray({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  gps_states.at(0) = 1.0;
  gps_states.at(1) = 3.0;
  gps_states.at(2) = 2.0;
  gps_states.at(3) = 3.0;
  gps_states.at(4) = 4.0;
  gps_states.at(5) = 0.5;
  gps_states.at(6) = 0.2;
  gps_states.at(7) = 0.7;

  EXPECT_EQ(gps_states.at(0), sut.get_status());
  EXPECT_EQ(gps_states.at(1), sut.get_service());
  EXPECT_DOUBLE_EQ(gps_states.at(2), sut.get_latitude());
  EXPECT_DOUBLE_EQ(gps_states.at(3), sut.get_longitude());
  EXPECT_DOUBLE_EQ(gps_states.at(4), sut.get_altitude());
  EXPECT_THAT(
    sut.get_covariance(), testing::ElementsAreArray({0.5, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.7}));
}

TEST_F(GPSSensorWithCovarianceTest, should_fill_gps_nav_sat_fix_msg_with_value_from_state_interface)
{
  EXPECT_TRUE(sut.assign_loaned_state_interfaces(state_interface));
  sensor_msgs::msg::NavSatFix message;
  EXPECT_TRUE(sut.get_values_as_message(message));
  EXPECT_EQ(gps_states.at(0), message.status.status);
  EXPECT_EQ(gps_states.at(1), message.status.service);
  EXPECT_DOUBLE_EQ(gps_states.at(2), message.latitude);
  EXPECT_DOUBLE_EQ(gps_states.at(3), message.longitude);
  EXPECT_DOUBLE_EQ(gps_states.at(4), message.altitude);
  EXPECT_THAT(
    message.position_covariance,
    testing::ElementsAreArray({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  gps_states.at(0) = 1.0;
  gps_states.at(1) = 2.0;
  gps_states.at(2) = 2.0;
  gps_states.at(3) = 3.0;
  gps_states.at(4) = 4.0;
  gps_states.at(5) = 0.5;
  gps_states.at(6) = 0.2;
  gps_states.at(7) = 0.7;

  EXPECT_TRUE(sut.get_values_as_message(message));
  EXPECT_EQ(gps_states.at(0), message.status.status);
  EXPECT_EQ(gps_states.at(1), message.status.service);
  EXPECT_DOUBLE_EQ(gps_states.at(2), message.latitude);
  EXPECT_DOUBLE_EQ(gps_states.at(3), message.longitude);
  EXPECT_DOUBLE_EQ(gps_states.at(4), message.altitude);
  EXPECT_THAT(
    message.position_covariance,
    testing::ElementsAreArray({0.5, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.7}));
}
