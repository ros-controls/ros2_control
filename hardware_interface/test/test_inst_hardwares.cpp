// Copyright 2023 LAAS CNRS
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

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"

class TestInstantiationHardwares : public ::testing::Test
{
protected:
  static void SetUpTestCase() {}
};

TEST_F(TestInstantiationHardwares, build_actuator) { hardware_interface::Actuator anActuator; }

TEST_F(TestInstantiationHardwares, build_sensor) { hardware_interface::Sensor aSensor; }

TEST_F(TestInstantiationHardwares, build_system) { hardware_interface::System aSystem; }
