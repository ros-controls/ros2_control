// Copyright (c) 2023, FZI Forschungszentrum Informatik
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
//
/// \author: Felix Exner <exner@fzi.de>

#ifndef ROS2_CONTROL_TEST_ASSETS__TEST_HARDWARE_INTERFACE_CONSTANTS_HPP_
#define ROS2_CONTROL_TEST_ASSETS__TEST_HARDWARE_INTERFACE_CONSTANTS_HPP_
namespace test_constants
{
/// Constants defining special values used inside tests to trigger things like deactivate or errors
/// on read/write.
constexpr double READ_FAIL_VALUE = 28282828.0;
constexpr double WRITE_FAIL_VALUE = 23232323.0;
constexpr double READ_DEACTIVATE_VALUE = 29292929.0;
constexpr double WRITE_DEACTIVATE_VALUE = 24242424.0;
}  // namespace test_constants
#endif  // ROS2_CONTROL_TEST_ASSETS__TEST_HARDWARE_INTERFACE_CONSTANTS_HPP_
