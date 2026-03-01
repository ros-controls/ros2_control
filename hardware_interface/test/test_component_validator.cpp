// Copyright 2025 ros2_control Development Team
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
#include <libxml2/libxml/parser.h>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "hardware_interface/component_validator.hpp"

using namespace ::testing;  // NOLINT

class TestComponentValidator : public Test
{
protected:
  std::string valid_xml_file_path;
  std::string valid_xml_file_path_with_tag;
  std::string valid_xml_file_path_with_web_tag;
  std::string invalid_xml_file_path;
  std::string xsd_file_path;
  void SetUp() override
  {
    // Use ament_index_cpp to get the package share directory
    std::string urdf_package_share_dir =
      ament_index_cpp::get_package_share_directory("ros2_control_test_assets");
    std::string xsd_package_share_dir =
      ament_index_cpp::get_package_share_directory("hardware_interface");
    valid_xml_file_path = urdf_package_share_dir + "/urdf/test_hardware_components.urdf";
    valid_xml_file_path_with_tag =
      urdf_package_share_dir + "/urdf/test_hardware_components_xsd_file.urdf";
    valid_xml_file_path_with_web_tag =
      urdf_package_share_dir + "/urdf/test_hardware_components_xsd_web.urdf";
    invalid_xml_file_path =
      urdf_package_share_dir + "/urdf/test_hardware_components_with_error.urdf";
    xsd_file_path = xsd_package_share_dir + "/schema/ros2_control.xsd";
  }
};

using hardware_interface::validate_urdf_file_path_with_xsd;
using hardware_interface::validate_urdf_file_with_xsd_tag;
using hardware_interface::validate_urdf_with_xsd;

TEST_F(TestComponentValidator, DryRun)
{
  SUCCEED();  // Minimal test to allow compilation and DRY test
}

TEST_F(TestComponentValidator, empty_string_throws_error)
{
  ASSERT_THROW(validate_urdf_with_xsd("", xsd_file_path), std::runtime_error);
}

TEST_F(TestComponentValidator, empty_urdf_throws_error)
{
  const std::string empty_urdf =
    "<?xml version=\"1.0\"?><robot name=\"robot\" xmlns:xacro=\"http://www.ros.org\"></robot>";

  ASSERT_TRUE(validate_urdf_with_xsd(
    empty_urdf, xsd_file_path));  // TODO(Sachin): discuss if should use throw error
}

TEST_F(TestComponentValidator, validate_valid_urdf_with_xsd)
{
  ASSERT_TRUE(validate_urdf_file_path_with_xsd(valid_xml_file_path, xsd_file_path));
}

TEST_F(TestComponentValidator, validate_invalid_urdf_with_xsd)
{
  ASSERT_FALSE(validate_urdf_file_path_with_xsd(invalid_xml_file_path, xsd_file_path));
}

TEST_F(TestComponentValidator, validate_valid_urdf_including_xsd_file_tag)
{
  ASSERT_TRUE(validate_urdf_file_with_xsd_tag(valid_xml_file_path_with_tag));
}

TEST_F(TestComponentValidator, validate_valid_urdf_including_xsd_web_tag)
{
  ASSERT_FALSE(validate_urdf_file_with_xsd_tag(
    valid_xml_file_path_with_web_tag));  // TODO(Sachin): Update the test to assert true when xsd
                                         // file is uploaded to web server
}
