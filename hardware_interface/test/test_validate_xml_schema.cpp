// Copyright 2025 ros2_control development team
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
#include <gtest/gtest.h>
#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

class XmlParser
{
public:
  XmlParser(const std::string & xmlFilePath, const std::string & xsdFilePath)
  : doc(nullptr), schema(nullptr), schemaCtx(nullptr)
  {
    this->xmlFile = xmlFilePath;
    this->xsdFile = xsdFilePath;
  }

  ~XmlParser()
  {
    if (doc)
    {
      xmlFreeDoc(doc);
    }
    if (schemaCtx)
    {
      xmlSchemaFreeParserCtxt(schemaCtx);
    }
    if (schema)
    {
      xmlSchemaFree(schema);
    }
    xmlCleanupParser();
  }

  bool parseAndValidate()
  {
    doc = xmlReadFile(xmlFile.c_str(), nullptr, 0);
    if (!doc)
    {
      return false;
    }

    schemaCtx = xmlSchemaNewParserCtxt(xsdFile.c_str());
    if (!schemaCtx)
    {
      return false;
    }
    schema = xmlSchemaParse(schemaCtx);
    if (!schema)
    {
      return false;
    }

    xmlSchemaValidCtxtPtr validCtx = xmlSchemaNewValidCtxt(schema);
    if (!validCtx)
    {
      return false;
    }
    int ret = xmlSchemaValidateDoc(validCtx, doc);
    xmlSchemaFreeValidCtxt(validCtx);

    return ret == 0;
  }

private:
  std::string xmlFile;
  std::string xsdFile;
  xmlDocPtr doc;
  xmlSchemaPtr schema;
  xmlSchemaParserCtxtPtr schemaCtx;
};

// Test fixture for XML schema validation
class XmlSchemaValidationTest : public ::testing::Test
{
protected:
  std::string valid_xml;
  std::string invalid_xml;
  std::string xsd;

  void SetUp() override
  {
    // Use ament_index_cpp to get the package share directory
    std::string urdf_package_share_dir =
      ament_index_cpp::get_package_share_directory("ros2_control_test_assets");
    std::string xsd_package_share_dir =
      ament_index_cpp::get_package_share_directory("hardware_interface");
    valid_xml = urdf_package_share_dir + "/urdf/test_hardware_components.urdf";
    invalid_xml = urdf_package_share_dir + "/urdf/test_hardware_components_with_error.urdf";
    xsd = xsd_package_share_dir + "/schema/ros2_control.xsd";
  }
};

TEST_F(XmlSchemaValidationTest, ValidXmlPasses)
{
  XmlParser parser(valid_xml, xsd);
  EXPECT_TRUE(parser.parseAndValidate());
}

TEST_F(XmlSchemaValidationTest, InvalidXmlFails)
{
  XmlParser parser(invalid_xml, xsd);
  EXPECT_FALSE(parser.parseAndValidate());
}
