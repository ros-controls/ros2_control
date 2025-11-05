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

#include <cstring>
#include <fstream>
#include <sstream>

#include "hardware_interface/component_validator.hpp"

namespace hardware_interface
{
bool validate_urdf_with_xsd(const std::string & urdf, const std::string & xsd_file_path)
{
  if (urdf.empty())
  {
    throw std::runtime_error("empty URDF passed to robot");
  }
  // Load URDF
  xmlDocPtr doc = xmlReadMemory(urdf.c_str(), static_cast<int>(urdf.size()), nullptr, nullptr, 0);
  if (!doc)
  {
    return false;
  }

  // Load XSD
  xmlSchemaParserCtxtPtr schemaCtx = xmlSchemaNewParserCtxt(xsd_file_path.c_str());
  if (!schemaCtx)
  {
    xmlFreeDoc(doc);
    return false;
  }
  // Parse XSD
  xmlSchemaPtr schema = xmlSchemaParse(schemaCtx);
  if (!schema)
  {
    xmlSchemaFreeParserCtxt(schemaCtx);
    xmlFreeDoc(doc);
    return false;
  }
  // Validate URDF against XSD
  xmlSchemaValidCtxtPtr validCtx = xmlSchemaNewValidCtxt(schema);
  if (!validCtx)
  {
    xmlSchemaFree(schema);
    xmlSchemaFreeParserCtxt(schemaCtx);
    xmlFreeDoc(doc);
    return false;
  }
  // Perform validation
  int ret = xmlSchemaValidateDoc(validCtx, doc);
  xmlSchemaFreeValidCtxt(validCtx);
  xmlSchemaFree(schema);
  xmlSchemaFreeParserCtxt(schemaCtx);
  xmlFreeDoc(doc);

  return ret == 0;
}

bool validate_urdf_file_path_with_xsd(
  const std::string & urdf_file_path, std::string & xsd_file_path)
{
  std::ifstream file(urdf_file_path);
  if (!file)
  {
    return false;
  }

  std::ostringstream ss;
  ss << file.rdbuf();
  const std::string urdf = ss.str();

  if (urdf.empty())
  {
    return false;
  }

  return validate_urdf_with_xsd(urdf, xsd_file_path);
}

bool validate_urdf_with_xsd_tag(const std::string & urdf)
{
  if (urdf.empty())
  {
    throw std::runtime_error("empty URDF passed to robot");
  }
  // extract xmlns tag from the urdf
  std::string ros2_control_xsd;
  bool result = extract_ros2_control_xsd_tag(urdf, ros2_control_xsd);
  if (!result)
  {
    return false;
  }
  return validate_urdf_with_xsd(urdf, ros2_control_xsd);
}

bool extract_ros2_control_xsd_tag(const std::string & urdf, std::string & ros2_control_xsd)
{
  if (urdf.empty())
  {
    throw std::runtime_error("empty URDF passed to robot");
  }

  xmlDocPtr doc = xmlReadMemory(urdf.c_str(), static_cast<int>(urdf.size()), nullptr, nullptr, 0);
  if (!doc)
  {
    return {};
  }

  xmlNodePtr root = xmlDocGetRootElement(doc);
  if (root)
  {
    auto check = [&](xmlNsPtr ns) -> bool
    {
      if (
        ns && ns->prefix &&
        std::strcmp(reinterpret_cast<const char *>(ns->prefix), "ros2_control") == 0)
      {
        ros2_control_xsd = ns->href ? reinterpret_cast<const char *>(ns->href) : "";
        return true;
      }
      return false;
    };
    if (!check(root->ns))
    {
      for (xmlNsPtr cur = root->nsDef; cur; cur = cur->next)
      {
        if (check(cur))
        {
          break;
        }
      }
    }
  }

  xmlFreeDoc(doc);
  return false;
}

}  // namespace hardware_interface
