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

#include "ros2_control/hardware_interface/component_validator.hpp"

namespace hardware_interface
{
bool validate_urdf_with_xsd(const std::string & urdf, const std::string & xsd_file_path)
{
  xmlDocPtr doc = xmlReadMemory(urdf.c_str(), static_cast<int>(urdf.size()), nullptr, nullptr, 0);
  if (!doc)
  {
    return false;
  }

  xmlSchemaParserCtxtPtr schemaCtx = xmlSchemaNewParserCtxt(xsd_file_path.c_str());
  if (!schemaCtx)
  {
    xmlFreeDoc(doc);
    return false;
  }
  xmlSchemaPtr schema = xmlSchemaParse(schemaCtx);
  if (!schema)
  {
    xmlSchemaFreeParserCtxt(schemaCtx);
    xmlFreeDoc(doc);
    return false;
  }

  xmlSchemaValidCtxtPtr validCtx = xmlSchemaNewValidCtxt(schema);
  if (!validCtx)
  {
    xmlSchemaFree(schema);
    xmlSchemaFreeParserCtxt(schemaCtx);
    xmlFreeDoc(doc);
    return false;
  }
  int ret = xmlSchemaValidateDoc(validCtx, doc);
  xmlSchemaFreeValidCtxt(validCtx);
  xmlSchemaFree(schema);
  xmlSchemaFreeParserCtxt(schemaCtx);
  xmlFreeDoc(doc);

  return ret == 0;
}
}  // namespace hardware_interface
