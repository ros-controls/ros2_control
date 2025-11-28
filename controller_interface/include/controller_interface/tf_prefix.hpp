// Copyright (c) 2025, ros2_control developers
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

#ifndef CONTROLLER_INTERFACE__TF_PREFIX_HPP_
#define CONTROLLER_INTERFACE__TF_PREFIX_HPP_

#include <string>

namespace controller_interface
{
/**
 * @brief Resolve the TF prefix with normalized slashes
 * @param prefix The TF prefix
 * @param node_ns Node namespace to use as prefix if prefix is empty
 * @return Prefix to be prepended
 */
inline std::string resolve_tf_prefix(const std::string & prefix, const std::string & node_ns)
{
  if (prefix.empty())
  {
    return "";
  }

  std::string nprefix = prefix;
  std::size_t pos = nprefix.find("~");
  if (pos != std::string::npos)
  {
    nprefix.replace(pos, 1, node_ns);
  }

  // ensure trailing '/'
  if (nprefix.back() != '/')
  {
    nprefix.push_back('/');
  }
  // remove leading '/'
  if (nprefix.front() == '/')
  {
    nprefix.erase(0, 1);
  }
  return nprefix;
}
}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__TF_PREFIX_HPP_
