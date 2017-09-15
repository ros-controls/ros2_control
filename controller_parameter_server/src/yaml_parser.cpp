// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <unordered_map>

#include "controller_parameter_server/yaml_parser.hpp"

#ifdef __clang__
// TODO(dirk-thomas) custom implementation until we can use libc++ 3.9
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {}
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
# include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace controller_parameter_server
{

namespace
{

static constexpr char separator = '.';

void
get_key_values(
  YAML::Node root, std::string key, std::unordered_map<std::string, std::string> & key_values)
{
  if (root.Type() == YAML::NodeType::Scalar) {
    key_values.emplace(key, root.as<std::string>());
    return;
  }

  if (root.Type() == YAML::NodeType::Map) {
    for (auto root_it = root.begin(); root_it != root.end(); ++root_it) {
      get_key_values(
        root_it->second, key + separator + root_it->first.as<std::string>(), key_values);
    }
  }

  if (root.Type() == YAML::NodeType::Sequence) {
    size_t index = 0;
    for (auto root_it = root.begin(); root_it != root.end(); ++root_it) {
      get_key_values(*root_it, key + separator + std::to_string((index++)), key_values);
    }
  }
}

}  // namespace

void
YamlParser::parse(const std::string & absolute_file_path)
{
  if (!fs::path(absolute_file_path).is_absolute()) {
    throw std::runtime_error(std::string("no absolute file path given: ") + absolute_file_path);
  }

  root_node_ = YAML::LoadFile(absolute_file_path);
}

std::unordered_map<std::string, std::string>
YamlParser::get_key_value_pairs()
{
  std::unordered_map<std::string, std::string> key_values;
  get_key_values(root_node_, "", key_values);
  return key_values;
}

}  // namespace controller_parameter_server
