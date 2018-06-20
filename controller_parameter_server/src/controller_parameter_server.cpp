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

#include <cstdio>
#include <memory>

#include "controller_parameter_server/parameter_server.hpp"

void
print_usage()
{
  fprintf(stderr, "controller_parameter_server [yaml1.file ... yamlN.file]\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    fprintf(stderr, "Missing arguments. No yaml files were given\n");
    print_usage();
    return -1;
  }

  auto ps = std::make_shared<controller_parameter_server::ParameterServer>();
  ps->load_parameters(argv[1]);

  rclcpp::spin(ps);
  rclcpp::shutdown();
  return 0;
}
