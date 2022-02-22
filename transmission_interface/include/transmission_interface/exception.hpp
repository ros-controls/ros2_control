// Copyright 2020 PAL Robotics S.L.
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
#ifndef TRANSMISSION_INTERFACE__EXCEPTION_HPP_
#define TRANSMISSION_INTERFACE__EXCEPTION_HPP_

#include <exception>
#include <string>

namespace transmission_interface
{
class Exception : public std::exception
{
public:
  explicit Exception(const char * message) : msg(message) {}
  explicit Exception(const std::string & message) : msg(message) {}
  const char * what() const noexcept override { return msg.c_str(); }

private:
  std::string msg;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__EXCEPTION_HPP_
