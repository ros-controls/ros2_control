// Copyright 2024 PAL Robotics S.L.
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

#ifndef TEST_ASYNC_FUNCTION_HANDLER_HPP_
#define TEST_ASYNC_FUNCTION_HANDLER_HPP_

#include "controller_interface/async_function_handler.hpp"
#include "controller_interface/controller_interface_base.hpp"

namespace controller_interface
{
class TestAsyncFunctionHandler
{
public:
  TestAsyncFunctionHandler();

  void initialize();

  ros2_control::AsyncFunctionHandler<return_type> & get_handler() { return handler_; }

  return_type trigger();

  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);

  const rclcpp_lifecycle::State & get_state() const;

  int get_counter() const;

  void activate();

  void deactivate();

private:
  rclcpp_lifecycle::State state_;
  rclcpp::Time last_callback_time_;
  rclcpp::Duration last_callback_period_{0, 0};
  int counter_;
  return_type return_state_;
  ros2_control::AsyncFunctionHandler<return_type> handler_;
};
}  // namespace controller_interface
#endif  // TEST_ASYNC_FUNCTION_HANDLER_HPP_
