// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__ASYNC_COMPONENTS_HPP_
#define HARDWARE_INTERFACE__ASYNC_COMPONENTS_HPP_

#include <atomic>
#include <thread>
#include <type_traits>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"

namespace hardware_interface
{

template <typename HardwareT>
class AsyncComponentThread
{
public:
  static_assert(
    std::is_same<hardware_interface::Actuator, HardwareT>::value ||
      std::is_same<hardware_interface::System, HardwareT>::value ||
      std::is_same<hardware_interface::Sensor, HardwareT>::value,
    "Async component has to have a valid hardware type.");

  explicit AsyncComponentThread(
    HardwareT & component, unsigned int update_rate,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface)
  : async_component_(component), cm_update_rate_(update_rate), clock_interface_(clock_interface)
  {
  }

  AsyncComponentThread(const AsyncComponentThread & t) = delete;
  AsyncComponentThread(AsyncComponentThread && t) = default;

  ~AsyncComponentThread()
  {
    terminated_.store(true, std::memory_order_seq_cst);
    if (write_and_read_.joinable())
    {
      write_and_read_.join();
    }
  }

  void start() { write_and_read_ = std::thread(&AsyncComponentThread::write_and_read, this); }

  void write_and_read()
  {
    using TimePoint = std::chrono::system_clock::time_point;

    auto previous_time = clock_interface_->get_clock()->now();
    while (!terminated_.load(std::memory_order_relaxed))
    {
      auto const period = std::chrono::nanoseconds(1'000'000'000 / cm_update_rate_);
      TimePoint next_iteration_time =
        TimePoint(std::chrono::nanoseconds(clock_interface_->get_clock()->now().nanoseconds()));

      if (async_component_.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        auto current_time = clock_interface_->get_clock()->now();
        auto measured_period = current_time - previous_time;
        previous_time = current_time;

        if constexpr (!std::is_same_v<hardware_interface::Sensor, HardwareT>)
        {
          // write
        }
        // read
      }
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }
  }

private:
  std::atomic<bool> terminated_{false};
  HardwareT & async_component_;
  std::thread write_and_read_{};

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  unsigned int cm_update_rate_;
};

};  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ASYNC_COMPONENTS_HPP_
