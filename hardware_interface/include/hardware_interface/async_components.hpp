// Copyright 2023 ros2_control development team
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
#include <variant>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/time.hpp"

namespace hardware_interface
{

class AsyncComponentThread
{
public:
  explicit AsyncComponentThread(
    unsigned int update_rate,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface)
  : cm_update_rate_(update_rate), clock_interface_(clock_interface)
  {
  }

  // Fills the internal variant with the desired component.
  template <typename T>
  void register_component(T * component)
  {
    hardware_component_ = component;
  }

  AsyncComponentThread(const AsyncComponentThread & t) = delete;
  AsyncComponentThread(AsyncComponentThread && t) = delete;

  // Destructor, called when the component is erased from its map.
  ~AsyncComponentThread()
  {
    terminated_.store(true, std::memory_order_seq_cst);
    if (write_and_read_.joinable())
    {
      write_and_read_.join();
    }
  }
  /// Creates the component's thread.
  /**
   * Called when the component is activated.
   *
   */
  void activate() { write_and_read_ = std::thread(&AsyncComponentThread::write_and_read, this); }

  /// Periodically execute the component's write and read methods.
  /**
   * Callback of the async component's thread.
   * **Not synchronized with the controller manager's update currently**
   *
   */
  void write_and_read()
  {
    using TimePoint = std::chrono::system_clock::time_point;

    std::visit(
      [this](auto & component)
      {
        auto previous_time = clock_interface_->get_clock()->now();
        while (!terminated_.load(std::memory_order_relaxed))
        {
          auto const period = std::chrono::nanoseconds(1'000'000'000 / cm_update_rate_);
          TimePoint next_iteration_time =
            TimePoint(std::chrono::nanoseconds(clock_interface_->get_clock()->now().nanoseconds()));

          if (
            component->get_lifecycle_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
          {
            auto current_time = clock_interface_->get_clock()->now();
            auto measured_period = current_time - previous_time;
            previous_time = current_time;

            if (!first_iteration)
            {
              component->write(clock_interface_->get_clock()->now(), measured_period);
            }
            component->read(clock_interface_->get_clock()->now(), measured_period);
            first_iteration = false;
          }
          next_iteration_time += period;
          std::this_thread::sleep_until(next_iteration_time);
        }
      },
      hardware_component_);
  }

private:
  std::atomic<bool> terminated_{false};
  std::variant<Actuator *, System *, Sensor *> hardware_component_;
  std::thread write_and_read_{};

  unsigned int cm_update_rate_;
  bool first_iteration = true;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
};

};  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ASYNC_COMPONENTS_HPP_
