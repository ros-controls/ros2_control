// Copyright 2024 ros2_control development team
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

#ifndef CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_
#define CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "controller_interface_base.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{

class AsyncControllerThread
{
public:
  /// Constructor for the AsyncControllerThread object.
  /**
   *
   * \param[in] controller shared pointer to a controller.
   * \param[in] cm_update_rate the controller manager's update rate.
   */
  AsyncControllerThread(
    std::shared_ptr<controller_interface::ControllerInterfaceBase> & controller, int cm_update_rate)
  : terminated_(false), controller_(controller), thread_{}, cm_update_rate_(cm_update_rate)
  {
  }

  AsyncControllerThread(const AsyncControllerThread & t) = delete;
  AsyncControllerThread(AsyncControllerThread && t) = delete;

  // Destructor, called when the component is erased from its map.
  ~AsyncControllerThread()
  {
    terminated_.store(true, std::memory_order_seq_cst);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  /// Creates the controller's thread.
  /**
   * Called when the controller is activated.
   *
   */
  void activate()
  {
    thread_ = std::thread(&AsyncControllerThread::controller_update_callback, this);
  }

  /// Periodically execute the controller's update method.
  /**
   * Callback of the async controller's thread.
   * **Not synchronized with the controller manager's write and read currently**
   *
   */
  void controller_update_callback()
  {
    using TimePoint = std::chrono::system_clock::time_point;
    unsigned int used_update_rate =
      controller_->get_update_rate() == 0 ? cm_update_rate_ : controller_->get_update_rate();

    auto previous_time = controller_->get_node()->now();
    while (!terminated_.load(std::memory_order_relaxed))
    {
      auto const period = std::chrono::nanoseconds(1'000'000'000 / used_update_rate);
      TimePoint next_iteration_time =
        TimePoint(std::chrono::nanoseconds(controller_->get_node()->now().nanoseconds()));

      if (
        controller_->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        auto const current_time = controller_->get_node()->now();
        auto const measured_period = current_time - previous_time;
        previous_time = current_time;
        controller_->update(
          controller_->get_node()->now(),
          (controller_->get_update_rate() != cm_update_rate_ && controller_->get_update_rate() != 0)
            ? rclcpp::Duration::from_seconds(1.0 / controller_->get_update_rate())
            : measured_period);
      }

      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }
  }

private:
  std::atomic<bool> terminated_;
  std::shared_ptr<controller_interface::ControllerInterfaceBase> controller_;
  std::thread thread_;
  unsigned int cm_update_rate_;
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_
