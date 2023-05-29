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

#ifndef CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_
#define CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_

#include <thread>
#include <memory>
#include <atomic>

#include "lifecycle_msgs/msg/state.hpp"
#include "controller_interface_base.hpp"

namespace controller_interface
{

class AsyncControllerThread
  {
  public:
    AsyncControllerThread(
      std::shared_ptr<controller_interface::ControllerInterfaceBase> & controller,
      int cm_update_rate)
    : terminated_(false), controller_(controller), thread_{}, cm_update_rate_(cm_update_rate)
    {
    }

    AsyncControllerThread(const AsyncControllerThread & t) = delete;
    AsyncControllerThread(AsyncControllerThread && t) = default;
    ~AsyncControllerThread()
    {
      terminated_.store(true, std::memory_order_seq_cst);
      if (thread_.joinable())
      {
        thread_.join();
      }
    }

    void activate()
    {
      thread_ = std::thread(&AsyncControllerThread::controller_update_callback, this);
    }

    void controller_update_callback()
    {
      using TimePoint = std::chrono::system_clock::time_point;
      unsigned int used_update_rate =
        controller_->get_update_rate() == 0
          ? cm_update_rate_
          : controller_
              ->get_update_rate();  // determines if the controller's or CM's update rate is used

      rclcpp::Time previous_time = controller_->get_node()->now();


      while (!terminated_.load(std::memory_order_relaxed))
      {
        auto const period = std::chrono::nanoseconds(1'000'000'000 / used_update_rate);
        TimePoint next_iteration_time =
          TimePoint(std::chrono::nanoseconds(controller_->get_node()->now().nanoseconds()));

        if (controller_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          auto const current_time = controller_->get_node()->now();
          auto const measured_period = current_time - previous_time;
          previous_time = current_time;
          controller_->update(
          controller_->get_node()->now(), (controller_->get_update_rate() != cm_update_rate_ && controller_->get_update_rate() != 0)
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

}

#endif // CONTROLLER_INTERFACE__ASYNC_CONTROLLER_HPP_
