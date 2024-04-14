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

#include "test_async_function_handler.hpp"
#include "gmock/gmock.h"

namespace controller_interface
{
TestAsyncFunctionHandler::TestAsyncFunctionHandler()
: state_(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "test_async")),
  counter_(0),
  return_state_(return_type::OK)
{
}

TestAsyncFunctionHandler::~TestAsyncFunctionHandler() {}

void TestAsyncFunctionHandler::initialize()
{
  handler_.init(
    std::bind(&TestAsyncFunctionHandler::get_state, this),
    std::bind(
      &TestAsyncFunctionHandler::update, this, std::placeholders::_1, std::placeholders::_2));
}

return_type TestAsyncFunctionHandler::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  last_callback_time_ = time;
  last_callback_period_ = period;
  // to simulate some work being done
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  counter_++;
  return return_state_;
}
const rclcpp_lifecycle::State & TestAsyncFunctionHandler::get_state() const { return state_; }

}  // namespace controller_interface
class AsyncFunctionHandlerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST_F(AsyncFunctionHandlerTest, check_initialization)
{
  controller_interface::TestAsyncFunctionHandler async_class;

  ASSERT_FALSE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_async());
  ASSERT_FALSE(async_class.get_handler().is_running());

  // It should not be possible to initialize setting wrong functions
  EXPECT_THROW(async_class.get_handler().init(nullptr, nullptr), std::runtime_error);

  async_class.initialize();
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_async());
  ASSERT_FALSE(async_class.get_handler().is_running());

  // Once initialized, it should not be possible to initialize again
  ASSERT_EQ(controller_interface::return_type::OK, async_class.trigger());
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_async());
  ASSERT_TRUE(async_class.get_handler().is_running());
  EXPECT_THROW(async_class.initialize(), std::runtime_error);
  // The preempt_async_update is already called with the destructor
  // async_class.get_handler().preempt_async_update();
}

TEST_F(AsyncFunctionHandlerTest, check_triggering)
{
  controller_interface::TestAsyncFunctionHandler async_class;

  ASSERT_FALSE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_async());
  ASSERT_FALSE(async_class.get_handler().is_running());
  // It shouldn't be possible to trigger without initialization
  EXPECT_THROW(async_class.trigger(), std::runtime_error);

  async_class.initialize();
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_async());
  ASSERT_FALSE(async_class.get_handler().is_running());

  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(controller_interface::return_type::OK, async_class.trigger());
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_async());
  ASSERT_TRUE(async_class.get_handler().is_running());
  async_class.get_handler().wait_for_update_to_finish();
  ASSERT_EQ(async_class.get_counter(), 1);

  // Trigger one more cycle
  // std::this_thread::sleep_for(std::chrono::microseconds(1));
  ASSERT_EQ(controller_interface::return_type::OK, async_class.trigger());
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_async());
  ASSERT_TRUE(async_class.get_handler().is_running());
  async_class.get_handler().wait_for_update_to_finish();
  async_class.get_handler().preempt_async_update();
  ASSERT_EQ(async_class.get_counter(), 2);

  // now the async update should be preempted
  ASSERT_FALSE(async_class.get_handler().is_async());
  ASSERT_FALSE(async_class.get_handler().is_running());
  async_class.get_handler().wait_for_update_to_finish();
}
