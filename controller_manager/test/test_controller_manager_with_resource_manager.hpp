#ifndef CONTROLLER_MANAGER_TEST_HPP_
#define CONTROLLER_MANAGER_TEST_HPP_

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestControllerManager : public controller_manager::ControllerManager
{
public:
  using ControllerManager::ControllerManager;

  // Expose callbacks
  using ControllerManager::robot_description_callback;

  using ControllerManager::is_resource_manager_initialized;
  
  using ControllerManager::resource_manager_;

  using ControllerManager::get_robot_description_timer;  
};

class ControllerManagerTest : public ::testing::Test
{
protected:
  static std::shared_ptr<rclcpp::Node> node_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  static std::unique_ptr<hardware_interface::ResourceManager> test_resource_manager_;
  virtual void SetUp();
  virtual void TearDown();
};

#endif  