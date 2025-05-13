// Copyright 2020 Open Source Robotics Foundation, Inc.
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

/*
 * Author: Wim Meeussen
 */

#ifndef CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/controller_info.hpp"
#include "hardware_interface/helpers.hpp"
#include "hardware_interface/types/statistics_types.hpp"

namespace controller_manager
{

using MovingAverageStatistics = ros2_control::MovingAverageStatistics;
/// Controller Specification
/**
 * This struct contains both a pointer to a given controller, \ref c, as well
 * as information about the controller, \ref info.
 *
 */
struct ControllerSpec
{
  ControllerSpec()
  {
    last_update_cycle_time = std::make_shared<rclcpp::Time>(0, 0, RCL_CLOCK_UNINITIALIZED);
    execution_time_statistics = std::make_shared<MovingAverageStatistics>();
    periodicity_statistics = std::make_shared<MovingAverageStatistics>();
  }

  hardware_interface::ControllerInfo info;
  controller_interface::ControllerInterfaceBaseSharedPtr c;
  std::shared_ptr<rclcpp::Time> last_update_cycle_time;
  std::shared_ptr<MovingAverageStatistics> execution_time_statistics;
  std::shared_ptr<MovingAverageStatistics> periodicity_statistics;
};

struct ControllerChainSpec
{
  std::vector<std::string> following_controllers;
  std::vector<std::string> preceding_controllers;
};

class ControllerChainDependencyGraph
{
public:
  void add_dependency(const std::string & predecessor, const std::string & successor)
  {
    if (predecessors.count(predecessor) == 0)
    {
      predecessors[predecessor] = {};
    }
    if (successors.count(successor) == 0)
    {
      successors[successor] = {};
    }

    ros2_control::add_item(predecessors[successor], predecessor);
    ros2_control::add_item(successors[predecessor], successor);
  }

  void remove_controller(const std::string & controller_name)
  {
    predecessors.erase(controller_name);
    successors.erase(controller_name);
    for (auto & [_, succ] : predecessors)
    {
      succ.erase(std::remove(succ.begin(), succ.end(), controller_name), succ.end());
    }
    for (auto & [_, preds] : successors)
    {
      preds.erase(std::remove(preds.begin(), preds.end(), controller_name), preds.end());
    }
  }

  void depth_first_search(
    const std::string & controller_name, std::unordered_set<std::string> & visited,
    std::unordered_map<std::string, std::vector<std::string>> & graph)
  {
    if (visited.find(controller_name) != visited.end())
    {
      return;
    }
    visited.insert(controller_name);
    for (const auto & neighbor : graph[controller_name])
    {
      if (visited.find(neighbor) == visited.end())
      {
        depth_first_search(neighbor, visited, graph);
      }
    }
  }

  std::vector<std::string> get_all_predecessors(const std::string & controller_name)
  {
    std::unordered_set<std::string> visited;
    depth_first_search(controller_name, visited, predecessors);
    std::vector<std::string> predecessors_list;
    std::copy(visited.begin(), visited.end(), std::back_inserter(predecessors_list));
    predecessors_list.pop_back();  // Remove the controller itself
    return predecessors_list;
  }

  std::vector<std::string> get_all_successors(const std::string & controller_name)
  {
    std::unordered_set<std::string> visited;
    depth_first_search(controller_name, visited, successors);
    std::vector<std::string> successors_list;
    std::copy(visited.begin(), visited.end(), std::back_inserter(successors_list));
    successors_list.pop_back();  // Remove the controller itself
    return successors_list;
  }

private:
  std::unordered_map<std::string, std::vector<std::string>> predecessors = {};
  std::unordered_map<std::string, std::vector<std::string>> successors = {};
};

}  // namespace controller_manager
#endif  // CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_
