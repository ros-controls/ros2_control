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

struct ControllerPeerInfo
{
  std::string name = "";
  std::vector<ControllerPeerInfo *> predecessors = {};
  std::vector<ControllerPeerInfo *> successors = {};

  std::vector<std::string> get_controllers_to_activate() const
  {
    std::vector<std::string> controllers_to_activate;
    for (const auto & successor : successors)
    {
      ros2_control::add_item(controllers_to_activate, successor->name);
      successor->get_successors_and_predecessors(controllers_to_activate, name);
    }
    ros2_control::remove_item(controllers_to_activate, name);
    return controllers_to_activate;
  }

  std::vector<std::string> get_controllers_to_deactivate() const
  {
    std::vector<std::string> controllers_to_deactivate;
    for (const auto & predecessor : predecessors)
    {
      ros2_control::add_item(controllers_to_deactivate, predecessor->name);
      predecessor->get_predecessors(controllers_to_deactivate, name);
      predecessor->get_successors(controllers_to_deactivate, name);
    }
    ros2_control::remove_item(controllers_to_deactivate, name);
    return controllers_to_deactivate;
  }

  void get_predecessors(std::vector<std::string> &total_list, const std::string &untill_controller) const
  {
    for (const auto & predecessor : predecessors)
    {
      if(!predecessor)
      {
        continue;
      }
      if(predecessor->name == untill_controller)
      {
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"), "skipping predecessor: %s - %s",
          predecessor->name.c_str(), untill_controller.c_str());
        continue;
      }
      const std::string predecessor_name = predecessor->name;
      if(!ros2_control::has_item(total_list, predecessor_name))
      {
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"),
          "Getting Predecessor: %s, Successor: %s - %s", predecessor_name.c_str(), name.c_str(), untill_controller.c_str());
        total_list.push_back(predecessor_name);
        predecessor->get_predecessors(total_list, predecessor_name);
      }
    }
  }

  void get_successors(std::vector<std::string> &total_list, const std::string &untill_controller) const
  {
    for (const auto & successor : successors)
    {
      if(!successor)
      {
        continue;
      }
      if(successor->name == untill_controller)
      {
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"), "skipping successor: %s - %s",
          successor->name.c_str(), untill_controller.c_str());
        continue;
      }
      const std::string successor_name = successor->name;
      if(!ros2_control::has_item(total_list, successor_name))
      {
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"),
          "Getting Successor: %s, Predecessor: %s - %s", successor_name.c_str(), name.c_str(), untill_controller.c_str());
        total_list.push_back(successor_name);
        successor->get_successors(total_list, successor_name);
      }
    }
  }

  void get_successors_and_predecessors(std::vector<std::string> &total_list, const std::string &untill_controller) const
  {
    for (const auto & predecessor : predecessors)
    {
      if(!predecessor)
      {
        continue;
      }
      if(predecessor->name == untill_controller)
      {
        continue;
      }
      const std::string predecessor_name = predecessor->name;
      if(!ros2_control::has_item(total_list, predecessor_name))
      {
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"),
          "Predecessor: %s, Successor: %s - %s", predecessor_name.c_str(), name.c_str(), untill_controller.c_str());
        total_list.push_back(predecessor_name);
        predecessor->get_successors_and_predecessors(total_list, predecessor_name);
      }
    }
    for (const auto & successor : successors)
    {
      if(!successor)
      {
        continue;
      }
      if(successor->name == untill_controller)
      {
        continue;
      }
      const std::string successor_name = successor->name;
      if(!ros2_control::has_item(total_list, successor_name))
      {
        total_list.push_back(successor_name);
        successor->get_successors_and_predecessors(total_list, successor_name);
      }
    }
  }
};
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
    if(controller_graph_.count(predecessor) == 0)
    {
      controller_graph_[predecessor] = ControllerPeerInfo();
      controller_graph_[predecessor].name = predecessor;
    }
    if(controller_graph_.count(successor) == 0)
    {
      controller_graph_[successor] = ControllerPeerInfo();
      controller_graph_[successor].name = successor;
    }
    controller_graph_[predecessor].successors.push_back(&controller_graph_[successor]);
    controller_graph_[successor].predecessors.push_back(&controller_graph_[predecessor]);
  }

  std::vector<std::string> get_dependencies_to_activate(
    const std::string & controller_name)
  {
    if (controller_graph_.count(controller_name) == 0)
    {
      return {};
    }
    return controller_graph_[controller_name].get_controllers_to_activate();
  }

  std::vector<std::string> get_dependencies_to_deactivate(
    const std::string & controller_name)
  {
    if (controller_graph_.count(controller_name) == 0)
    {
      return {};
    }
    return controller_graph_[controller_name].get_controllers_to_deactivate();
  }

private:
  std::unordered_map<std::string, ControllerPeerInfo> controller_graph_;
};


// class ControllerChainDependencyGraph
// {
// public:
//   void add_dependency(const std::string & predecessor, const std::string & successor)
//   {
//     if (predecessors.count(predecessor) == 0)
//     {
//       predecessors[predecessor] = {};
//     }
//     if (successors.count(successor) == 0)
//     {
//       successors[successor] = {};
//     }

//     ros2_control::add_item(predecessors[successor], predecessor);
//     ros2_control::add_item(successors[predecessor], successor);
//   }

//   void remove_controller(const std::string & controller_name)
//   {
//     predecessors.erase(controller_name);
//     successors.erase(controller_name);
//     for (auto & [_, succ] : predecessors)
//     {
//       succ.erase(std::remove(succ.begin(), succ.end(), controller_name), succ.end());
//     }
//     for (auto & [_, preds] : successors)
//     {
//       preds.erase(std::remove(preds.begin(), preds.end(), controller_name), preds.end());
//     }
//   }

//   void depth_first_search(
//     const std::string & controller_name, std::unordered_set<std::string> & visited,
//     std::unordered_map<std::string, std::vector<std::string>> & graph, const std::string & untill_node = "")
//   {
//     if (visited.find(controller_name) != visited.end())
//     {
//       return;
//     }
//     if(!untill_node.empty() && controller_name == untill_node)
//     {
//       return;
//     }
//     visited.insert(controller_name);
//     for (const auto & neighbor : graph[controller_name])
//     {
//       if (visited.find(neighbor) == visited.end())
//       {
//         depth_first_search(neighbor, visited, graph);
//       }
//     }
//   }

//   std::vector<std::string> get_all_predecessors(const std::string & controller_name, const std::string & untill_node = "")
//   {
//     std::unordered_set<std::string> visited;
//     depth_first_search(controller_name, visited, predecessors, untill_node);
//     std::vector<std::string> predecessors_list;
//     std::copy(visited.begin(), visited.end(), std::back_inserter(predecessors_list));
//     ros2_control::remove_item(predecessors_list, controller_name);
//     return predecessors_list;
//   }

//   std::vector<std::string> get_all_successors(const std::string & controller_name, const std::string & untill_node = "")
//   {
//     std::unordered_set<std::string> visited;
//     depth_first_search(controller_name, visited, successors, untill_node);
//     std::vector<std::string> successors_list;
//     std::copy(visited.begin(), visited.end(), std::back_inserter(successors_list));
//     ros2_control::remove_item(successors_list, controller_name);
//     return successors_list;
//   }

//   std::vector<std::string> get_dependents_to_activate(const std::string & controller_name)
//   {
//     std::unordered_set<std::string> visited;
//     depth_first_search(controller_name, visited, successors);
//     // now for every visited controller look for all it's predecessors and their predecessors and
//     // add to the dependents list
//     std::vector<std::string> dependents_to_activate;
//     std::copy(visited.begin(), visited.end(), std::back_inserter(dependents_to_activate));
//     ros2_control::remove_item(dependents_to_activate, controller_name);
//     for (const auto & controller : visited)
//     {
//       std::vector<std::string> predecessors_list = get_all_predecessors(controller, controller_name);
//       ros2_control::remove_item(predecessors_list, controller_name);
      
//       for(const auto & predecessor : predecessors_list)
//       {
//         RCLCPP_INFO(
//           rclcpp::get_logger("controller_manager"),
//           "Predecessor of %s is %s", controller_name.c_str(), predecessor.c_str());
//         std::vector<std::string> successors_of_predecessor = get_all_successors(predecessor, controller_name);
//         ros2_control::remove_item(successors_of_predecessor, predecessor);

//         for (const auto & succ_pred : successors_of_predecessor)
//         {
//           RCLCPP_INFO(
//             rclcpp::get_logger("controller_manager"),
//             "Successor of predecessor %s is %s", predecessor.c_str(), succ_pred.c_str());
//         }
//         // insert if not already in the list
//         std::copy_if(
//           successors_of_predecessor.begin(), successors_of_predecessor.end(),
//           std::back_inserter(dependents_to_activate),
//           [&dependents_to_activate](const std::string & succ_pred)
//           {
//             return std::find(
//                     dependents_to_activate.begin(), dependents_to_activate.end(), succ_pred) ==
//                   dependents_to_activate.end();
//           });
//       }
//       // insert if not already in the list
//       std::copy_if(
//         predecessors_list.begin(), predecessors_list.end(),
//         std::back_inserter(dependents_to_activate),
//         [&dependents_to_activate](const std::string & predecessor)
//         {
//           return std::find(
//                    dependents_to_activate.begin(), dependents_to_activate.end(), predecessor) ==
//                  dependents_to_activate.end();
//         });
//     }
//     ros2_control::remove_item(dependents_to_activate, controller_name);
//     return dependents_to_activate;
//   }

// private:
//   std::unordered_map<std::string, std::vector<std::string>> predecessors = {};
//   std::unordered_map<std::string, std::vector<std::string>> successors = {};
// };

}  // namespace controller_manager
#endif  // CONTROLLER_MANAGER__CONTROLLER_SPEC_HPP_
