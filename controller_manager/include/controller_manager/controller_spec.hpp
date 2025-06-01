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

#include <fmt/core.h>
#include <fmt/ranges.h>
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
  controller_interface::ControllerInterfaceBase::WeakPtr controller;
  std::unordered_set<std::string> command_interfaces = {};
  std::unordered_set<std::string> state_interfaces = {};
  std::unordered_set<std::string> reference_interfaces = {};
  std::vector<std::unordered_set<std::string>> mutually_exclusive_predecessor_groups = {};
  std::vector<std::unordered_set<std::string>> mutually_exclusive_successor_groups = {};

  void build_mutually_exclusive_predecessor_groups()
  {
    // Build mutually exclusive groups of predecessor controllers, that could utilize all the
    // reference interfaces of the current controller. This is used to determine which predecessor
    // controllers can be activated together with the current controller.

    mutually_exclusive_predecessor_groups.clear();
    const auto are_all_reference_interfaces_found =
      [](
        const std::unordered_set<std::string> & ref_itfs,
        const std::unordered_set<std::string> & cmd_itfs)
    {
      return std::all_of(
        ref_itfs.begin(), ref_itfs.end(), [&cmd_itfs](const std::string & reference_itf)
        { return cmd_itfs.find(reference_itf) != cmd_itfs.end(); });
    };
    const auto & current_reference_interfaces = reference_interfaces;
    std::for_each(
      predecessors.begin(), predecessors.end(),
      [this, &are_all_reference_interfaces_found](const ControllerPeerInfo * p)
      {
        // check if all the command interfaces of the predecessor are in the current controller's
        // reference interfaces If they are, add them as individual group
        std::unordered_set<std::string> predecessor_group = {};
        bool all_predecessor_interfaces_match =
          are_all_reference_interfaces_found(reference_interfaces, p->command_interfaces);
        if (all_predecessor_interfaces_match)
        {
          // If the predecessor's command interfaces are all in the current controller's reference
          // interfaces, add it as individual group
          predecessor_group.insert(p->name);
          mutually_exclusive_predecessor_groups.push_back(predecessor_group);
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger("controller_manager"),
            "Adding predecessor: "
              << p->name
              << " as individual group, as all its command "
                 "interfaces are in the current controller's reference interfaces.");
        }
      });

    // If the predecessor's command interfaces are not all in the current controller's reference
    // interfaces, then check other predecessors and see if they can be grouped together

    // generate combinations of predecessors that can be grouped together
    for (const auto & predecessor : predecessors)
    {
      // check if the predessort is already in the mutually exclusive group as single group
      if (std::any_of(
            mutually_exclusive_predecessor_groups.begin(),
            mutually_exclusive_predecessor_groups.end(),
            [&predecessor](const std::unordered_set<std::string> & group)
            { return group.find(predecessor->name) != group.end() && group.size() == 1; }))
      {
        continue;  // skip this predecessor, as it is already in a group as individual
      }

      // create all combinations of predecessors that can be grouped together
      // For instance, predecessors A,B,C,D. Get combinations like:
      // A,B; A,C; A,D; B,C; B,D; C,D; A,B,C; A,B,D; A,C,D; B,C,D; A,B,C,D
      // Note: This is a simplified version, in practice you would want to check if the
      // command interfaces of the predecessors do not overlap and are unique

      std::vector<std::vector<std::string>> combinations;
      std::vector<std::string> current_combination;
      std::function<void(size_t)> generate_combinations = [&](size_t start_index)
      {
        if (current_combination.size() > 1)
        {
          // check if the current combination's command interfaces are all in the
          // current controller's reference interfaces
          std::unordered_set<std::string> combined_command_interfaces;
          for (const auto & predecessor_name : current_combination)
          {
            const auto & predecessor_itf = std::find_if(
              predecessors.begin(), predecessors.end(),
              [&predecessor_name](const ControllerPeerInfo * p)
              { return p->name == predecessor_name; });
            if (predecessor_itf != predecessors.end())
            {
              combined_command_interfaces.insert(
                (*predecessor_itf)->command_interfaces.begin(),
                (*predecessor_itf)->command_interfaces.end());
            }
          }
          if (are_all_reference_interfaces_found(
                current_reference_interfaces, combined_command_interfaces))
          {
            combinations.push_back(current_combination);
          }
        }
        for (size_t i = start_index; i < predecessors.size(); ++i)
        {
          if (std::any_of(
                mutually_exclusive_predecessor_groups.begin(),
                mutually_exclusive_predecessor_groups.end(),
                [&](const std::unordered_set<std::string> & group)
                { return group.find(predecessors[i]->name) != group.end() && group.size() == 1; }))
          {
            continue;  // skip this predecessor, as it is already in a group as individual
          }

          current_combination.push_back(predecessors[i]->name);
          generate_combinations(i + 1);
          current_combination.pop_back();
        }
      };

      RCLCPP_INFO(
        rclcpp::get_logger("controller_manager"),
        "Generating combinations of predecessors for controller: %s", name.c_str());
      generate_combinations(0);
      // Add the combinations to the mutually exclusive predecessor groups
      for (const auto & combination : combinations)
      {
        std::unordered_set<std::string> group(combination.begin(), combination.end());
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"),
          fmt::format(
            "Adding predecessor group: {} with size: {}", fmt::join(combination, ", "),
            combination.size())
            .c_str());
        if (!group.empty())
        {
          mutually_exclusive_predecessor_groups.push_back(group);
        }
      }
    }
  }

  void build_mutually_exclusive_successor_groups()
  {
    // Build mutually exclusive groups of successor controllers, that could utilize all the
    // reference interfaces of the current controller. This is used to determine which successor
    // controllers can be activated together with the current controller.

    mutually_exclusive_successor_groups.clear();
    const auto are_all_command_interfaces_found =
      [](
        const std::unordered_set<std::string> & ref_itfs,
        const std::unordered_set<std::string> & cmd_itfs)
    {
      return std::all_of(
        cmd_itfs.begin(), cmd_itfs.end(), [&ref_itfs](const std::string & command_itf)
        { return ref_itfs.find(command_itf) != ref_itfs.end(); });
    };
    const auto & current_reference_interfaces = reference_interfaces;
    std::for_each(
      successors.begin(), successors.end(),
      [this, &are_all_command_interfaces_found](const ControllerPeerInfo * s)
      {
        // check if all the command interfaces of the successor are in the current controller's
        // reference interfaces If they are, add them as individual group
        std::unordered_set<std::string> successor_group = {};
        bool all_successor_interfaces_match =
          are_all_command_interfaces_found(s->reference_interfaces, command_interfaces);
        if (all_successor_interfaces_match)
        {
          // If the successor's command interfaces are all in the current controller's reference
          // interfaces, add it as individual group
          successor_group.insert(s->name);
          mutually_exclusive_successor_groups.push_back(successor_group);
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger("controller_manager"),
            "Adding successor: "
              << s->name
              << " as individual group, as all its command "
                 "interfaces are in the current controller's reference interfaces.");
        }
      });

    // If the successor's command interfaces are not all in the current controller's reference
    // interfaces, then check other successors and see if they can be grouped together

    // generate combinations of successors that can be grouped together
    for (const auto & successor : successors)
    {
      // check if the successor is already in the mutually exclusive group as single group
      if (std::any_of(
            mutually_exclusive_successor_groups.begin(), mutually_exclusive_successor_groups.end(),
            [&successor](const std::unordered_set<std::string> & group)
            { return group.find(successor->name) != group.end() && group.size() == 1; }))
      {
        continue;  // skip this successor, as it is already in a group as individual
      }
      // create all combinations of successors that can be grouped together
      // For instance, successors A,B,C,D. Get combinations like:
      // A,B; A,C; A,D; B,C; B,D; C,D; A,B,C; A,B,D; A,C,D; B,C,D; A,B,C,D
      std::vector<std::vector<std::string>> combinations;
      std::vector<std::string> current_combination;
      std::function<void(size_t)> generate_combinations = [&](size_t start_index)
      {
        if (current_combination.size() > 1)
        {
          // check if the current combination's command interfaces are all in the
          // current controller's reference interfaces
          std::unordered_set<std::string> combined_reference_interfaces;
          for (const auto & successor_name : current_combination)
          {
            const auto & successor_itf = std::find_if(
              successors.begin(), successors.end(), [&successor_name](const ControllerPeerInfo * s)
              { return s->name == successor_name; });
            if (successor_itf != successors.end())
            {
              combined_reference_interfaces.insert(
                (*successor_itf)->reference_interfaces.begin(),
                (*successor_itf)->reference_interfaces.end());
            }
          }
          if (are_all_command_interfaces_found(combined_reference_interfaces, command_interfaces))
          {
            combinations.push_back(current_combination);
          }
        }
        for (size_t i = start_index; i < successors.size(); ++i)
        {
          if (std::any_of(
                mutually_exclusive_successor_groups.begin(),
                mutually_exclusive_successor_groups.end(),
                [&](const std::unordered_set<std::string> & group)
                { return group.find(successors[i]->name) != group.end() && group.size() == 1; }))
          {
            continue;  // skip this successor, as it is already in a group as individual
          }

          current_combination.push_back(successors[i]->name);
          generate_combinations(i + 1);
          current_combination.pop_back();
        }
      };
      RCLCPP_INFO(
        rclcpp::get_logger("controller_manager"),
        "Generating combinations of successors for controller: %s", name.c_str());
      generate_combinations(0);
      // Add the combinations to the mutually exclusive successor groups
      for (const auto & combination : combinations)
      {
        std::unordered_set<std::string> group(combination.begin(), combination.end());
        RCLCPP_INFO(
          rclcpp::get_logger("controller_manager"),
          fmt::format(
            "Adding successor group: {} with size: {}", fmt::join(combination, ", "),
            combination.size())
            .c_str());
        if (!group.empty())
        {
          mutually_exclusive_successor_groups.push_back(group);
        }
      }
    }
  }

  void get_controllers_to_activate(std::vector<std::string> & controllers_to_activate) const
  {
    // Check the predecessors of the controller and check if they belong to the controller's state
    // interfaces If they do, add them to the list of controllers to activatestate_itf
    /// @todo Handle the cases where the predecessor is not active in the current state
    std::unordered_set<std::string> predecessor_command_interfaces_set = {};
    std::vector<std::string> predecessor_in_active_list = {};
    std::for_each(
      predecessors.begin(), predecessors.end(),
      [&predecessor_command_interfaces_set, &predecessor_in_active_list, &controllers_to_activate,
       this](const ControllerPeerInfo * predecessor)
      {
        if (ros2_control::has_item(controllers_to_activate, predecessor->name))
        {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("controller_manager"),
            "The predecessor: " << predecessor->name << " is already in the active list.");
          ros2_control::add_item(predecessor_in_active_list, predecessor->name);

          // Only insert those that has name of the current controller in their command interfaces
          std::for_each(
            predecessor->command_interfaces.begin(), predecessor->command_interfaces.end(),
            [&predecessor_command_interfaces_set, &predecessor,
             this](const std::string & command_itf)
            {
              if (command_itf.find(name) != std::string::npos)
              {
                predecessor_command_interfaces_set.insert(command_itf);
              }
            });
          // break;
        }
      });

    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("controller_manager"),
      "The predecessor command interfaces of the predecessor:"
        << name << " are: " << predecessor_command_interfaces_set.size());
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("controller_manager"),
      "The reference interfaces of the controller:" << name
                                                    << " are: " << reference_interfaces.size());
    if (
      !predecessor_in_active_list.empty() &&
      (predecessor_command_interfaces_set.size() != reference_interfaces.size()))
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("controller_manager"),
        "The predecessor command interfaces of the predecessor:"
          << name << " are not equal to the reference interfaces of the controller:" << name
          << " : " << predecessor_command_interfaces_set.size()
          << " != " << reference_interfaces.size());
      for (const auto & predecessor : predecessors)
      {
        if (!ros2_control::has_item(predecessor_in_active_list, predecessor->name))
        {
          ros2_control::add_item(controllers_to_activate, predecessor->name);
          predecessor->get_controllers_to_activate(controllers_to_activate);
        }
      }
    }
    for (const auto & predecessor : predecessors)
    {
      for (const auto & state_itf : state_interfaces)
      {
        if (state_itf.find(predecessor->name) != std::string::npos)
        {
          ros2_control::add_item(controllers_to_activate, predecessor->name);
          break;
        }
      }
    }

    std::unordered_set<std::string> command_interfaces_set(
      command_interfaces.begin(), command_interfaces.end());
    size_t successors_reference_interfaces_count = 0;
    for (const auto & successor : successors)
    {
      successors_reference_interfaces_count += successor->reference_interfaces.size();
    }
    for (const auto & successor : successors)
    {
      // check if all the successors reference interfaces are in the current controller's command
      // interfaces If they are, add them to the list of controllers to activate

      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("controller_manager"),
        "The command interfaces of the predecessor:" << name
                                                     << " are: " << command_interfaces_set.size());
      for (const auto & command_itf : command_interfaces_set)
      {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("controller_manager"),
          "The command interfaces of the predecessor:" << name << " are: " << command_itf);
      }

      for (const auto & reference_itf : successor->reference_interfaces)
      {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("controller_manager"),
          "The reference interfaces of the successor:" << successor->name
                                                       << " are: " << reference_itf);
      }

      bool all_successor_interfaces_match = false;
      std::for_each(
        command_interfaces.begin(), command_interfaces.end(),
        [&successor, &all_successor_interfaces_match](const std::string & command_itf)
        {
          if (
            successor->reference_interfaces.find(command_itf) !=
            successor->reference_interfaces.end())
          {
            all_successor_interfaces_match = true;
          }
        });
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("controller_manager"),
        "The reference interfaces of the successor: "
          << successor->name << " are within the command interfaces of the predecessor: " << name
          << " : " << std::boolalpha << all_successor_interfaces_match);
      if (all_successor_interfaces_match)
      {
        ros2_control::add_item(controllers_to_activate, successor->name);
        successor->get_controllers_to_activate(controllers_to_activate);
        continue;
      }
      else
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("controller_manager"),
          "Controller %s has a successor %s who has more reference interfaces that use different "
          "controllers. This is not supported now.",
          name.c_str(), successor->name.c_str());
      }
    }
  }

  void get_controllers_to_deactivate(std::vector<std::string> & controllers_to_deactivate) const
  {
    // All predecessors of the controller should be deactivated except the state interface ones
    for (const auto & predecessor : predecessors)
    {
      if (ros2_control::has_item(controllers_to_deactivate, predecessor->name))
      {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("controller_manager"),
          "The predecessor: " << predecessor->name << " is already in the deactivation list.");
        continue;
      }
      ros2_control::add_item(controllers_to_deactivate, predecessor->name);
      std::for_each(
        state_interfaces.begin(), state_interfaces.end(),
        [&predecessor, &controllers_to_deactivate](const std::string & state_itf)
        {
          if (state_itf.find(predecessor->name) != std::string::npos)
          {
            ros2_control::remove_item(controllers_to_deactivate, predecessor->name);
          }
        });
      predecessor->get_controllers_to_deactivate(controllers_to_deactivate);
    }

    // All successors of controller with no command interfaces should be deactivated
    for (const auto & successor : successors)
    {
      if (ros2_control::has_item(controllers_to_deactivate, successor->name))
      {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("controller_manager"),
          "The successor: " << successor->name << " is already in the deactivation list.");
        continue;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("controller_manager"),
        fmt::format(
          "The controllers to deactivate list is {}", fmt::join(controllers_to_deactivate, ", "))
          .c_str());

      // Check if the successor is an individual exclusive group, if so, then return
      if (std::any_of(
            mutually_exclusive_successor_groups.begin(), mutually_exclusive_successor_groups.end(),
            [&successor](const std::unordered_set<std::string> & group)
            { return group.find(successor->name) != group.end() && group.size() == 1; }))
      {
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("controller_manager"),
          "The successor: " << successor->name
                            << " is in a mutually exclusive group, skipping further deactivation.");
        continue;
      }

      if (successor->command_interfaces.empty())
      {
        ros2_control::add_item(controllers_to_deactivate, successor->name);
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("controller_manager"),
          "Adding successor: " << successor->name
                               << " to the deactivation list, as it has no command interfaces.");
      }
      else
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("controller_manager"),
          "Controller %s has a successor %s who has command interfaces. This is not supported now.",
          name.c_str(), successor->name.c_str());
      }
      successor->get_controllers_to_deactivate(controllers_to_deactivate);
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
    if (controller_graph_.count(predecessor) == 0)
    {
      controller_graph_[predecessor] = ControllerPeerInfo();
      controller_graph_[predecessor].name = predecessor;
    }
    if (controller_graph_.count(successor) == 0)
    {
      controller_graph_[successor] = ControllerPeerInfo();
      controller_graph_[successor].name = successor;
    }
    controller_graph_[predecessor].successors.push_back(&controller_graph_[successor]);
    controller_graph_[successor].predecessors.push_back(&controller_graph_[predecessor]);
  }

  void add_dependency(const ControllerPeerInfo & predecessor, const ControllerPeerInfo & successor)
  {
    if (controller_graph_.count(predecessor.name) == 0)
    {
      controller_graph_[predecessor.name] = predecessor;
    }
    if (controller_graph_.count(successor.name) == 0)
    {
      controller_graph_[successor.name] = successor;
    }
    controller_graph_[predecessor.name].successors.push_back(&controller_graph_[successor.name]);
    controller_graph_[successor.name].predecessors.push_back(&controller_graph_[predecessor.name]);
  }

  std::vector<std::string> get_dependencies_to_activate(const std::string & controller_name)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("controller_manager"),
      "+++++++++++++++++++++++++++++++ Getting dependencies to ACTIVATE "
      "+++++++++++++++++++++++++++++++");
    std::vector<std::string> controllers_to_activate({controller_name});
    if (controller_graph_.count(controller_name) == 0)
    {
      return {};
    }
    controller_graph_[controller_name].build_mutually_exclusive_predecessor_groups();
    controller_graph_[controller_name].build_mutually_exclusive_successor_groups();
    controller_graph_[controller_name].get_controllers_to_activate(controllers_to_activate);
    return controllers_to_activate;
  }

  std::vector<std::string> get_dependencies_to_deactivate(const std::string & controller_name)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("controller_manager"),
      "+++++++++++++++++++++++++++++++ Getting dependencies to DEACTIVATE "
      "+++++++++++++++++++++++++++++++");
    std::vector<std::string> controllers_to_deactivate({controller_name});
    if (controller_graph_.count(controller_name) == 0)
    {
      return {};
    }
    controller_graph_[controller_name].build_mutually_exclusive_predecessor_groups();
    controller_graph_[controller_name].build_mutually_exclusive_successor_groups();
    controller_graph_[controller_name].get_controllers_to_deactivate(controllers_to_deactivate);
    return controllers_to_deactivate;
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
//     std::unordered_map<std::string, std::vector<std::string>> & graph, const std::string &
//     untill_node = "")
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

//   std::vector<std::string> get_all_predecessors(const std::string & controller_name, const
//   std::string & untill_node = "")
//   {
//     std::unordered_set<std::string> visited;
//     depth_first_search(controller_name, visited, predecessors, untill_node);
//     std::vector<std::string> predecessors_list;
//     std::copy(visited.begin(), visited.end(), std::back_inserter(predecessors_list));
//     ros2_control::remove_item(predecessors_list, controller_name);
//     return predecessors_list;
//   }

//   std::vector<std::string> get_all_successors(const std::string & controller_name, const
//   std::string & untill_node = "")
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
//       std::vector<std::string> predecessors_list = get_all_predecessors(controller,
//       controller_name); ros2_control::remove_item(predecessors_list, controller_name);

//       for(const auto & predecessor : predecessors_list)
//       {
//         RCLCPP_INFO(
//           rclcpp::get_logger("controller_manager"),
//           "Predecessor of %s is %s", controller_name.c_str(), predecessor.c_str());
//         std::vector<std::string> successors_of_predecessor = get_all_successors(predecessor,
//         controller_name); ros2_control::remove_item(successors_of_predecessor, predecessor);

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
