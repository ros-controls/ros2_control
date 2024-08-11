// Copyright 2020 PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef JOINT_LIMITS__JOINT_LIMITS_ROSPARAM_HPP_
#define JOINT_LIMITS__JOINT_LIMITS_ROSPARAM_HPP_

#include <limits>
#include <string>
#include <vector>

#include "joint_limits/joint_limits.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace  // utilities
{
/// Declare and initialize a parameter with a type.
/**
 *
 * Wrapper function for templated node's declare_parameter() which checks if
 * parameter is already declared.
 * For use in all components that inherit from ControllerInterface
 */
template <typename ParameterT>
auto auto_declare(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const std::string & name, const ParameterT & default_value)
{
  if (!param_itf->has_parameter(name))
  {
    auto param_default_value = rclcpp::ParameterValue(default_value);
    param_itf->declare_parameter(name, param_default_value);
  }
  return param_itf->get_parameter(name).get_value<ParameterT>();
}
}  // namespace

namespace joint_limits
{
/**
 * Declare JointLimits and SoftJointLimits parameters for joint with \p joint_name using node
 * parameters interface \p param_itf.
 *
 * The following parameter structure is declared with base name `joint_limits.joint_name`:
 * \code
 *   has_position_limits: bool
 *   min_position: double
 *   max_position: double
 *   has_velocity_limits: bool
 *   max_velocity: double
 *   has_acceleration_limits: bool
 *   max_acceleration: double
 *   has_deceleration_limits: bool
 *   max_deceleration: double
 *   has_jerk_limits: bool
 *   max_jerk: double
 *   has_effort_limits: bool
 *   max_effort: double
 *   angle_wraparound: bool
 *   has_soft_limits: bool
 *   k_position: double
 *   k_velocity: double
 *   soft_lower_limit: double
 *   soft_upper_limit: double
 * \endcode
 *
 * \param[in] joint_name name of the joint for which parameters will be declared.
 * \param[in] param_itf node parameters interface object to access parameters.
 * \param[in] logging_itf node logging interface to log if error happens.
 *
 * \returns True if parameters are successfully declared, false otherwise.
 */
inline bool declare_parameters(
  const std::string & joint_name,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  try
  {
    auto_declare<bool>(param_itf, param_base_name + ".has_position_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".min_position", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>(
      param_itf, param_base_name + ".max_position", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".has_velocity_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".max_velocity", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".has_acceleration_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".max_acceleration", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".has_deceleration_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".max_deceleration", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".has_jerk_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".max_jerk", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".has_effort_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".max_effort", std::numeric_limits<double>::quiet_NaN());
    auto_declare<bool>(param_itf, param_base_name + ".angle_wraparound", false);
    auto_declare<bool>(param_itf, param_base_name + ".has_soft_limits", false);
    auto_declare<double>(
      param_itf, param_base_name + ".k_position", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>(
      param_itf, param_base_name + ".k_velocity", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>(
      param_itf, param_base_name + ".soft_lower_limit", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>(
      param_itf, param_base_name + ".soft_upper_limit", std::numeric_limits<double>::quiet_NaN());
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(logging_itf->get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

/**
 * Declare JointLimits and SoftJointLimits parameters for joint with \p joint_name for the \p node
 * object.
 * This is a convenience function.
 * For parameters structure see the underlying `declare_parameters` function.
 *
 * \param[in] joint_name name of the joint for which parameters will be declared.
 * \param[in] node node for parameters should be declared.
 *
 * \returns True if parameters are successfully declared, false otherwise.
 */
inline bool declare_parameters(const std::string & joint_name, const rclcpp::Node::SharedPtr & node)
{
  return declare_parameters(
    joint_name, node->get_node_parameters_interface(), node->get_node_logging_interface());
}

/**
 * Declare JointLimits and SoftJointLimits parameters for joint with joint_name for the
 * lifecycle_node object. This is a convenience function. For parameters structure see the
 * underlying `declare_parameters` function.
 *
 * \param[in] joint_name name of the joint for which parameters will be declared.
 * \param[in] lifecycle_node lifecycle node for parameters should be declared.
 *
 * \returns True if parameters are successfully declared, false otherwise.
 */
inline bool declare_parameters(
  const std::string & joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node)
{
  return declare_parameters(
    joint_name, lifecycle_node->get_node_parameters_interface(),
    lifecycle_node->get_node_logging_interface());
}

/// Populate a JointLimits instance from the node parameters.
/**
 * It is assumed that parameter structure is the following:
 * \code
 *   has_position_limits: bool
 *   min_position: double
 *   max_position: double
 *   has_velocity_limits: bool
 *   max_velocity: double
 *   has_acceleration_limits: bool
 *   max_acceleration: double
 *   has_deceleration_limits: bool
 *   max_deceleration: double
 *   has_jerk_limits: bool
 *   max_jerk: double
 *   has_effort_limits: bool
 *   max_effort: double
 *   angle_wraparound: bool  # will be ignored if there are position limits
 * \endcode
 *
 * Unspecified parameters are not added to the joint limits specification.
 * A specification in a yaml would look like this:
 * \code
 * <node_name>
 *   ros__parameters:
 *     joint_limits:
 *       foo_joint:
 *         has_position_limits: true
 *         min_position: 0.0
 *         max_position: 1.0
 *         has_velocity_limits: true
 *         max_velocity: 2.0
 *         has_acceleration_limits: true
 *         max_acceleration: 5.0
 *         has_deceleration_limits: true
 *         max_deceleration: 7.5
 *         has_jerk_limits: true
 *         max_jerk: 100.0
 *         has_effort_limits: true
 *         max_effort: 20.0
 *       bar_joint:
 *         has_position_limits: false   # Continuous joint
 *         angle_wraparound: true       # available only for continuous joints
 *         has_velocity_limits: true
 *         max_velocity: 4.0
 * \endcode
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched, e.g., "foo_joint".
 * \param[in] param_itf node parameters interface of the node where parameters are specified.
 * \param[in] logging_itf node logging interface to provide log errors.
 * \param[out] limits Where joint limit data gets written into. Limits specified in the parameter
 * server will overwrite existing values. Values in \p limits not specified in the parameter server
 * remain unchanged.
 *
 * \returns True if a limits specification is found (i.e., the \p joint_limits/joint_name parameter
 * exists in \p node), false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  JointLimits & limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  try
  {
    if (
      !param_itf->has_parameter(param_base_name + ".has_position_limits") &&
      !param_itf->has_parameter(param_base_name + ".min_position") &&
      !param_itf->has_parameter(param_base_name + ".max_position") &&
      !param_itf->has_parameter(param_base_name + ".has_velocity_limits") &&
      !param_itf->has_parameter(param_base_name + ".max_velocity") &&
      !param_itf->has_parameter(param_base_name + ".has_acceleration_limits") &&
      !param_itf->has_parameter(param_base_name + ".max_acceleration") &&
      !param_itf->has_parameter(param_base_name + ".has_deceleration_limits") &&
      !param_itf->has_parameter(param_base_name + ".max_deceleration") &&
      !param_itf->has_parameter(param_base_name + ".has_jerk_limits") &&
      !param_itf->has_parameter(param_base_name + ".max_jerk") &&
      !param_itf->has_parameter(param_base_name + ".has_effort_limits") &&
      !param_itf->has_parameter(param_base_name + ".max_effort") &&
      !param_itf->has_parameter(param_base_name + ".angle_wraparound"))
    {
      RCLCPP_ERROR(
        logging_itf->get_logger(),
        "No joint limits specification found for joint '%s' in the parameter server "
        "(param name: %s).",
        joint_name.c_str(), param_base_name.c_str());
      return false;
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(logging_itf->get_logger(), "%s", ex.what());
    return false;
  }

  // Position limits
  if (param_itf->has_parameter(param_base_name + ".has_position_limits"))
  {
    limits.has_position_limits =
      param_itf->get_parameter(param_base_name + ".has_position_limits").as_bool();
    if (
      limits.has_position_limits && param_itf->has_parameter(param_base_name + ".min_position") &&
      param_itf->has_parameter(param_base_name + ".max_position"))
    {
      limits.min_position = param_itf->get_parameter(param_base_name + ".min_position").as_double();
      limits.max_position = param_itf->get_parameter(param_base_name + ".max_position").as_double();
    }
    else
    {
      limits.has_position_limits = false;
    }

    if (
      !limits.has_position_limits &&
      param_itf->has_parameter(param_base_name + ".angle_wraparound"))
    {
      limits.angle_wraparound =
        param_itf->get_parameter(param_base_name + ".angle_wraparound").as_bool();
    }
  }

  // Velocity limits
  if (param_itf->has_parameter(param_base_name + ".has_velocity_limits"))
  {
    limits.has_velocity_limits =
      param_itf->get_parameter(param_base_name + ".has_velocity_limits").as_bool();
    if (limits.has_velocity_limits && param_itf->has_parameter(param_base_name + ".max_velocity"))
    {
      limits.max_velocity = param_itf->get_parameter(param_base_name + ".max_velocity").as_double();
    }
    else
    {
      limits.has_velocity_limits = false;
    }
  }

  // Acceleration limits
  if (param_itf->has_parameter(param_base_name + ".has_acceleration_limits"))
  {
    limits.has_acceleration_limits =
      param_itf->get_parameter(param_base_name + ".has_acceleration_limits").as_bool();
    if (
      limits.has_acceleration_limits &&
      param_itf->has_parameter(param_base_name + ".max_acceleration"))
    {
      limits.max_acceleration =
        param_itf->get_parameter(param_base_name + ".max_acceleration").as_double();
    }
    else
    {
      limits.has_acceleration_limits = false;
    }
  }

  // Deceleration limits
  if (param_itf->has_parameter(param_base_name + ".has_deceleration_limits"))
  {
    limits.has_deceleration_limits =
      param_itf->get_parameter(param_base_name + ".has_deceleration_limits").as_bool();
    if (
      limits.has_deceleration_limits &&
      param_itf->has_parameter(param_base_name + ".max_deceleration"))
    {
      limits.max_deceleration =
        param_itf->get_parameter(param_base_name + ".max_deceleration").as_double();
    }
    else
    {
      limits.has_deceleration_limits = false;
    }
  }

  // Jerk limits
  if (param_itf->has_parameter(param_base_name + ".has_jerk_limits"))
  {
    limits.has_jerk_limits =
      param_itf->get_parameter(param_base_name + ".has_jerk_limits").as_bool();
    if (limits.has_jerk_limits && param_itf->has_parameter(param_base_name + ".max_jerk"))
    {
      limits.max_jerk = param_itf->get_parameter(param_base_name + ".max_jerk").as_double();
    }
    else
    {
      limits.has_jerk_limits = false;
    }
  }

  // Effort limits
  if (param_itf->has_parameter(param_base_name + ".has_effort_limits"))
  {
    limits.has_effort_limits =
      param_itf->get_parameter(param_base_name + ".has_effort_limits").as_bool();
    if (limits.has_effort_limits && param_itf->has_parameter(param_base_name + ".max_effort"))
    {
      limits.has_effort_limits = true;
      limits.max_effort = param_itf->get_parameter(param_base_name + ".max_effort").as_double();
    }
    else
    {
      limits.has_effort_limits = false;
    }
  }

  return true;
}

/**
 * Populate a JointLimits instance from the node parameters.
 * This is a convenience function.
 * For parameters structure see the underlying `get_joint_limits` function.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] node Node object for which parameters should be fetched.
 * \param[out] limits Where joint limit data gets written into. Limits specified in the parameter
 * server will overwrite existing values. Values in \p limits not specified in the parameter server
 * remain unchanged.
 *
 * \returns True if a limits specification is found, false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name, const rclcpp::Node::SharedPtr & node, JointLimits & limits)
{
  return get_joint_limits(
    joint_name, node->get_node_parameters_interface(), node->get_node_logging_interface(), limits);
}

/**
 * Populate a JointLimits instance from the node parameters.
 * This is a convenience function.
 * For parameters structure see the underlying `get_joint_limits` function.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] lifecycle_node Lifecycle node object for which parameters should be fetched.
 * \param[out] limits Where joint limit data gets written into. Limits specified in the parameter
 * server will overwrite existing values. Values in \p limits not specified in the parameter server
 * remain unchanged.
 *
 * \returns True if a limits specification is found, false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node,
  JointLimits & limits)
{
  return get_joint_limits(
    joint_name, lifecycle_node->get_node_parameters_interface(),
    lifecycle_node->get_node_logging_interface(), limits);
}

/**
 * Check if any of updated parameters are related to JointLimits.
 *
 * This method is intended to be used in the parameters update callback.
 * It is recommended that it's result is temporarily stored and synchronized with the JointLimits
 * structure in the main loop.
 *
 * \param[in] joint_name Name of the joint whose limits should be checked.
 * \param[in] parameters List of the parameters that should be checked if they belong to this
 * structure and that are used for updating the internal values.
 * \param[in] logging_itf node logging interface to provide log errors.
 * \param[out] updated_limits structure with updated JointLimits. It is recommended that the
 * currently used limits are stored into this structure before calling this method.
 */
inline bool check_for_limits_update(
  const std::string & joint_name, const std::vector<rclcpp::Parameter> & parameters,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  JointLimits & updated_limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  bool changed = false;

  // update first numerical values to make later checks for "has" limits members
  for (auto & parameter : parameters)
  {
    const std::string param_name = parameter.get_name();
    try
    {
      if (param_name == param_base_name + ".min_position")
      {
        changed = updated_limits.min_position != parameter.get_value<double>();
        updated_limits.min_position = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_position")
      {
        changed = updated_limits.max_position != parameter.get_value<double>();
        updated_limits.max_position = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_velocity")
      {
        changed = updated_limits.max_velocity != parameter.get_value<double>();
        updated_limits.max_velocity = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_acceleration")
      {
        changed = updated_limits.max_acceleration != parameter.get_value<double>();
        updated_limits.max_acceleration = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_deceleration")
      {
        changed = updated_limits.max_deceleration != parameter.get_value<double>();
        updated_limits.max_deceleration = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_jerk")
      {
        changed = updated_limits.max_jerk != parameter.get_value<double>();
        updated_limits.max_jerk = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".max_effort")
      {
        changed = updated_limits.max_effort != parameter.get_value<double>();
        updated_limits.max_effort = parameter.get_value<double>();
      }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      RCLCPP_WARN(logging_itf->get_logger(), "Please use the right type: %s", e.what());
    }
  }

  for (auto & parameter : parameters)
  {
    const std::string param_name = parameter.get_name();
    try
    {
      if (param_name == param_base_name + ".has_position_limits")
      {
        updated_limits.has_position_limits = parameter.get_value<bool>();
        if (updated_limits.has_position_limits)
        {
          if (std::isnan(updated_limits.min_position) || std::isnan(updated_limits.max_position))
          {
            RCLCPP_WARN(
              logging_itf->get_logger(),
              "PARAMETER NOT UPDATED: Position limits can not be used, i.e., "
              "'has_position_limits' flag can not be set, if 'min_position' "
              "and 'max_position' are not set or not have valid double values.");
            updated_limits.has_position_limits = false;
          }
          else if (updated_limits.min_position >= updated_limits.max_position)
          {
            RCLCPP_WARN(
              logging_itf->get_logger(),
              "PARAMETER NOT UPDATED: Position limits can not be used, i.e., "
              "'has_position_limits' flag can not be set, if not "
              "'min_position' < 'max_position'");
            updated_limits.has_position_limits = false;
          }
          else
          {
            changed = true;
          }
        }
      }
      else if (param_name == param_base_name + ".has_velocity_limits")
      {
        updated_limits.has_velocity_limits = parameter.get_value<bool>();
        if (updated_limits.has_velocity_limits && std::isnan(updated_limits.max_velocity))
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'has_velocity_limits' flag can not be set if 'min_velocity' "
            "and 'max_velocity' are not set or not have valid double values.");
          updated_limits.has_velocity_limits = false;
        }
        else
        {
          changed = true;
        }
      }
      else if (param_name == param_base_name + ".has_acceleration_limits")
      {
        updated_limits.has_acceleration_limits = parameter.get_value<bool>();
        if (updated_limits.has_acceleration_limits && std::isnan(updated_limits.max_acceleration))
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'has_acceleration_limits' flag can not be set if "
            "'max_acceleration' is not set or not have valid double values.");
          updated_limits.has_acceleration_limits = false;
        }
        else
        {
          changed = true;
        }
      }
      else if (param_name == param_base_name + ".has_deceleration_limits")
      {
        updated_limits.has_deceleration_limits = parameter.get_value<bool>();
        if (updated_limits.has_deceleration_limits && std::isnan(updated_limits.max_deceleration))
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'has_deceleration_limits' flag can not be set if "
            "'max_deceleration' is not set or not have valid double values.");
          updated_limits.has_deceleration_limits = false;
        }
        else
        {
          changed = true;
        }
      }
      else if (param_name == param_base_name + ".has_jerk_limits")
      {
        updated_limits.has_jerk_limits = parameter.get_value<bool>();
        if (updated_limits.has_jerk_limits && std::isnan(updated_limits.max_jerk))
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'has_jerk_limits' flag can not be set if 'max_jerk' is not set "
            "or not have valid double values.");
          updated_limits.has_jerk_limits = false;
        }
        else
        {
          changed = true;
        }
      }
      else if (param_name == param_base_name + ".has_effort_limits")
      {
        updated_limits.has_effort_limits = parameter.get_value<bool>();
        if (updated_limits.has_effort_limits && std::isnan(updated_limits.max_effort))
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'has_effort_limits' flag can not be set if 'max_effort' is not "
            "set or not have valid double values.");
          updated_limits.has_effort_limits = false;
        }
        else
        {
          changed = true;
        }
      }
      else if (param_name == param_base_name + ".angle_wraparound")
      {
        updated_limits.angle_wraparound = parameter.get_value<bool>();
        if (updated_limits.angle_wraparound && updated_limits.has_position_limits)
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "PARAMETER NOT UPDATED: 'angle_wraparound' flag can not be set if "
            "'has_position_limits' flag is set.");
          updated_limits.angle_wraparound = false;
        }
        else
        {
          changed = true;
        }
      }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      RCLCPP_WARN(
        logging_itf->get_logger(), "PARAMETER NOT UPDATED: Please use the right type: %s",
        e.what());
    }
  }

  return changed;
}

/// Populate a SoftJointLimits instance from the ROS parameter server.
/**
 * It is assumed that the parameter structure is the following:
 * \code
 *   has_soft_limits: bool
 *   k_position: double
 *   k_velocity: double
 *   soft_lower_limit: double
 *   soft_upper_limit: double
 * \endcode
 *
 * Only completely specified soft joint limits specifications will be considered valid.
 * For example a valid yaml configuration would look like:
 * \code
 * <node_name>
 *   ros__parameters:
 *     joint_limits:
 *       foo_joint:
 *         soft_lower_limit: 0.0
 *         soft_upper_limit: 1.0
 *         k_position: 10.0
 *         k_velocity: 10.0
 * \endcode
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched, e.g., "foo_joint".
 * \param[in] param_itf node parameters interface of the node where parameters are specified.
 * \param[in] logging_itf node logging interface to provide log errors.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the
 * parameter server will overwrite existing values. \return True if a complete soft limits
 * specification is found (ie. if all \p k_position, \p k_velocity, \p soft_lower_limit and \p
 * soft_upper_limit exist in \p joint_limits/joint_name namespace), false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  SoftJointLimits & soft_limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  try
  {
    if (
      !param_itf->has_parameter(param_base_name + ".has_soft_limits") &&
      !param_itf->has_parameter(param_base_name + ".k_velocity") &&
      !param_itf->has_parameter(param_base_name + ".k_position") &&
      !param_itf->has_parameter(param_base_name + ".soft_lower_limit") &&
      !param_itf->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      RCLCPP_DEBUG(
        logging_itf->get_logger(),
        "No soft joint limits specification found for joint '%s' in the parameter server "
        "(param name: %s).",
        joint_name.c_str(), param_base_name.c_str());
      return false;
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(logging_itf->get_logger(), "%s", ex.what());
    return false;
  }

  // Override soft limits if complete specification is found
  if (param_itf->has_parameter(param_base_name + ".has_soft_limits"))
  {
    if (
      param_itf->get_parameter(param_base_name + ".has_soft_limits").as_bool() &&
      param_itf->has_parameter(param_base_name + ".k_position") &&
      param_itf->has_parameter(param_base_name + ".k_velocity") &&
      param_itf->has_parameter(param_base_name + ".soft_lower_limit") &&
      param_itf->has_parameter(param_base_name + ".soft_upper_limit"))
    {
      soft_limits.k_position =
        param_itf->get_parameter(param_base_name + ".k_position").as_double();
      soft_limits.k_velocity =
        param_itf->get_parameter(param_base_name + ".k_velocity").as_double();
      soft_limits.min_position =
        param_itf->get_parameter(param_base_name + ".soft_lower_limit").as_double();
      soft_limits.max_position =
        param_itf->get_parameter(param_base_name + ".soft_upper_limit").as_double();
      return true;
    }
  }

  return false;
}

/**
 * Populate a JointLimits instance from the node parameters.
 * This is a convenience function.
 * For parameters structure see the underlying `get_joint_limits` function.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] node Node object for which parameters should be fetched.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the
 * parameter server will overwrite existing values.
 *
 * \returns True if a soft limits specification is found, false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name, const rclcpp::Node::SharedPtr & node,
  SoftJointLimits & soft_limits)
{
  return get_joint_limits(
    joint_name, node->get_node_parameters_interface(), node->get_node_logging_interface(),
    soft_limits);
}

/**
 * Populate a JointLimits instance from the node parameters.
 * This is a convenience function.
 * For parameters structure see the underlying `get_joint_limits` function.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] lifecycle_node Lifecycle node object for which parameters should be fetched.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the
 * parameter server will overwrite existing values.
 *
 * \returns True if a soft limits specification is found, false otherwise.
 */
inline bool get_joint_limits(
  const std::string & joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node,
  SoftJointLimits & soft_limits)
{
  return get_joint_limits(
    joint_name, lifecycle_node->get_node_parameters_interface(),
    lifecycle_node->get_node_logging_interface(), soft_limits);
}

/**
 * Check if any of updated parameters are related to SoftJointLimits.
 *
 * This method is intended to be used in the parameters update callback.
 * It is recommended that it's result is temporarily stored and synchronized with the
 * SoftJointLimits structure in the main loop.
 *
 * \param[in] joint_name Name of the joint whose limits should be checked.
 * \param[in] parameters List of the parameters that should be checked if they belong to this
 * structure and that are used for updating the internal values.
 * \param[in] logging_itf node logging interface to provide log errors.
 * \param[out] updated_limits structure with updated SoftJointLimits. It is recommended that the
 * currently used limits are stored into this structure before calling this method.
 */
inline bool check_for_limits_update(
  const std::string & joint_name, const std::vector<rclcpp::Parameter> & parameters,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  SoftJointLimits & updated_limits)
{
  const std::string param_base_name = "joint_limits." + joint_name;
  bool changed = false;

  for (auto & parameter : parameters)
  {
    const std::string param_name = parameter.get_name();
    try
    {
      if (param_name == param_base_name + ".has_soft_limits")
      {
        if (!parameter.get_value<bool>())
        {
          RCLCPP_WARN(
            logging_itf->get_logger(),
            "Parameter 'has_soft_limits' is not set, therefore the limits will not be updated!");
          return false;
        }
      }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      RCLCPP_INFO(logging_itf->get_logger(), "Please use the right type: %s", e.what());
    }
  }

  for (auto & parameter : parameters)
  {
    const std::string param_name = parameter.get_name();
    try
    {
      if (param_name == param_base_name + ".k_position")
      {
        changed = updated_limits.k_position != parameter.get_value<double>();
        updated_limits.k_position = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".k_velocity")
      {
        changed = updated_limits.k_velocity != parameter.get_value<double>();
        updated_limits.k_velocity = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".soft_lower_limit")
      {
        changed = updated_limits.min_position != parameter.get_value<double>();
        updated_limits.min_position = parameter.get_value<double>();
      }
      else if (param_name == param_base_name + ".soft_upper_limit")
      {
        changed = updated_limits.max_position != parameter.get_value<double>();
        updated_limits.max_position = parameter.get_value<double>();
      }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      RCLCPP_INFO(logging_itf->get_logger(), "Please use the right type: %s", e.what());
    }
  }

  return changed;
}

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_ROSPARAM_HPP_
