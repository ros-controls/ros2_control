:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/joint_limiting.rst

.. _joint_limiting:


Joint Limiting for ros2_control
---------------------------------------------------------

ros2_control provides several mechanisms to handle joint limits for different hardware interfaces.

Enable Joint Limits
#####################

There exists several ways in controlling the joint limits handling in ros2_control:

* globally for all interfaces of all hardware components via the ``ros2_control_node`` parameter ``enforce_command_limits``, for details see :ref:`here <doc/ros2_control/controller_manager/doc/userdoc:parameters>`.
* for all interfaces of a joint

  .. code-block:: xml

      <joint name="joint1">
        ...
        <limits enable="false"/>
        ...
      </joint>
* for a single interface of a joint

  .. code-block:: xml

      <joint name="joint1">
        ...
        <command_interface name="position">
          <limits enable="false"/>
        </command_interface>
        ...
      </joint>

If joint limits are active for a specific interface, the controller_manger will print a similar message on startup:

  .. code-block:: text

    [controller_manager]: Using JointLimiter for joint 'joint1' in hardware 'RRBot' : '  has position limits: true [-1, 1]
    [ros2_control_node-1]   has velocity limits: true [1]
    [ros2_control_node-1]   has acceleration limits: false [nan]
    [ros2_control_node-1]   has deceleration limits: false [nan]
    [ros2_control_node-1]   has jerk limits: false [nan]
    [ros2_control_node-1]   has effort limits: true [100]
    [ros2_control_node-1]   angle wraparound: true'

Configuration of Limits
########################
tba

Description of the Limiter Algorithms
######################################
tba
