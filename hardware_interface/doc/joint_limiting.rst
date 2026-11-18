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

There are two different ways the ``joint_limits`` package is used, and they are configured differently:

* **Limits enforced by the controller_manager** (see `Enable Joint Limits`_): the limits are read **only from the robot description**. ROS parameters are not used, and the limits cannot be changed at runtime.
* **Using the joint_limits library directly**, e.g., from within a controller: the limiter can read its limits from ROS parameters of that node, see `Using the joint_limits Library Directly`_.

Limits Enforced by the controller_manager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**1. Hard Limits from the URDF**

Hard limits are read from the standard ``<limit>`` tag of the URDF ``<joint>`` element (not from the ``<ros2_control>`` tag):

.. code-block:: xml

   <joint name="joint1" type="revolute">
     <parent link="base_link"/>
     <child link="link1"/>
     <limit lower="-1.57" upper="1.57" velocity="1.0" effort="100.0"/>
   </joint>
   <joint name="joint2" type="continuous">
     <parent link="link1"/>
     <child link="link2"/>
     <limit velocity="2.0" effort="50.0"/>
   </joint>

Position limits are used for ``revolute`` and ``prismatic`` joints; ``continuous`` joints get ``angle_wraparound`` enabled instead.
Velocity and effort limits are always enabled if a ``<limit>`` tag exists.
The URDF has no acceleration, deceleration, or jerk limits.

**2. Additional Limits from the ros2_control Tag**

The ``min`` and ``max`` parameters of a command interface in the ``<ros2_control>`` tag further restrict the limits:

.. code-block:: xml

   <ros2_control name="RRBot" type="system">
     <hardware>
       <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
     </hardware>
     <joint name="joint1">
       <command_interface name="position">
         <param name="min">-1.0</param>
         <param name="max">1.0</param>
       </command_interface>
       <command_interface name="acceleration">
         <param name="min">-3.0</param>  <!-- max. deceleration -->
         <param name="max">2.0</param>   <!-- max. acceleration -->
       </command_interface>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
   </ros2_control>

* ``position``, ``velocity``, ``effort``: the more restrictive of the URDF and the ``<ros2_control>`` values is used.
* ``acceleration``: ``max`` sets the acceleration limit, ``|min|`` the deceleration limit. If only ``max`` is given, it is also used for deceleration.
* ``jerk``: ``|max|`` sets the jerk limit; ``min`` is not supported. Jerk limits are parsed, but not enforced by the controller_manager (jerk commands are not passed to the limiter).

Acceleration limits can only be set this way, i.e., the joint must declare an ``acceleration`` command interface.

With the example above, ``joint1`` is limited to ``[-1.0, 1.0]`` rad.

**3. Soft Limits from the URDF**

Soft limits are read from the ``<safety_controller>`` tag of the URDF ``<joint>`` element. If they are present, the controller_manager uses the soft limiter instead of the saturation limiter for this joint.

.. code-block:: xml

   <joint name="joint1" type="revolute">
     <parent link="base_link"/>
     <child link="link1"/>
     <!-- Hard limits -->
     <limit lower="-1.57" upper="1.57" velocity="1.0" effort="100.0"/>
     <!-- Soft limits (optional) -->
     <safety_controller
       k_position="10.0"
       k_velocity="20.0"
       soft_lower_limit="-1.40"
       soft_upper_limit="1.40"/>
   </joint>

If the joint has position limits, the soft position limits are clamped to lie within them.

Limits can be disabled per joint or per command interface with ``<limits enable="false"/>``, see `Enable Joint Limits`_.

Using the joint_limits Library Directly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. note::

   This section applies only if you instantiate a limiter yourself, e.g., inside a controller.
   The limiters of the controller_manager do not read these parameters.

When a limiter is initialized with a node, e.g., ``limiter.init(joint_names, node)``, it declares and reads the parameters below from that node.
The parameters for each joint are organized under the ``joint_limits.<joint_name>`` namespace:

.. code-block:: yaml

   joint_limits:
     <joint_name>:
       # Position Limits
       has_position_limits: bool         # Enable/disable position limits
       min_position: double              # Minimum position [rad or m]
       max_position: double              # Maximum position [rad or m]
       # Velocity Limits
       has_velocity_limits: bool         # Enable/disable velocity limits
       max_velocity: double              # Maximum absolute velocity [rad/s or m/s]
       # Acceleration Limits
       has_acceleration_limits: bool     # Enable/disable acceleration limits
       max_acceleration: double          # Maximum acceleration [rad/s² or m/s²]
       # Deceleration Limits
       has_deceleration_limits: bool     # Enable/disable deceleration limits
       max_deceleration: double          # Maximum deceleration [rad/s² or m/s²]
       # Jerk Limits (rate of change of acceleration)
       has_jerk_limits: bool             # Enable/disable jerk limits
       max_jerk: double                  # Maximum jerk [rad/s³ or m/s³]
       # Effort Limits
       has_effort_limits: bool           # Enable/disable effort/force limits
       max_effort: double                # Maximum effort/force [N·m or N]
       # Angle Wraparound (for continuous joints)
       angle_wraparound: bool            # Enable angle wrapping for continuous joints [default: false]
       # Soft Limits (optional, for gradual limiting)
       has_soft_limits: bool             # Enable/disable soft limits [default: false]
       k_position: double                # Spring constant for position [dimensionless]
       k_velocity: double                # Damping coefficient for velocity [dimensionless]
       soft_lower_limit: double          # Soft limit lower boundary [rad or m]
       soft_upper_limit: double          # Soft limit upper boundary [rad or m]

The parameters belong to the node that initializes the limiter, e.g., for a controller named ``my_controller``:

.. code-block:: yaml

   my_controller:
     ros__parameters:
       joint_limits:
         joint1:
           has_position_limits: true
           min_position: -1.57
           max_position: 1.57
           has_velocity_limits: true
           max_velocity: 1.0
           has_acceleration_limits: true
           max_acceleration: 2.0
           has_effort_limits: true
           max_effort: 100.0
         joint2:
           has_position_limits: false
           has_velocity_limits: true
           max_velocity: 2.0
           angle_wraparound: true

The hard limits are updated whenever the parameters change at runtime (e.g., via ``ros2 param set``).
The soft limit parameters are declared, but not read by ``init()``. Read them yourself with ``joint_limits::get_joint_limits(joint_name, param_itf, logging_itf, soft_limits)``, see ``joint_limits/joint_limits_rosparam.hpp``.
To read limits from a URDF instead, use ``joint_limits::getJointLimits`` and ``joint_limits::getSoftJointLimits`` from ``joint_limits/joint_limits_urdf.hpp``.

Description of the Limiter Algorithms
######################################

The controller_manager uses one of two limiters per joint:

* ``joint_limits::JointSaturationLimiter<JointControlInterfacesData>`` (default), if no soft limits are defined for the joint.
* ``joint_limits::JointSoftLimiter``, if soft limits are defined for the joint.

Both limiters work on the command interfaces of a single joint (``position``, ``velocity``, ``effort``, ``acceleration``) and clamp each command independently.
They use the current state of the joint and the previous (limited) command.

.. note::

   The ``joint_limits`` package also provides ``joint_limits::JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>``, which works on ``trajectory_msgs/JointTrajectoryPoint`` messages.
   It is not used by the controller_manager and behaves differently, e.g., it recomputes position and velocity commands from each other and checks stopping distances.

Saturation Limiter (Default)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Saturation Limiter clamps each command to an interval computed from the hard limits, the current state, and the previous command (``dt`` is the update period of the controller_manager):

1. **Position**: Clamped to ``[min_position, max_position]``. If velocity limits are set, it is also clamped to ``previous_command ± max_velocity * dt``.
   If acceleration limits are set too, ``max_velocity`` is replaced by ``min(max_velocity, |current_velocity| + max_acceleration * dt)``.

2. **Velocity**: Clamped to ``[-max_velocity, max_velocity]`` and to the velocity that reaches a position limit within one update, ``[(min_position - current_position) / dt, (max_position - current_position) / dt]``.
   If acceleration limits are set, it is also clamped to ``previous_command ± max_acceleration * dt``.
   If the joint is outside its position limits, velocity commands pointing further outside are set to zero.

3. **Effort**: Clamped to ``[-max_effort, max_effort]``.
   At (or beyond) a position limit while moving towards it, effort pushing further outside is set to zero.
   The same applies if the current velocity exceeds the velocity limit.

4. **Acceleration**: Clamped to ``±max_deceleration`` if the command opposes the current velocity (i.e., decelerates) and deceleration limits are set, otherwise to ``±max_acceleration``.

5. **Jerk**: Clamped to ``[-max_jerk, max_jerk]``. This is only relevant when using the library directly, as the controller_manager does not pass jerk commands to the limiter.

If the current position is outside the position limits (plus a small tolerance), the limiter throws an exception when limiting a position command.

**Example:**

.. code-block:: text

   Position limits: [-1.0, 1.0] rad, velocity limit: 1.0 rad/s, acceleration limit: 2.0 rad/s²
   Current position: 0.95 rad, previous velocity command: 0.5 rad/s, dt = 0.01 s

   Velocity reaching the limit in one update: (1.0 - 0.95) / 0.01 = 5.0 rad/s → velocity limit 1.0 rad/s applies
   Acceleration limit: 0.5 ± 2.0 * 0.01 → velocity command is clamped to [0.48, 0.52] rad/s

The velocity bound from the position limits only prevents overshooting within the next update.
It does not consider the braking distance, so a joint can reach the position limit at high velocity.

Soft Limiter
~~~~~~~~~~~~

The Soft Limiter is used if soft limits are defined for the joint (``<safety_controller>`` in the URDF).
In addition to the hard limits enforced as in the `Saturation Limiter (Default)`_, it slows the joint down when approaching the soft position limits, like a virtual spring.
The spring is only active if the joint has both position and velocity limits, and ``k_position`` is not zero.

**Velocity bounds:**

The allowed velocity depends on the distance of the previous position command (or the current position, if there is none) to the soft limits:

.. code-block:: text

   v_min = clamp(-k_position * (previous_position - soft_lower_limit), -max_velocity, max_velocity)
   v_max = clamp(-k_position * (previous_position - soft_upper_limit), -max_velocity, max_velocity)

* **Between the soft limits**: the velocity towards a soft limit decreases linearly with the distance to it and reaches zero at the soft limit. This applies everywhere between the soft limits, not only close to them.
* **Beyond a soft limit, but within the hard limits**: the magnitude of the velocity bounds is set to 1°/s (≈0.017 rad/s), i.e., the joint creeps back towards the soft limits.
* **Beyond the hard limits** (plus a small tolerance): the velocity bounds are set to zero.

**Applying the bounds to the commands:**

1. **Position**: Clamped to ``[soft_lower_limit, soft_upper_limit]`` and to ``[previous_position + v_min * dt, previous_position + v_max * dt]``, and additionally to the position limits of the saturation limiter.
2. **Velocity**: Clamped to ``[v_min, v_max]``. If acceleration limits are set, also to ``current_velocity ± max_acceleration * dt``. Additionally, the velocity limits of the saturation limiter apply.
3. **Effort**: If effort limits are set, the effort is clamped like a damper on the velocity bounds, additionally to the effort limits of the saturation limiter:

   .. code-block:: text

      effort_min = clamp(-k_velocity * (current_velocity - v_min), -max_effort, max_effort)
      effort_max = clamp(-k_velocity * (current_velocity - v_max), -max_effort, max_effort)

4. **Acceleration** and **Jerk**: Same as in the saturation limiter (jerk only when using the library directly).

``k_velocity`` is only used for effort commands.

**Example:**

.. code-block:: text

   Hard limits: [-1.57, 1.57] rad, velocity limit: 1.0 rad/s
   Soft limits: [-1.40, 1.40] rad, k_position: 10.0 1/s
   Previous position command = current position = 1.35 rad, dt = 0.01 s

   v_max = -10.0 * (1.35 - 1.40) = 0.5 rad/s
   v_min = clamp(-10.0 * (1.35 + 1.40), -1.0, 1.0) = -1.0 rad/s
   → velocity commands are clamped to [-1.0, 0.5] rad/s
   → position commands are clamped to [1.34, 1.355] rad

   If the previous command and the current position are 1.45 rad (beyond the soft limit), both bounds are negative and set to -1°/s:
   the joint is moved back towards the soft limit at ≈0.017 rad/s.

Enforcement in the controller_manager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The controller_manager applies the limiter of a joint at two points:

1. **When a controller writes a command**: ``LoanedCommandInterface::set_value()`` passes the value through the limiter of the joint, using the nominal period ``1 / update_rate``.
   This applies to ``position``, ``velocity``, ``effort``, and ``acceleration`` command interfaces.
   The command interface stores the limited value, and ``is_limited()`` reports whether the last value was limited.

2. **After all controllers were updated**: the limiters are applied to the current values of all claimed command interfaces of the joint, using the actual period of the update cycle, before the commands are written to the hardware.

The limiters of the controller_manager are initialized once from the robot description; their limits cannot be changed at runtime.

Best Practices
~~~~~~~~~~~~~~

1. **Set velocity limits**: Without velocity limits, position commands are not rate-limited and the spring of the soft limiter is inactive.

2. **Choose k_position for the soft limits**: The velocity towards a soft limit is bounded by ``k_position * distance``.
   A higher ``k_position`` allows faster motion closer to the soft limit, i.e., the joint slows down later.
   A lower value makes the joint slow down earlier.

3. **Account for the braking distance**: The limiters do not plan a deceleration towards the position limits.
   Place the soft limits far enough from the hard limits, or limit the velocity, so that the hardware can stop in time.

4. **Check the startup log**: The controller_manager prints the limits and the limiter type for every joint, see `Enable Joint Limits`_.

5. **Test before deployment**: Verify the behavior of the joints with the configured limits in simulation before using real hardware.
