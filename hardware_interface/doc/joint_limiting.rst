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
* ``jerk``: ``|max|`` sets the jerk limit; ``min`` is not supported.

Acceleration and jerk limits can only be set this way, i.e., the joint must declare the corresponding command interface.

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

Both limiters work on the command interfaces of a single joint (``position``, ``velocity``, ``effort``, ``acceleration``, ``jerk``) and clamp each command independently.
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

5. **Jerk**: Clamped to ``[-max_jerk, max_jerk]``.

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

The Soft Limiter implements a **spring-damper model** for gradual limiting instead of hard clamping. This provides smoother behavior and can prevent abrupt changes in joint behavior, especially useful for safety-critical applications.

**Spring-Damper Model:**

The soft limits create a "virtual spring" that gradually increases resistance as the joint approaches the hard limit. The velocity is reduced proportionally to the distance from the soft limit boundary:

.. code-block:: text

   For lower soft limit:
   v_limit_min = -k_position * (current_position - soft_lower_limit)
   For upper soft limit:
   v_limit_max = -k_position * (current_position - soft_upper_limit)
   Where:
   - k_position: Spring constant (higher = stronger braking force)
   - current_position: Current joint position
   - soft_limit_position: The soft limit boundary

**Sign Convention:**

- When position exceeds upper soft limit (position > soft_upper): negative velocity limit applied (must move downward)
- When position falls below lower soft limit (position < soft_lower): positive velocity limit applied (must move upward)
- Uses previous command position for stability (reduces feedback lag)

**Algorithm Phases:**

1. **Normal Operation Zone** (between soft limits):
   - Joint moves freely within hard velocity limits
   - No soft limit effects applied

2. **Soft Limit Zone** (between soft and hard limits):
   - Virtual spring activates and applies proportional braking
   - Velocity is gradually reduced based on distance from soft limit
   - Damping term ``k_velocity`` prevents oscillation and provides smooth deceleration
   - Minimum velocity (≈0.017 rad/s or 1°/s) maintained for controlled approach to hard limit

3. **Hard Limit Zone** (beyond hard limits or at boundary):
   - Velocity is restricted to prevent exceeding hard limits
   - Joint can only move back toward soft limits
   - Effort limits prevent pushing further out of bounds

**Example:**

.. code-block:: text

   Hard limits: [-1.57, 1.57]
   Soft limits: [-1.40, 1.40]
   k_position: 10.0
   k_velocity: 20.0
   Position = 1.45 (in soft zone, approaching upper limit)
   Calculation:
   v_limit_max = -10.0 * (1.45 - 1.40)
   v_limit_max = -10.0 * 0.05 = -0.5 rad/s
   Result: Joint velocity is limited to -0.5 rad/s (moving downward/backward)
   This gives the joint time to decelerate before reaching hard limit at 1.57

**Soft Limits vs Hard Limits:**

- **Soft Limits**: Gradual, spring-like braking that feels smooth to operators
- **Hard Limits**: Immediate clamping that can feel abrupt but guarantees hard bounds

Use soft limits for human-robot interaction and smooth motion profiles.

Dual Enforcement Strategy
~~~~~~~~~~~~~~~~~~~~~~~~~~

Joint limits are enforced at **two points** in the control loop:

1. **Command Interface Level**: When a controller sets a command value via ``set_value()``:
   - The limiter immediately constrains the command
   - ``is_limited()`` flag indicates if limiting was applied
   - Command Interface reflects the limited value
   - Happens before the control loop update

2. **Controller Manager Update Level**: At the end of the update cycle:
   - Final enforcement ensures coherence across multiple interfaces
   - Example: If both position and velocity commands exist, limits ensure consistency
   - Prevents race conditions between different control interfaces
   - Applies additional limiting if needed

This dual approach guarantees limits are never violated, even with complex multi-interface scenarios involving feedback delays.

Thread Safety
~~~~~~~~~~~~~

Joint limits are thread-safe and support **dynamic reconfiguration**:

- Limits can be changed at runtime via parameter server updates
- Real-time safe: Uses ``RealtimeBuffer`` for zero-copy parameter access (lock-free reading in control loop)
- Safe parameter updates from the parameter server using callback mechanism
- No blocking operations in the control loop

Best Practices
~~~~~~~~~~~~~~

1. **Set Conservative Soft Limits**: Place soft limits 5-10% before hard limits to give the controller time to react
   Example: Hard limits [-1.57, 1.57] → Soft limits [-1.40, 1.40]

2. **Match Acceleration to System Capabilities**: Set ``max_acceleration`` and ``max_deceleration`` to achievable values
   - Too low: Sluggish motion, excessive limiting
   - Too high: Risk of not meeting hard limits in emergency

3. **Use Soft Limits for User Safety**: Apply soft limits in applications where sudden stops could be problematic
   - Collaborative robots (cobots)
   - Force-feedback systems
   - Sensitive machinery

4. **Enable Velocity Limits**: Always enable velocity limits for safety, especially for high-mass robots

5. **Tune Spring Constant (k_position)**: Higher values = earlier and stronger braking
   - Conservative: k_position = 5-10
   - Balanced: k_position = 10-20
   - Aggressive: k_position = 20+

6. **Monitor Logged Output**: Watch controller_manager startup messages to confirm limits are loaded correctly

7. **Test Before Deployment**: Verify joint behavior with configured limits using simulation before real hardware
