:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/joints_userdoc.rst

.. _joints_userdoc:


Joint Kinematics for ros2_control
---------------------------------------------------------

This page should give an overview of the joint kinematics in the context of ros2_control. It is intended to give a brief introduction to the topic and to explain the current implementation in ros2_control.

Nomenclature
############

Degrees of Freedom (DoF)
  From `wikipedia <https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics)>`__:

    In physics, the degrees of freedom (DoF) of a mechanical system is the number of independent parameters that define its configuration or state.

Joint
  A joint is a connection between two links. In the ROS ecosystem, three types are more typical: Revolute (A hinge joint with position limits), Continuous (A continuous hinge without any position limits) or Prismatic (A sliding joint that moves along the axis).

  In general, a joint can be actuated or non-actuated, also called passive. Passive joints are joints that do not have their own actuation mechanism but instead allow movement by external forces or by being passively moved by other joints. A passive joint can have a DoF of one, such as a pendulum, or it can be part of a parallel kinematic mechanism with zero DoF.

Serial Kinematics
  Serial kinematics refers to the arrangement of joints in a robotic manipulator where each joint is independent of the others, and the number of joints is equal to the DoF of the kinematic chain.

  A typical example is an industrial robot with six revolute joints, having 6-DoF. Each joint can be actuated independently, and the end-effector can be moved to any position and orientation in the workspace.

Kinematic Loops
  On the other hand, kinematic loops, also known as closed-loop mechanisms, involve several joints that are connected in a kinematic chain and being actuated together. This means that the joints are coupled and cannot be moved independently: In general, the number of DoFs is smaller than the number of joints.
  This structure is typical for parallel kinematic mechanisms, where the end-effector is connected to the base by several kinematic chains.

  An example is the four-bar linkage, which consists of four links and four joints. It can have one or two actuators and consequently one or two DoFs, despite having four joints. Furthermore, we can say that we have one (two) actuated joint and three (two) passive joints, which must satisfy the kinematic constraints of the mechanism.

URDF
#############

URDF is the default format to describe robot kinematics in ROS. However, only serial kinematic chains are supported, except for the so-called mimic joints. See the `URDF specification <http://wiki.ros.org/urdf/XML/joint>`__ for more details.

Mimic joints can be defined in the following way in the URDF

  .. code-block:: xml

    <joint name="right_finger_joint" type="prismatic">
      <axis xyz="0 1 0"/>
      <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
      <parent link="base"/>
      <child link="finger_right"/>
      <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
    </joint>
    <joint name="left_finger_joint" type="prismatic">
      <mimic joint="right_finger_joint" multiplier="1" offset="0"/>
      <axis xyz="0 1 0"/>
      <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
      <parent link="base"/>
      <child link="finger_left"/>
      <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
    </joint>

Mimic joints are an abstraction of the real world. For example, they can be used to describe

* simple closed-loop kinematics with linear dependencies of the joint positions and velocities
* links connected with belts, like belt and pulley systems or telescope arms
* a simplified model of passive joints, e.g. a pendulum at the end-effector always pointing downwards
* abstract complex groups of actuated joints, where several joints are directly controlled by low-level control loops and move synchronously. Without giving a real-world example, this could be several motors with their individual power electronics but commanded with the same setpoint.

Mimic joints defined in the URDF are parsed from the resource manager and stored in a class variable of type ``HardwareInfo``, which can be accessed by the hardware components. The mimic joints must not have command interfaces but can have state interfaces.

.. code-block:: xml

  <ros2_control>
    <joint name="right_finger_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

From the officially released packages, the following packages are already using this information:

* :ref:`mock_components (generic system) <mock_components_userdoc>`
* :ref:`gazebo_ros2_control <gazebo_ros2_control>`
* :ref:`ign_ros2_control <ign_ros2_control>`

As the URDF specifies only the kinematics, the mimic tag has to be independent of the hardware interface type used in ros2_control. This means that we interpret this info in the following way:

* **position = multiplier * other_joint_position + offset**
* **velocity = multiplier * other_joint_velocity**

If someone wants to deactivate the mimic joint behavior for whatever reason without changing the URDF, it can be done by setting the attribute ``mimic=false`` of the joint tag in the ``<ros2_control>`` section of the XML.

.. code-block:: xml

  <joint name="left_finger_joint" mimic="false">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

Transmission Interface
#######################
Mechanical transmissions transform effort/flow variables such that their product (power) remains constant. Effort variables for linear and rotational domains are force and torque; while the flow variables are respectively linear velocity and angular velocity.

In robotics it is customary to place transmissions between actuators and joints. This interface adheres to this naming to identify the input and output spaces of the transformation. The provided interfaces allow bidirectional mappings between actuator and joint spaces for effort, velocity and position. Position is not a power variable, but the mappings can be implemented using the velocity map plus an integration constant representing the offset between actuator and joint zeros.

The ``transmission_interface`` provides a base class and some implementations for plugins, which can be integrated and loaded by custom hardware components. They are not automatically loaded by any hardware component or the gazebo plugins, each hardware component is responsible for loading the appropriate transmission interface to map the actuator readings to joint readings.

Currently the following implementations are available:

* ``SimpleTransmission``: A simple transmission with a constant reduction ratio and no additional dynamics.
* ``DifferentialTransmission``: A differential transmission with two actuators and two joints.
* ``FourBarLinkageTransmission``: A four-bar-linkage transmission with two actuators and two joints.

For more information, see :ref:`example_8 <ros2_control_demos_example_8_userdoc>` or the `transmission_interface <http://docs.ros.org/en/{DISTRO}/p/transmission_interface/index.html>`__ documentation.

Simulating Closed-Loop Kinematic Chains
#######################################
Depending on the simulation plugin, different approaches can be used to simulate closed-loop kinematic chains. The following list gives an overview of the available simulation plugins and their capabilities:

gazebo_ros2_control:
  * mimic joints
  * closed-loop kinematics are supported with ``<gazebo>`` tags in the URDF, see, e.g., `here <http://classic.gazebosim.org/tutorials?tut=kinematic_loop>`__.

gz_ros2_control:
  * mimic joints
  * closed-loop kinematics are not directly supported yet, but can be implemented by using a ``DetachableJoint`` via custom plugins. Follow `this issue <https://github.com/gazebosim/gz-physics/issues/25>`__ for updates on this topic.
