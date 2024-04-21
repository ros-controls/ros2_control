:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/joints_userdoc.rst

.. _joints_userdoc:


Joint Kinematics for ros2_control
---------------------------------------------------------

Nomenclature
************

Joint
######
A joint is a connection between two links. In the ROS world, two types are typical: Revolute (having position limits; or continuous without position limits) or prismatic.

In general, a joint can be actuated or non-actuated, also called passive. A passive joints can have degrees-of-freedom of one, e.g., a pendulum, but also can be part of a parallel-kinematic mechanism having zero degrees-of-freedom.

Serial Kinematics
#################
Every joint is independent of others, and the number of joints equals the degrees-of-freedom of the kinematic chain.

Parallel Kinematics
###################
With parallel kinematics, or closed-loop kinematics, several joints are connected in a kinematic chain and are actuated together. This means that the joints are coupled and cannot be moved independently: In general, the number of degrees-of-freedom is lower than the number of joints. An example is a four-bar-linkage, having four links and four joints. It can have one or two actuators and, subsequently, one or two degrees-of-freedom despite having four joints. Furthermore, we can say that we have one (two) actuated joint and three (two) passive joints, which have to fulfil the kinematic constraints of the mechanism.


Relevant for ros2_control
*************************

URDF
#############

URDF is the default format to describe robot kinematics in ROS. However, only serial kinematic chains are supported, except for the so-called mimic joints.
From the `URDF <http://wiki.ros.org/urdf/XML/joint>`__ specification, we can read

  This tag is used to specify that the defined joint mimics another existing joint. The value of this joint can be computed as value = multiplier * other_joint_value + offset.

It can be defined in the following way

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

Mimic joints are an abstraction of the real world: They can be used to describe

* simple parallel kinematics with linear dependencies of the joint positions
* abstract complex groups of actuated joints, where the positions of several joints are directly controlled by low-level control loops and move synchronously.

Mimic joints defined in the URDF are parsed from the resource manager and stored in a class variable of type ``HardwareInfo``. It can be used by custom hardware components to implement the mimic joint behavior. From our released packages, the following packages are already using this information:

* :ref:`mock_components (generic system) <mock_components_userdoc>`
* :ref:`gazebo_ros2_control <gazebo_ros2_control>`
* :ref:`gz_ros2_control <gz_ros2_control>`


Transmission Interface
#######################
Mechanical transmissions transform effort/flow variables such that their product (power) remains constant. Effort variables for linear and rotational domains are force and torque; while the flow variables are respectively linear velocity and angular velocity.

In robotics it is customary to place transmissions between actuators and joints. This interface adheres to this naming to identify the input and output spaces of the transformation. The provided interfaces allow bidirectional mappings between actuator and joint spaces for effort, velocity and position. Position is not a power variable, but the mappings can be implemented using the velocity map plus an integration constant representing the offset between actuator and joint zeros.

The ``transmission_interface`` provides a base class and some implementations for plugins, which can be integrated and loaded by custom hardware components. They are not automatically loaded by any hardware component or the gazebo plugins.

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
  * closed-loop kinematics are supported with ``<gazebo>`` tags in the URDF, see, e.g., `here <http://classic.gazebosim.org/tutorials?tut=kinematic_loop&cat=#Split4-barlinkageinURDFwithanSDFormatfixedjoint>`__.

gz_ros2_control:
  * mimic joints
  * closed-loop kinematics are not directly supported yet, but can be implemented by using a ``DetachableJoint`` via custom plugins. Follow `this issue <https://github.com/gazebosim/gz-physics/issues/25>`__ for updates on this topic.
