:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Galactic to Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Galactic (previous) and Humble (current) releases. Bugfixes are not included in this list.

.. note::

  This list was created in July 2024, earlier changes may not be included.

controller_interface
********************

* The new ``PoseSensor`` semantic component provides a standard interface for hardware providing cartesian poses (`#1775 <https://github.com/ros-controls/ros2_control/pull/1775>`_)

controller_manager
******************

<<<<<<< HEAD
=======
  *  Configured chainable controller: Listed exported interfaces are unavailable and unclaimed
  *  Active chainable controller (not in chained mode): Listed exported interfaces are available but unclaimed
  *  Active chainable controller (in chained mode): Listed exported interfaces are available and claimed
* Try using SCHED_FIFO on any kernel (`#1142 <https://github.com/ros-controls/ros2_control/pull/1142>`_)
* A method to get node options to setup the controller node was added (`#1169 <https://github.com/ros-controls/ros2_control/pull/1169>`_): ``get_node_options`` can be overridden by controllers, this would make it easy for other controllers to be able to setup their own custom node options
* CM now subscribes to ``robot_description`` topic instead of ``~/robot_description`` (`#1410 <https://github.com/ros-controls/ros2_control/pull/1410>`_).
* Change the controller sorting with an approach similar to directed acyclic graphs (`#1384 <https://github.com/ros-controls/ros2_control/pull/1384>`_)
* Changes from `(PR #1256) <https://github.com/ros-controls/ros2_control/pull/1256>`__

  * All ``joints`` defined in the ``<ros2_control>``-tag have to be present in the URDF received :ref:`by the controller manager <doc/ros2_control/controller_manager/doc/userdoc:subscribers>`, otherwise the following error is shown:

      The published robot description file (URDF) seems not to be genuine. The following error was caught: <unknown_joint> not found in URDF.

    This is to ensure that the URDF and the ``<ros2_control>``-tag are consistent. E.g., for configuration ports use ``gpio`` interface types instead.

  * The syntax for mimic joints is changed to the `official URDF specification <https://wiki.ros.org/urdf/XML/joint>`__.

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

   The parameters within the ``ros2_control`` tag are not supported any more.
* The support for the ``description`` parameter for loading the URDF was removed (`#1358 <https://github.com/ros-controls/ros2_control/pull/1358>`_).
* The ``--controller-type`` or ``-t`` spawner arg is removed. Now the controller type is defined in the controller configuration file with ``type`` field (`#1639 <https://github.com/ros-controls/ros2_control/pull/1639>`_).
* The ``--namespace`` or ``-n`` spawner arg is deprecated. Now the spawner namespace can be defined using the ROS 2 standard way (`#1640 <https://github.com/ros-controls/ros2_control/pull/1640>`_).
* Added support for the wildcard entries for the controller configuration files (`#1724 <https://github.com/ros-controls/ros2_control/pull/1724>`_).
* ``ros2_control_node`` can now handle the sim time used by different simulators, when ``use_sim_time`` is set to true (`#1810 <https://github.com/ros-controls/ros2_control/pull/1810>`_).
>>>>>>> d714e8b ([ros2_control_node] Handle simulation environment clocks (#1810))
* The ``ros2_control_node`` node now accepts the ``thread_priority`` parameter to set the scheduler priority of the controller_manager's RT thread (`#1820 <https://github.com/ros-controls/ros2_control/pull/1820>`_).
* Added support for the wildcard entries for the controller configuration files (`#1724 <https://github.com/ros-controls/ros2_control/pull/1724>`_).
* The ``ros2_control_node`` node has a new ``lock_memory`` parameter to lock memory at startup to physical RAM in order to avoid page faults (`#1822 <https://github.com/ros-controls/ros2_control/pull/1822>`_).
* The ``ros2_control_node`` node has a new ``cpu_affinity`` parameter to bind the process to a specific CPU core. By default, this is not enabled. (`#1852 <https://github.com/ros-controls/ros2_control/pull/1852>`_).
