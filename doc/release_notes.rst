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

* ``ros2_control_node`` can now handle the sim time used by different simulators, when ``use_sim_time`` is set to true (`#1810 <https://github.com/ros-controls/ros2_control/pull/1810>`_).
* The ``ros2_control_node`` node now accepts the ``thread_priority`` parameter to set the scheduler priority of the controller_manager's RT thread (`#1820 <https://github.com/ros-controls/ros2_control/pull/1820>`_).
* Added support for the wildcard entries for the controller configuration files (`#1724 <https://github.com/ros-controls/ros2_control/pull/1724>`_).
* The ``ros2_control_node`` node has a new ``lock_memory`` parameter to lock memory at startup to physical RAM in order to avoid page faults (`#1822 <https://github.com/ros-controls/ros2_control/pull/1822>`_).
* The ``ros2_control_node`` node has a new ``cpu_affinity`` parameter to bind the process to a specific CPU core. By default, this is not enabled. (`#1852 <https://github.com/ros-controls/ros2_control/pull/1852>`_).
* ``--switch-timeout`` was added as parameter to the helper scripts ``spawner.py`` and ``unspawner.py``. Useful if controllers cannot be switched immediately, e.g., paused simulations at startup (`#1790 <https://github.com/ros-controls/ros2_control/pull/1790>`_).
* The spawner now supports parsing multiple ``-p`` or ``--param-file`` arguments, this should help in loading multiple parameter files for a controller or for multiple controllers (`#1805 <https://github.com/ros-controls/ros2_control/pull/1805>`_).
* The ``--service-call-timeout`` was added as parameter to the helper scripts ``spawner.py``. Useful when the CPU load is high at startup and the service call does not return immediately (`#1808 <https://github.com/ros-controls/ros2_control/pull/1808>`_).
<<<<<<< HEAD
* A python module ``test_utils`` was added to the ``controller_manager`` package to help with integration testing (`#1955 <https://github.com/ros-controls/ros2_control/pull/1955>`_).
=======
* The ``cpu_affinity`` parameter can now accept of types ``int`` or ``int_array`` to bind the process to a specific CPU core or multiple CPU cores. (`#1915 <https://github.com/ros-controls/ros2_control/pull/1915>`_).

hardware_interface
******************
* A portable version for string-to-double conversion was added: ``hardware_interface::stod`` (`#1257 <https://github.com/ros-controls/ros2_control/pull/1257>`_)
* ``test_components`` was moved to its own package (`#1325 <https://github.com/ros-controls/ros2_control/pull/1325>`_)
* The ``ros2_control`` tag now supports parsing of the limits from the URDF into the ``HardwareInfo`` structure. More conservative limits can be defined using the ``min`` and ``max`` attributes per interface (`#1472 <https://github.com/ros-controls/ros2_control/pull/1472>`_)

  .. code:: xml

    <ros2_control name="RRBotSystemMutipleGPIOs" type="system">
      <hardware>
        <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
        <param name="example_param_hw_start_duration_sec">2.0</param>
        <param name="example_param_hw_stop_duration_sec">3.0</param>
        <param name="example_param_hw_slowdown">2.0</param>
      </hardware>
      <joint name="joint1">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
          <limits enable="false"/>
        </command_interface>
        <state_interface name="position"/>
      </joint>
    </ros2_control>

* Soft limits are also parsed from the URDF into the ``HardwareInfo`` structure for the defined joints (`#1488 <https://github.com/ros-controls/ros2_control/pull/1488>`_)
* Access to logger and clock through ``get_logger`` and ``get_clock`` methods in ResourceManager and HardwareComponents ``Actuator``, ``Sensor`` and ``System`` (`#1585 <https://github.com/ros-controls/ros2_control/pull/1585>`_)
* The ``ros2_control`` tag now supports parsing read/write rate ``rw_rate`` for the each hardware component parsed through the URDF (`#1570 <https://github.com/ros-controls/ros2_control/pull/1570>`_)

  .. code:: xml

    <ros2_control name="RRBotSystemMutipleGPIOs" type="system" rw_rate="500">
      <hardware>
        <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
        <param name="example_param_hw_start_duration_sec">2.0</param>
        <param name="example_param_hw_stop_duration_sec">3.0</param>
        <param name="example_param_hw_slowdown">2.0</param>
      </hardware>
      <joint name="joint1">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>
      <joint name="joint2">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
    </ros2_control>
    <ros2_control name="MultimodalGripper" type="actuator" rw_rate="200">
      <hardware>
        <plugin>ros2_control_demo_hardware/MultimodalGripper</plugin>
      </hardware>
      <joint name="parallel_fingers">
        <command_interface name="position">
          <param name="min">0</param>
          <param name="max">100</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>
      <gpio name="suction">
        <command_interface name="suction"/>
        <state_interface name="suction"/>
      </gpio>
    </ros2_control>

* Added ``get_hardware_info`` method to the hardware components interface to access the ``HardwareInfo`` instead of accessing the variable ``info_`` directly (`#1643 <https://github.com/ros-controls/ros2_control/pull/1643>`_)
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.
* With (`#1421 <https://github.com/ros-controls/ros2_control/pull/1421>`_) a key-value storage is added to InterfaceInfo. This allows to define extra params with per Command-/StateInterface in the ``.ros2_control.xacro`` file.
* With (`#1763 <https://github.com/ros-controls/ros2_control/pull/1763>`_) parsing for SDF published to ``robot_description`` topic is now also supported.

joint_limits
************
* Add header to import limits from standard URDF definition (`#1298 <https://github.com/ros-controls/ros2_control/pull/1298>`_)

Adaption of Command-/StateInterfaces
***************************************
Changes from `(PR #1688) <https://github.com/ros-controls/ros2_control/pull/1688>`_ for an overview of related changes and discussion refer to `(PR #1240) <https://github.com/ros-controls/ros2_control/pull/1240>`_.

* ``Command-/StateInterfaces`` are now created and exported automatically by the framework via the ``on_export_command_interfaces()`` or ``on_export_state_interfaces()`` methods based on the interfaces defined in the ``ros2_control`` XML-tag, which gets parsed and the ``InterfaceDescription`` is created accordingly (check the `hardware_info.hpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/hardware_info.hpp>`__).
* The memory for storing the value of a ``Command-/StateInterfaces`` is no longer allocated in the hardware but instead in the ``Command-/StateInterfaces`` itself.
* To access the automatically created ``Command-/StateInterfaces`` we provide the ``std::unordered_map<std::string, InterfaceDescription>``, where the string is the fully qualified name of the interface and the ``InterfaceDescription`` is the configuration of the interface. The ``std::unordered_map<>`` are divided into ``type_state_interfaces_`` and ``type_command_interfaces_`` where type can be: ``joint``, ``sensor``, ``gpio`` and ``unlisted``. E.g. the ``CommandInterfaces`` for all joints can be found in the  ``joint_command_interfaces_`` map. The ``unlisted`` includes all interfaces not listed in the ``ros2_control`` XML-tag but were created by overriding the ``export_unlisted_command_interfaces()`` or ``export_unlisted_state_interfaces()`` function by creating some custom ``Command-/StateInterfaces``.


ros2controlcli
**************
* Spawner colours were added to ``list_controllers`` depending upon active or inactive (`#1409 <https://github.com/ros-controls/ros2_control/pull/1409>`_)
* The ``set_hardware_component_state`` verb was added (`#1248 <https://github.com/ros-controls/ros2_control/pull/1248>`_). Use the following command to set the state of a hardware component

  .. code-block:: bash

    ros2 control set_hardware_component_state <hardware_component_name> <state>

* The ``load_controller`` now supports parsing of the params file (`#1703 <https://github.com/ros-controls/ros2_control/pull/1703>`_).

  .. code-block:: bash

    ros2 control load_controller <controller_name> <realtive_or_absolute_file_path>

* All the ros2controlcli verbs now support the namespacing through the ROS 2 standard way (`#1703 <https://github.com/ros-controls/ros2_control/pull/1703>`_).

  .. code-block:: bash

    ros2 control <verb> <arguments> --ros-args -r __ns:=<namespace>
>>>>>>> 0ef99fe (Update CPU affinity parameter to be able to set multiple CPUs (#1915))
