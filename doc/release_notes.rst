:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes/Jazzy.rst

Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

General
*******
* A ``version.h`` file will be generated per package using the ament_generate_version_header  (`#1449 <https://github.com/ros-controls/ros2_control/pull/1449>`_). For example, you can include the version of the package in the logs.

  .. code-block:: cpp

    #include <controller_manager/version.h>
    ...
    RCLCPP_INFO(get_logger(), "controller_manager version: %s", CONTROLLER_MANAGER_VERSION_STR);

controller_interface
********************
For details see the controller_manager section.

* Pass URDF to controllers on init (`#1088 <https://github.com/ros-controls/ros2_control/pull/1088>`_).
* Pass controller manager update rate on the init of the controller interface  (`#1141 <https://github.com/ros-controls/ros2_control/pull/1141>`_)
* A method to get node options to setup the controller node #api-breaking (`#1169 <https://github.com/ros-controls/ros2_control/pull/1169>`_)
* Export state interfaces from the chainable controller #api-breaking (`#1021 <https://github.com/ros-controls/ros2_control/pull/1021>`_)
* All chainable controllers must implement the method ``export_state_interfaces`` to export the state interfaces, similar to ``export_reference_interfaces`` method that is exporting the reference interfaces.
* The controllers will now set ``use_global_arguments`` from `NodeOptions <https://docs.ros.org/en/rolling/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html#_CPPv4N6rclcpp11NodeOptions20use_global_argumentsEb>`__ to false, to avoid getting influenced by global arguments (Issue : `#1684 <https://github.com/ros-controls/ros2_control/issues/1684>`_) (`#1694 <https://github.com/ros-controls/ros2_control/pull/1694>`_). From now on, in order to set the parameters to the controller, the ``--param-file`` option from spawner should be used.
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.
* The ``assign_interfaces`` and ``release_interfaces`` methods are now virtual, so that the user can override them to store the interfaces into custom variable types, so that the user can have the flexibility to take the ownership of the loaned interfaces to the controller (`#1743 <https://github.com/ros-controls/ros2_control/pull/1743>`_)
* The new ``PoseSensor`` semantic component provides a standard interface for hardware providing cartesian poses (`#1775 <https://github.com/ros-controls/ros2_control/pull/1775>`_)

controller_manager
******************
* URDF is now passed to controllers on init (`#1088 <https://github.com/ros-controls/ros2_control/pull/1088>`_)
  This should help avoiding extra legwork in controllers to get access to the ``/robot_description``.
* Pass controller manager update rate on the init of the controller interface (`#1141 <https://github.com/ros-controls/ros2_control/pull/1141>`_)
* Report inactive controllers as a diagnostics ok instead of an error (`#1184 <https://github.com/ros-controls/ros2_control/pull/1184>`_)
* Set chained controller interfaces 'available' for activated controllers (`#1098 <https://github.com/ros-controls/ros2_control/pull/1098>`_)

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
* Added ``get_hardware_info`` method to the hardware components interface to access the ``HardwareInfo`` instead of accessing the variable ``info_`` directly (`#1643 <https://github.com/ros-controls/ros2_control/pull/1643>`_)
* With (`#1683 <https://github.com/ros-controls/ros2_control/pull/1683>`_) the ``rclcpp_lifecycle::State & get_state()`` and ``void set_state(const rclcpp_lifecycle::State & new_state)`` are replaced by ``rclcpp_lifecycle::State & get_lifecycle_state()`` and ``void set_lifecycle_state(const rclcpp_lifecycle::State & new_state)``. This change affects controllers and hardware. This is related to (`#1240 <https://github.com/ros-controls/ros2_control/pull/1240>`_) as variant support introduces ``get_state`` and ``set_state`` methods for setting/getting state of handles.
* With (`#1421 <https://github.com/ros-controls/ros2_control/pull/1421>`_) a key-value storage is added to InterfaceInfo. This allows to define extra params with per Command-/StateInterface in the ``.ros2_control.xacro`` file.

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
