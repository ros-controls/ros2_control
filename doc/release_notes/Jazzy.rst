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

hardware_interface
******************
* A portable version for string-to-double conversion was added: ``hardware_interface::stod`` (`#1257 <https://github.com/ros-controls/ros2_control/pull/1257>`_)
* ``test_components`` was moved to its own package (`#1325 <https://github.com/ros-controls/ros2_control/pull/1325>`_)

joint_limits
************
* Add header to import limits from standard URDF definition (`#1298 <https://github.com/ros-controls/ros2_control/pull/1298>`_)

ros2controlcli
**************
* Spawner colours were added to ``list_controllers`` depending upon active or inactive (`#1409 <https://github.com/ros-controls/ros2_control/pull/1409>`_)
* The ``set_hardware_component_state`` verb was added (`#1248 <https://github.com/ros-controls/ros2_control/pull/1248>`_). Use the following command to set the state of a hardware component

  .. code-block:: bash

    ros2 control set_hardware_component_state <hardware_component_name> <state>
