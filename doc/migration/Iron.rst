Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

component parser
*****************
Changes from `(PR #1256) <https://github.com/ros-controls/ros2_control/pull/1256>`__

* All ``joints`` defined in the ``<ros2_control>``-tag have to be present in the URDF received :ref:`by the controller manager <doc/ros2_control/controller_manager/doc/userdoc:subscribers>`, otherwise a ``std::runtime_error`` is thrown. This is to ensure that the URDF and the ``<ros2_control>``-tag are consistent. E.g., for configuration ports use ``gpio`` tags instead.
* The syntax for mimic joints is changed to the `official URDF specification <https://wiki.ros.org/urdf/XML/joint>`__. The parameters within the ``ros2_control`` tag are not supported any more. Instead of

  .. code-block:: xml

    <ros2_control name="GazeboSystem" type="system">
      <joint name="right_finger_joint">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.15</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      <joint name="left_finger_joint">
        <param name="mimic">right_finger_joint</param>
        <param name="multiplier">1</param>
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </ros2_control>

  define your mimic joints directly in the joint definitions:

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
