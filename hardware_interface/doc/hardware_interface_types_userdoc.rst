:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/hardware_interface_types_userdoc.rst

.. _hardware_interface_types_userdoc:

``ros2_control`` hardware interface types
---------------------------------------------------------

The ``ros2_control`` framework provides a set of hardware interface types that can be used to implement
a hardware component for a specific robot or device.
The following sections describe the different hardware interface types and their usage.

Joints
*****************************
``<joint>``-tag groups the interfaces associated with the joints of physical robots and actuators.
They have command and state interfaces to set the goal values for hardware and read its current state.

State interfaces of joints can be published as a ROS topic by means of the :ref:`joint_state_broadcaster <joint_state_broadcaster_userdoc>`

Sensors
*****************************
``<sensor>``-tag groups multiple state interfaces describing, e.g., internal states of hardware.

Depending on the type of sensor, there exist a couple of specific semantic components with broadcasters shipped with ros2_controllers, e.g.

- :ref:`Imu Sensor Broadcaster <imu_sensor_broadcaster_userdoc>`
- :ref:`Force Torque Sensor Broadcaster <force_torque_sensor_broadcaster_userdoc>`

GPIOs
*****************************
The ``<gpio>``-tag is used for describing input and output ports of a robotic device that cannot be associated with any joint or sensor.
Parsing of ``<gpio>``-tag is similar to this of a ``<joint>``-tag having command and state interfaces.
The tag must have at least one ``<command>``- or ``<state>``-tag as a child.

The keyword "gpio" is chosen for its generality.
Although strictly used for digital signals, it describes any electrical analog, digital signal, or physical value.

The ``<gpio>`` tag can be used as a child of all three types of hardware components, i.e., system, sensor, or actuator.

Because ports implemented as ``<gpio>``-tag are typically very application-specific, there exists no generic publisher
within the ros2_control framework. A custom gpio-controller has to be implemented for each application. As an example, see :ref:`the GPIO controller example <ros2_control_demos_example_10_userdoc>` as part of the demo repository.

Examples
*****************************
The following examples show how to use the different hardware interface types in a ``ros2_control`` URDF.
They can be combined together within the different hardware component types (system, actuator, sensor) (:ref:`see detailed documentation <overview_hardware_components>`) as follows

1. Robot with multiple GPIO interfaces

   - RRBot System
   - Digital: 4 inputs and 2 outputs
   - Analog: 2 inputs and 1 output
   - Vacuum valve at the flange (on/off)


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
        <state_interface name="position"/>
      </joint>
      <joint name="joint2">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>
      <gpio name="flange_digital_IOs">
        <command_interface name="digital_output1"/>
        <state_interface name="digital_output1"/>    <!-- Needed to know current state of the output -->
        <command_interface name="digital_output2"/>
        <state_interface name="digital_output2"/>
        <state_interface name="digital_input1"/>
        <state_interface name="digital_input2"/>
      </gpio>
      <gpio name="flange_analog_IOs">
        <command_interface name="analog_output1"/>
        <state_interface name="analog_output1">    <!-- Needed to know current state of the output -->
          <param name="initial_value">3.1</param>  <!-- Optional initial value for mock_hardware -->
        </state_interface>
        <state_interface name="analog_input1"/>
        <state_interface name="analog_input2"/>
      </gpio>
      <gpio name="flange_vacuum">
        <command_interface name="vacuum"/>
        <state_interface name="vacuum"/>    <!-- Needed to know current state of the output -->
      </gpio>
    </ros2_control>

2. Gripper with electrical and suction grasping possibilities

   - Multimodal gripper
   - 1-DoF parallel gripper
   - suction on/off

  .. code:: xml

    <ros2_control name="MultimodalGripper" type="actuator">
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
        <state_interface name="suction"/>    <!-- Needed to know current state of the output -->
      </gpio>
    </ros2_control>

3. Force-Torque-Sensor with temperature feedback and adjustable calibration

   - 2D FTS
   - Temperature feedback in °C
   - Choice between 3 calibration matrices, i.e., calibration ranges

  .. code:: xml

    <ros2_control name="RRBotForceTorqueSensor2D" type="sensor">
      <hardware>
        <plugin>ros2_control_demo_hardware/ForceTorqueSensor2DHardware</plugin>
        <param name="example_param_read_for_sec">0.43</param>
      </hardware>
      <sensor name="tcp_fts_sensor">
        <state_interface name="fx"/>
        <state_interface name="tz"/>
        <param name="frame_id">kuka_tcp</param>
        <param name="fx_range">100</param>
        <param name="tz_range">100</param>
      </sensor>
      <sensor name="temp_feedback">
        <state_interface name="temperature"/>
      </sensor>
      <gpio name="calibration">
        <command_interface name="calibration_matrix_nr"/>
        <state_interface name="calibration_matrix_nr"/>
      </gpio>
    </ros2_control>
