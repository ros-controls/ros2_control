:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/different_update_rates_userdoc.rst

.. _asynchronous_components:

Running Hardware Components Asynchronously
============================================

The ``ros2_control`` framework allows to run hardware components asynchronously. This is useful when some of the hardware components need to run in a separate thread or executor. For example, a sensor takes longer to read data that affects the periodicity of the  ``controller_manager`` control loop. In this case, the sensor can be run in a separate thread or executor to avoid blocking the control loop.

Parameters
-----------

The following parameters can be set in the ``ros2_control`` hardware configuration to run the hardware component asynchronously:

* ``is_async``: (optional) If set to ``true``, the hardware component will run asynchronously. Default is ``false``.
* ``thread_priority``: (optional) The priority of the thread that runs the hardware component. The priority is an integer value between 0 and 99. The default value is 50.

.. note::
  The thread priority is only used when the hardware component is run asynchronously.
  When the hardware component is run asynchronously, it uses the FIFO scheduling policy.

Examples
---------

The following examples show how to use the different hardware interface types synchronously and asynchronously with ``ros2_control`` URDF.
They can be combined together within the different hardware component types (system, actuator, sensor) (:ref:`see detailed documentation <overview_hardware_components>`) as follows

For a RRBot with multimodal gripper and external sensor:

.. code-block:: xml

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
  </ros2_control>
  <ros2_control name="MultimodalGripper" type="actuator" is_async="true" thread_priority="30">
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
  </ros2_control>
  <ros2_control name="RRBotForceTorqueSensor2D" type="sensor" is_async="true">
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

In the above example, the following components are defined:

* A system hardware component named ``RRBotSystemMutipleGPIOs`` with two joints and a GPIO component that runs synchronously.
* An actuator hardware component named ``MultimodalGripper`` with a joint that runs asynchronously with a thread priority of 30.
* A sensor hardware component named ``RRBotForceTorqueSensor2D`` with two sensors and a GPIO component that runs asynchronously with the default thread priority of 50.
