:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/different_update_rates_userdoc.rst

.. _different_update_rates_userdoc:

Different update rates for Hardware Components
===============================================================================

The ``ros2_control`` framework allows to run different hardware components at different update rates. This is useful when some of the hardware components needs to run at a different frequency than the traditional control loop frequency which is same as the one of the ``controller_manager``. It is very typical to have different components with different frequencies in a robotic system with different sensors or different components using different communication protocols.
This is useful when you have a hardware component that needs to run at a higher rate than the rest of the components. For example, you might have a sensor that needs to be read at 1000Hz, while the rest of the components can run at 500Hz, given that the control loop frequency of the ``controller_manager`` is higher than 1000Hz. The read/write rate can be defined easily by adding the parameter ``rw_rate`` to the ``ros2_control`` tag of the hardware component.

Examples
*****************************
The following examples show how to use the different hardware interface types with different update frequencies in a ``ros2_control`` URDF.
They can be combined together within the different hardware component types (system, actuator, sensor) (:ref:`see detailed documentation <overview_hardware_components>`) as follows

For a RRBot with Multimodal gripper and external sensor running at different rates:

.. code-block:: xml

  <ros2_control name="RRBotSystemMutipleGPIOs" type="system" rw_rate="500">
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
      <state_interface name="suction"/>    <!-- Needed to know current state of the output -->
    </gpio>
  </ros2_control>
  <ros2_control name="RRBotForceTorqueSensor2D" type="sensor" rw_rate="250">
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

In the above example, the system hardware component that controls the joints of the RRBot is running at 500 Hz, the multimodal gripper is running at 200 Hz and the force torque sensor is running at 250 Hz.

.. note::
  In the above example, the ``rw_rate`` parameter is set to 500 Hz, 200 Hz and 250 Hz for the system, actuator and sensor hardware components respectively. This parameter is optional and if not set, the default value of 0 will be used which means that the hardware component will run at the same rate as the ``controller_manager``. However, if the specified rate is higher than the ``controller_manager`` rate, the hardware component will then run at the rate of the ``controller_manager``.
