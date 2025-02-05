:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/semantic_components.rst

.. _semantic_components:

Semantic Components
---------------------------------------------------------

In order to streamline the configuration of commonly used hardware interface, so-called semantic components can be used to wrap mechanisms to claim / release the interfaces. The base components ``semantic_components::SemanticComponentInterface`` and ``semantic_components::SemanticComponentCommandInterface`` are used to define semantic components for read-only and write-only devices, respectively.

List of existing ``SemanticComponentInterface`` `(link to header file) <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/semantic_component_interface.hpp>`__ and associated broadcaster, if any:

   * `IMUSensor <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/imu_sensor.hpp>`__, used by :ref:`IMU Sensor Broadcaster <imu_sensor_broadcaster_userdoc>`
   * `ForceTorqueSensor <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/force_torque_sensor.hpp>`__, used by :ref:`Force Torque Sensor Broadcaster <force_torque_sensor_broadcaster_userdoc>`
   * `GPSSensor <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/gps_sensor.hpp>`__
   * `PoseSensor <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/pose_sensor.hpp>`__, used by :ref:`Pose Broadcaster <pose_broadcaster_userdoc>`
   * `RangeSensor <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/range_sensor.hpp>`__, used by :ref:`Range Sensor Broadcaster <range_sensor_broadcaster_userdoc>`

List of existing ``SemanticComponentCommandInterface`` `(link to header file) <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/semantic_component_command_interface.hpp>`__ and associated controller, if any:

   * `LedRgbDevice <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/led_rgb_device.hpp>`__
