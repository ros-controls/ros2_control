:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/doc/semantic_components.rst

.. _semantic_components:

Semantic Components
---------------------------------------------------------

In order to streamline the configuration of commonly used hardware interface, so-called semantic components can be used to wrap mechanisms to claim / release the interfaces. The base components ``semantic_components::SemanticComponentInterface`` and ``semantic_components::SemanticComponentCommandInterface`` are used to define semantic components for read-only and write-only devices, respectively.

List of existing ``SemanticComponentInterface`` and associated broadcaster, if any:

   * IMUSensor, used by :ref:`IMU Sensor Broadcaster <imu_sensor_broadcaster_userdoc>`
   * ForceTorqueSensor, used by :ref:`Force Torque Sensor Broadcaster <force_torque_sensor_broadcaster_userdoc>`
   * GPSSensor
   * PoseSensor, used by :ref:`Pose Broadcaster <pose_broadcaster_userdoc>`
   * RangeSensor, used by :ref:`Range Sensor Broadcaster <range_sensor_broadcaster_userdoc>`

List of existing ``SemanticComponentCommandInterface`` and associated controller, if any:

   * LedRgbDevice
