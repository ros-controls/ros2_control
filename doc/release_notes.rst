:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

controller_interface
********************
* The new ``MagneticFieldSensor`` semantic component provides an interface for reading data from magnetometers. `(#2627 <https://github.com/ros-controls/ros2_control/pull/2627>`__)

controller_manager
******************

hardware_interface
******************

ros2controlcli
**************
* The CLI verbs ``list_hardware_components`` and ``list_hardware_interfaces`` will now show the data type used by the internal Command and State interfaces (`#2204 <https://github.com/ros-controls/ros2_control/pull/2204>`_).

transmission_interface
**********************
* The ``simple_transmission`` and ``differential_transmission`` now also support the ``force`` interface (`#2588 <https://github.com/ros-controls/ros2_control/pull/2588>`_).
