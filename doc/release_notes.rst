:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

controller_interface
********************
* The new ``MagneticFieldSensor`` semantic component provides an interface for reading data from magnetometers. (`#2627 <https://github.com/ros-controls/ros2_control/pull/2627>`__)
* The controller_manager will now deactivate the entire controller chain if a controller in the chain fails during the update cycle. `(#2681 <https://github.com/ros-controls/ros2_control/pull/2681>`__)
* The update rate of the controller will now be approximated to a closer achievable frequency, when its frequency is not achievable with the current controller manager update rate. (`#2828 <https://github.com/ros-controls/ros2_control/pull/2828>`__)
* The lifecycle ID is cached internally in the controller to avoid calls to get_lifecycle_state() in the real-time control loop. (`#2884 <https://github.com/ros-controls/ros2_control/pull/2884>`__)

controller_manager
******************
* The ``bcolors`` class now respects the ``RCUTILS_COLORIZED_OUTPUT`` environment
  variable to automatically disable colors in non-TTY and CI environments.
* The default strictness for ``switch_controller`` is changed to ``strict``. (`#2742 <https://github.com/ros-controls/ros2_control/pull/2742>`__)
* A new parameter ``handle_exceptions`` is added to the controller manager to control whether exceptions thrown by controllers during update are caught and handled internally or propagated. (`#2807 <https://github.com/ros-controls/ros2_control/pull/2807>`__)

hardware_interface
******************
* The lifecycle ID is cached internally in the controller to avoid calls to get_lifecycle_state() in the real-time control loop. (`#2884 <https://github.com/ros-controls/ros2_control/pull/2884>`__)
* Handles now also support ``float32``, ``uint8``, ``int8``, ``uint16``, ``int16``, ``uint32``, ``int32`` data types in addition to double and bool. (`#2879 <https://github.com/ros-controls/ros2_control/pull/2879>`__)

ros2controlcli
**************

No notable changes in this release.

transmission_interface
**********************
* The ``simple_transmission`` and ``differential_transmission`` now also support the ``force`` interface (`#2588 <https://github.com/ros-controls/ros2_control/pull/2588>`_).
