:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

controller_interface
********************
* The new ``MagneticFieldSensor`` semantic component provides an interface for reading data from magnetometers. `(#2627 <https://github.com/ros-controls/ros2_control/pull/2627>`__)

controller_manager
********************
* The ``bcolors`` class now respects the ``RCUTILS_COLORIZED_OUTPUT`` environment variable
  to automatically disable colors in non-TTY and CI environments. (`#2741 <https://github.com/ros-controls/ros2_control/pull/2741>`__)

hardware_interface
******************

ros2controlcli
**************
transmission_interface
**********************
* The ``simple_transmission`` and ``differential_transmission`` now also support the ``force`` interface (`#2588 <https://github.com/ros-controls/ros2_control/pull/2588>`_).
