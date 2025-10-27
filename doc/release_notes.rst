:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

controller_interface
********************
* The new ``MagneticFieldSensor`` semantic component provides an interface for reading data from magnetometers. (`#2627 <https://github.com/ros-controls/ros2_control/pull/2627>`__)
* The controller_manager will now deactivate the entire controller chain if a controller in the chain fails during the update cycle. `(#2681 <https://github.com/ros-controls/ros2_control/pull/2681>`__)

controller_manager
******************
* The default strictness for ``switch_controller`` is changed to ``strict``. (`#2742 <https://github.com/ros-controls/ros2_control/pull/2742>`__)


hardware_interface
******************

ros2controlcli
**************
transmission_interface
**********************
* The ``simple_transmission`` and ``differential_transmission`` now also support the ``force`` interface (`#2588 <https://github.com/ros-controls/ros2_control/pull/2588>`_).
