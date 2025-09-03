:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

controller_manager
******************
* The default strictness of the ``switch_controllers`` can now we be chosen using ROS 2 parameters. The default behaviour is still left to ``BEST_EFFORT`` (`#2168 <https://github.com/ros-controls/ros2_control/pull/2168>`_).
* Parameter ``shutdown_on_initial_state_failure`` was added to avoid shutting down on hardware initial state failure  (`#2230 <https://github.com/ros-controls/ros2_control/pull/2230>`_).
* The controller manager now supports switching (activating and deactivating) controllers in both realtime and non-realtime modes. This is controlled by the parameter ``activate_asap`` of the ``switch_controllers`` service (`#2452 <https://github.com/ros-controls/ros2_control/pull/2453>`_).
* The spawner now supports two new arguments ``--switch-asap`` and ``--no-switch-asap`` to control the behaviour of the spawner when switching controllers to be in realtime loop (or) non-realtime loop. By default, it is set to ``--no-switch-asap`` because when activating multiple controllers at same time might affect the realtime loop performance (`#2452 <https://github.com/ros-controls/ros2_control/pull/2453>`_).

hardware_interface
******************
* The ``prepare_command_mode_switch`` and ``perform_command_mode_switch`` methods will now only receive the start/stop interfaces that belong to the hardware component instead of everything (`#2120 <https://github.com/ros-controls/ros2_control/pull/2120>`_)

ros2controlcli
**************
* The CLI verbs ``list_hardware_components`` and ``list_hardware_interfaces`` will now show the data type used by the internal Command and State interfaces (`#2204 <https://github.com/ros-controls/ros2_control/pull/2204>`_).
