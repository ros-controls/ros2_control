:github_url: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

controller_manager
******************
* The default strictness of the ``swtich_controllers`` can now we be chosen using ROS 2 parameters. The default behaviour is still left to ``BEST_EFFORT`` (`#2168 <https://github.com/ros-controls/ros2_control/pull/2168>`_).

ros2controlcli
**************
* The CLI verbs ``list_hardware_components`` and ``list_hardware_interfaces`` will now show the data type used by the internal Command and State interfaces (`#2204 <https://github.com/ros-controls/ros2_control/pull/2204>`_).
