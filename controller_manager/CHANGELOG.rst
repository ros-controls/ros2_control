^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2021-02-05)
------------------

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------
* fix float conversion warning (`#312 <https://github.com/ros-controls/ros2_control/issues/312>`_)
* update doxygen style according to ros2 core standard (`#300 <https://github.com/ros-controls/ros2_control/issues/300>`_)
* Capitalized messages in controller_manager.cpp upto line669 (`#285 <https://github.com/ros-controls/ros2_control/issues/285>`_)
* Sleep accurate duration on ros2_control_node (`#302 <https://github.com/ros-controls/ros2_control/issues/302>`_)
* Contributors: Achinta-Choudhury, João Victor Torres Borges, Karsten Knese, Yutaka Kondo

0.1.3 (2021-01-21)
------------------
* Fix building on macOS with clang (`#292 <https://github.com/ros-controls/ros2_control/issues/292>`_)
ail.com>
* Contributors: Karsten Knese

0.1.2 (2021-01-06)
------------------
* Fix update rate issues by working around MutliThreadedExecutor (`#275 <https://github.com/ros-controls/ros2_control/issues/275>`_)
  * Fix update rate issues by working around MutliThreadedExecutor
  Currently the MutliThreadedExecutor performance is very bad. This leads
  to controllers not meeting their update rate. This PR is a temporary
  workaround for these issues.
  The current approach uses a `rclcpp` timer to execute the control loop.
  When used in combination with the `MutliThreadedExecutor`, the timers
  are not execute at their target frequency. I've converted the control
  loop to a while loop on a separate thread that uses `nanosleep` to
  execute the correct update rate. This means that `rclcpp` is not
  involved in the execution and leads to much better performance.
  * Address review comments by rewriting several comments
* Contributors: Ramon Wijnands

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
* Add configure controller service (`#272 <https://github.com/ros-controls/ros2_control/issues/272>`_)
* Remove lifecycle node (`#261 <https://github.com/ros-controls/ros2_control/issues/261>`_)
* Added starting of resources into CM and RM (`#240 <https://github.com/ros-controls/ros2_control/issues/240>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* Remove pluginlib warnings on reload test (`#237 <https://github.com/ros-controls/ros2_control/issues/237>`_)
* resource loaning (`#224 <https://github.com/ros-controls/ros2_control/issues/224>`_)
* Allocate memory for components and handles (`#207 <https://github.com/ros-controls/ros2_control/issues/207>`_)
* Add controller manager services (`#139 <https://github.com/ros-controls/ros2_control/issues/139>`_)
* Change Hardware return type to enum class (`#114 <https://github.com/ros-controls/ros2_control/issues/114>`_)
* Use rclcpp::Executor instead of rclcpp::executor::Executor(deprecated) (`#82 <https://github.com/ros-controls/ros2_control/issues/82>`_)
* Replace RCUTILS\_ with RCLCPP\_ for logging (`#62 <https://github.com/ros-controls/ros2_control/issues/62>`_)
* dont include pluginlib header in controller manager header (`#63 <https://github.com/ros-controls/ros2_control/issues/63>`_)
* export controller_interface (`#58 <https://github.com/ros-controls/ros2_control/issues/58>`_)
* Use pluginlib instead of class_loader for loading controllers (`#41 <https://github.com/ros-controls/ros2_control/issues/41>`_)
* import controller_manager
* Contributors: Bence Magyar, Denis Štogl, Jafar Abdi, Jordan Palacios, Karsten Knese, Parth Chopra, Victor Lopez
