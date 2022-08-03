^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2022-08-03)
-------------------

0.10.1 (2022-05-31)
-------------------

0.10.0 (2022-02-23)
-------------------
* added a fixed control period to loop (backport `#647 <https://github.com/ros-controls/ros2_control/issues/647>`_) (`#651 <https://github.com/ros-controls/ros2_control/issues/651>`_)
* Contributors: Jack Center, Denis Štogl

0.9.0 (2021-12-20)
------------------
* Make output of not available controllers nicer and make it informative. (`#577 <https://github.com/ros-controls/ros2_control/issues/577>`_) (`#578 <https://github.com/ros-controls/ros2_control/issues/578>`_)
  (cherry picked from commit a47a0347acd28a47b5095e4b206257e1520918ee)
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* feat: add colored output into spawner.py (`#560 <https://github.com/ros-controls/ros2_control/issues/560>`_) (`#563 <https://github.com/ros-controls/ros2_control/issues/563>`_)
  in order to highlight if controllers are loaded and configured and started.
  (cherry picked from commit afa0e01989e5b923e5c330f71d9270b30f03276d)
  Co-authored-by: Michael <50864015+fmros@users.noreply.github.com>
* Contributors: mergify[bot]

0.8.1 (2021-10-25)
------------------
* [ControllerManager] Fix method name upon fetching state of controller(s) (`#526 <https://github.com/ros-controls/ros2_control/issues/526>`_)
* Controller Manager should not crash when trying to start finalized or unconfigured controller (`#461 <https://github.com/ros-controls/ros2_control/issues/461>`_) (`#524 <https://github.com/ros-controls/ros2_control/issues/524>`_)
* Contributors: Denis Štogl, Lovro Ivanov

0.8.0 (2021-08-28)
------------------
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Use example urdf from the test_assests package. (`#495 <https://github.com/ros-controls/ros2_control/issues/495>`_)
* Separate controller manager test cases (`#476 <https://github.com/ros-controls/ros2_control/issues/476>`_)
* Add Controller Manager docs (`#467 <https://github.com/ros-controls/ros2_control/issues/467>`_)
* sort interfaces in resource manager (`#483 <https://github.com/ros-controls/ros2_control/issues/483>`_)
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Make controller_manager set controller's use_sim_time param when use_sim_time=True (`#468 <https://github.com/ros-controls/ros2_control/issues/468>`_)
  * potential solution to controller_manager use_sim_time sharing issue
  * removed debug print statements
  * added INFO message to warn user that use_sim_time is being set automatically
* Add load-only option into controller spawner (`#427 <https://github.com/ros-controls/ros2_control/issues/427>`_)
* Fixes for windows (`#443 <https://github.com/ros-controls/ros2_control/issues/443>`_)
  * Fix building on windows
  * Fix MSVC linker error when building tests
  * Fix hang when loading controller on windows
  * Use better log for configuring controller
  * Be consistent with visibility control
  * Use try_lock throw exception on failure
* Add an argument to define controller manager timeout (`#444 <https://github.com/ros-controls/ros2_control/issues/444>`_)
* Contributors: Akash, Bence Magyar, Darko Lukić, Denis Štogl, Karsten Knese, Simon Honigmann

0.7.1 (2021-06-15)
------------------
* Use namespace in controller_manager (`#435 <https://github.com/ros-controls/ros2_control/issues/435>`_)
* Contributors: Jonatan Olofsson

0.7.0 (2021-06-06)
------------------

0.6.1 (2021-05-31)
------------------
* Add missing dependency on controller_manager_msgs (`#426 <https://github.com/ros-controls/ros2_control/issues/426>`_)
* Contributors: Denis Štogl

0.6.0 (2021-05-23)
------------------
* List controller claimed interfaces (`#407 <https://github.com/ros-controls/ros2_control/issues/407>`_)
  * List controllers now also shows the claimed interfaces
  * Fixed tests that perform switches
  Successfull controller switches require more than one call to update()
  in order to update the controller list
  * Can now set the command interface configuration
  * Added checks for the claimed interfaces
* Contributors: Jordan Palacios

0.5.0 (2021-05-03)
------------------
* Make controller manager update rate optional (`#404 <https://github.com/ros-controls/ros2_control/issues/404>`_)
* Bump `wait_for_service` timeout to 10 seconds (`#403 <https://github.com/ros-controls/ros2_control/issues/403>`_)
* introduce --stopped for spawner (`#402 <https://github.com/ros-controls/ros2_control/issues/402>`_)
* hardware_interface mode switching using prepareSwitch doSwitch approach (`#348 <https://github.com/ros-controls/ros2_control/issues/348>`_)
* Avoid std::stringstream (`#391 <https://github.com/ros-controls/ros2_control/issues/391>`_)
* avoid deprecations (`#393 <https://github.com/ros-controls/ros2_control/issues/393>`_)
* Use RCLCPP_DEBUG_STREAM for char * (`#389 <https://github.com/ros-controls/ros2_control/issues/389>`_)
* Check controller_interface::init return value when loading (`#386 <https://github.com/ros-controls/ros2_control/issues/386>`_)
* Do not throw when controller type is not found, return nullptr instead (`#387 <https://github.com/ros-controls/ros2_control/issues/387>`_)
* Contributors: Auguste Bourgois, Karsten Knese, Matt Reynolds, Tyler Weaver, Mathias Hauan Arbo, Bence Magyar

0.4.0 (2021-04-07)
------------------
* Fix deprecation warnings: SUCCESS -> OK (`#375 <https://github.com/ros-controls/ros2_control/issues/375>`_)
* Don't use FileType for param-file (`#351 <https://github.com/ros-controls/ros2_control/issues/351>`_)
* Remodel ros2controlcli, refactor spawner/unspawner and fix test (`#349 <https://github.com/ros-controls/ros2_control/issues/349>`_)
* Add spawner and unspawner scripts (`#310 <https://github.com/ros-controls/ros2_control/issues/310>`_)
* Contributors: Bence Magyar, Jordan Palacios, Karsten Knese, Victor Lopez

0.3.0 (2021-03-21)
------------------
* release_interfaces when stopping controller (`#343 <https://github.com/ros-controls/ros2_control/issues/343>`_)
  * release_interfaces when stopping controller
  * Moved release_interfaces after deactivate
  * First attempt at test_release_interfaces
  * Switched to std::async with cm\_->update
* Capatalized error message and put the controllers name and resource name inside quote (`#338 <https://github.com/ros-controls/ros2_control/issues/338>`_)
* Contributors: mahaarbo, suab321321

0.2.1 (2021-03-02)
------------------

0.2.0 (2021-02-26)
------------------
* Add "Fake" components for simple integration of framework (`#323 <https://github.com/ros-controls/ros2_control/issues/323>`_)
* Contributors: Denis Štogl

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
