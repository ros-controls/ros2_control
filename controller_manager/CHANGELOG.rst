^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.5.1 (2023-01-06)
------------------
* Prevent controller manager from crashing when controller's plugin has error during loading. (`#881 <https://github.com/ros-controls/ros2_control/issues/881>`_)
* Contributors: Denis Štogl

3.5.0 (2022-12-06)
------------------
* Rename class type to plugin name #api-breaking #abi-breaking (`#780 <https://github.com/ros-controls/ros2_control/issues/780>`_)
* Namespace Loaded Controllers (`#852 <https://github.com/ros-controls/ros2_control/issues/852>`_)
* Contributors: Bence Magyar, sp-sophia-labs

3.4.0 (2022-11-27)
------------------
* Use a thread priority library from realtime_tools (`#794 <https://github.com/ros-controls/ros2_control/issues/794>`_)
* [Doc] Correct type of update_rate parameter (`#858 <https://github.com/ros-controls/ros2_control/issues/858>`_)
* Contributors: Andy Zelenak, Denis Štogl, Bence Magyar

3.3.0 (2022-11-15)
------------------
* Adding activation/deactivation tests for chain controllers (`#809 <https://github.com/ros-controls/ros2_control/issues/809>`_)
* Fix const-ness in std::chrono::time_point construction and explicitly use std::chrono::nanoseconds as std::chrono::time_point template parameter to help compilation on macOS as its std::chrono::system_clock::time_point defaults to std::chrono::milliseconds for duration type (`#848 <https://github.com/ros-controls/ros2_control/issues/848>`_)
* [ControllerManager] Fix wrong initialization order and avoid compiler warnings (`#836 <https://github.com/ros-controls/ros2_control/issues/836>`_)
* Contributors: Adrian Zwiener, Bilal Gill, Felix Exner, light-tech

3.2.0 (2022-10-15)
------------------

3.1.0 (2022-10-05)
------------------
* Don't ask to export reference interface if controller not 'inactive' or 'active' (`#824 <https://github.com/ros-controls/ros2_control/issues/824>`_)
* Add diagnostics for controllers activity (`#820 <https://github.com/ros-controls/ros2_control/issues/820>`_)
* Search for controller manager in the same namespace as spawner (`#810 <https://github.com/ros-controls/ros2_control/issues/810>`_)
* Handle HW errors on read and write in CM by stopping controllers (`#742 <https://github.com/ros-controls/ros2_control/issues/742>`_)
  Add code for deactivating controller when hardware gets an error on read and write.
  Fix misleading variable name in the tests.
  Remove all interface from available list for hardware when an error happens.
  Do some more variable renaming to the new nomenclature.
* Contributors: Denis Štogl, Tony Najjar

3.0.0 (2022-09-19)
------------------

2.15.0 (2022-09-19)
-------------------

2.14.0 (2022-09-04)
-------------------
* Add doxygen comments (`#777 <https://github.com/ros-controls/ros2_control/issues/777>`_)
* Contributors: Bence Magyar, Denis Štogl

2.13.0 (2022-08-03)
-------------------
* Clang tidy: delete a redundant return (`#790 <https://github.com/ros-controls/ros2_control/issues/790>`_)
* Add chained controllers information in list controllers service #abi-braking (`#758 <https://github.com/ros-controls/ros2_control/issues/758>`_)
  * add chained controllers in ros2controlcli
  * remove controller_group from service
  * added comments to ControllerState message
  * added comments to ChainedConnection message
* spawner.py: Fix python logging on deprecation warning (`#787 <https://github.com/ros-controls/ros2_control/issues/787>`_)
* Add documentation for realtime permission (`#781 <https://github.com/ros-controls/ros2_control/issues/781>`_)
* Fix bug in spawner with getter for node's logger (`#776 <https://github.com/ros-controls/ros2_control/issues/776>`_)
* Contributors: Andy Zelenak, Felix Exner, Paul Gesel, Bijou Abraham

2.12.1 (2022-07-14)
-------------------
* Rename CM members from start/stop to activate/deactivate nomenclature. (`#756 <https://github.com/ros-controls/ros2_control/issues/756>`_)
* Fix spelling in comment (`#769 <https://github.com/ros-controls/ros2_control/issues/769>`_)
* Contributors: Denis Štogl, Tyler Weaver

2.12.0 (2022-07-09)
-------------------
* Deprecate and rename `start` and `stop` nomenclature toward user to `activate` and `deactivate` #ABI-breaking (`#755 <https://github.com/ros-controls/ros2_control/issues/755>`_)
  * Rename fields and deprecate old nomenclature.
  * Add new defines to SwitchController.srv
  * Deprecated start/stop nomenclature in all CLI commands.
  * Deprecate 'start_asap' too as other fields.
* [ros2_control_node] Automatically detect if RT kernel is used and opportunistically enable SCHED_FIFO (`#748 <https://github.com/ros-controls/ros2_control/issues/748>`_)
* Contributors: Denis Štogl, Tyler Weaver

2.11.0 (2022-07-03)
-------------------
* Remove hybrid services in controller manager. (`#761 <https://github.com/ros-controls/ros2_control/issues/761>`_)
* [Interfaces] Improved ```get_name()``` method of hardware interfaces #api-breaking (`#737 <https://github.com/ros-controls/ros2_control/issues/737>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Fix test dependency for chainable test (`#751 <https://github.com/ros-controls/ros2_control/issues/751>`_)
* Remove ament autolint (`#749 <https://github.com/ros-controls/ros2_control/issues/749>`_)
* Full functionality of chainable controllers in controller manager (`#667 <https://github.com/ros-controls/ros2_control/issues/667>`_)
  * auto-switching of chained mode in controllers
  * interface-matching approach for managing chaining controllers
* Fixup spanwer and unspawner tests. It changes spawner a bit to handle interupts internally. (`#745 <https://github.com/ros-controls/ros2_control/issues/745>`_)
* Add missing field to initializer lists in tests (`#746 <https://github.com/ros-controls/ros2_control/issues/746>`_)
* Small but useful output update on controller manager. (`#741 <https://github.com/ros-controls/ros2_control/issues/741>`_)
* Fixed period passed to hardware components always 0 (`#738 <https://github.com/ros-controls/ros2_control/issues/738>`_)
* Contributors: Bence Magyar, Denis Štogl, Maciej Bednarczyk, Lucas Schulze

2.10.0 (2022-06-18)
-------------------
* Make RHEL CI happy! (`#730 <https://github.com/ros-controls/ros2_control/issues/730>`_)
* CMakeLists cleanup (`#733 <https://github.com/ros-controls/ros2_control/issues/733>`_)
* Update to clang format 12 (`#731 <https://github.com/ros-controls/ros2_control/issues/731>`_)
* Contributors: Andy Zelenak, Bence Magyar, Márk Szitanics

2.9.0 (2022-05-19)
------------------
* Adding base class for chained controllers: `ChainedControllersInterface` (`#663 <https://github.com/ros-controls/ros2_control/issues/663>`_)
  * Extending ControllerInterface with methods for chainable controllers.
  * Switching to chained_mode is only forbidden if controller is active.
  * Default implementation for 'on_set_chained_mode' method.
  * Use two internal methods instead of 'update' directly on chained controllers.
* Add ControllerInterfaceBase class with methods for chainable controller (`#717 <https://github.com/ros-controls/ros2_control/issues/717>`_)
* Contributors: Denis Štogl

2.8.0 (2022-05-13)
------------------
* Pass time and period to read() and write() (`#715 <https://github.com/ros-controls/ros2_control/issues/715>`_)
* Contributors: Bence Magyar

2.7.0 (2022-04-29)
------------------
* Update ControllerManager documenation describing some concepts (`#677 <https://github.com/ros-controls/ros2_control/issues/677>`_)
* Make node private in ControllerInterface (`#699 <https://github.com/ros-controls/ros2_control/issues/699>`_)
* Contributors: Chen Bainian, Denis Štogl, Jack Center, Bence Magyar

2.6.0 (2022-04-20)
------------------
* Add controller_manager_msgs dependency to test_hardware_management_srvs (`#702 <https://github.com/ros-controls/ros2_control/issues/702>`_)
* Remove unused variable from the test (`#700 <https://github.com/ros-controls/ros2_control/issues/700>`_)
* Enable namespaces for controllers. (`#693 <https://github.com/ros-controls/ros2_control/issues/693>`_)
* Spawner waits for services (`#683 <https://github.com/ros-controls/ros2_control/issues/683>`_)
* Contributors: Denis Štogl, Rufus Wong, Tyler Weaver

2.5.0 (2022-03-25)
------------------
* Make ControllerManager tests more flexible and reusable for different scenarios. Use more parameterized tests regarding strictness. (`#661 <https://github.com/ros-controls/ros2_control/issues/661>`_)
* Use lifecycle nodes in controllers again (`#538 <https://github.com/ros-controls/ros2_control/issues/538>`_)
  * Add lifecycle nodes
  * Add custom 'configure' to controller interface to get 'update_rate' parameter.
  * Disable external interfaces of LifecycleNode.
* Small fixes in controller manager tests. (`#660 <https://github.com/ros-controls/ros2_control/issues/660>`_)
* Enable controller manager services to control hardware lifecycle #abi-breaking (`#637 <https://github.com/ros-controls/ros2_control/issues/637>`_)
  * Implement CM services for hardware lifecycle management.
  * Added default behavior to activate all controller and added description of CM parameters.
* Contributors: Denis Štogl, Vatan Aksoy Tezer, Bence Magyar

2.4.0 (2022-02-23)
------------------
* Fixes of issue with seg-fault when checking interfaces on unconfigured controllers. (`#580 <https://github.com/ros-controls/ros2_control/issues/580>`_)
* Update CM service QoS so that we don't lose service calls when using many controllers. (`#643 <https://github.com/ros-controls/ros2_control/issues/643>`_)
* Contributors: Denis Štogl, Bence Magyar

2.3.0 (2022-02-18)
------------------
* added a fixed control period to loop (`#647 <https://github.com/ros-controls/ros2_control/issues/647>`_)
* install spawner/unspawner using console_script entrypoint (`#607 <https://github.com/ros-controls/ros2_control/issues/607>`_)
* Add BEST_EFFORT in the controller switch tests. (`#582 <https://github.com/ros-controls/ros2_control/issues/582>`_)
* Resolve unused parameter warnings (`#636 <https://github.com/ros-controls/ros2_control/issues/636>`_)
* Contributors: Bence Magyar, Denis Štogl, Jack Center, Melvin Wang, Xi-Huang

2.2.0 (2022-01-24)
------------------
* Resource Manager API changes for hardware lifecycle #api-breaking #abi-breaking (`#589 <https://github.com/ros-controls/ros2_control/issues/589>`_)
  * Towards selective starting and stoping of hardware components. Cleaning and renaming.
  * Move Lifecycle of hardware component to the bottom for better overview.
  * Use the same nomenclature as for controllers. 'start' -> 'activate'; 'stop' -> 'deactivate'
  * Add selective starting and stopping of hardware resources.
  Add HardwareComponentInfo structure in resource manager.
  Use constants for HW parameters in tests of resource_manager.
  Add list hardware components in CM to get details about them and check their status.
  Use clear name for 'guard' and move release cmd itfs for better readability.
  RM: Add lock for accesing maps with stored interfaces.
  Separate hardware components-related services after controllers-related services.
  Add service for activate/deactive hardware components.
  Add activation and deactivation through ResourceStorage. This helps to manage available command interfaces.
  * Use lifecycle_msgs/State in ListHardwareCompoents for state representation.
  * Simplify repeatable code in methods.
  * Add HW shutdown structure into ResouceManager.
  * Fill out service callback in CM and add parameter for auto-configure.
  * Move claimed_command_itf_map to ResourceStorage from ResourceManager.
  * Do not automatically configure hardware in RM.
  * Lifecycle and claiming in Resource Manager is working.
  * Extend controller manager to support HW lifecycle.
  * Add also available and claimed status into list components service output.
  * Add SetHardwareComponentState service.
  * Make all output in services debug-output.
  * Remove specific services for hardware lifecycle management and leave only 'set_hardware_component_state' service.
  * Make init_resource_manager less stateful.
  * Keep old api to start/activate all components per default.
  * Remove 'moving'/'non-moving' interface-handling.
  * Remove obsolete 'import_components' methods without hardware info and fix post_initialization test.
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* Contributors: Denis Štogl

2.1.0 (2022-01-11)
------------------

2.0.0 (2021-12-29)
------------------
* Add service-skeletons for controlling hardware lifecycle. (`#585 <https://github.com/ros-controls/ros2_control/issues/585>`_)
* fix get_update_rate visibility in windows (`#586 <https://github.com/ros-controls/ros2_control/issues/586>`_)
* Make output of not available controller nicer and make it informational. (`#577 <https://github.com/ros-controls/ros2_control/issues/577>`_)
* Contributors: Denis Štogl, Melvin Wang

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------
* feat: add colored output into spawner.py (`#560 <https://github.com/ros-controls/ros2_control/issues/560>`_)
* Added timeout argument for service_caller timeout (`#552 <https://github.com/ros-controls/ros2_control/issues/552>`_)
* controller_manager: Use command_interface_configuration for the claimed interfaces when calling list_controllers (`#544 <https://github.com/ros-controls/ros2_control/issues/544>`_)
* Clean up test_load_controller (`#532 <https://github.com/ros-controls/ros2_control/issues/532>`_)
* Contributors: Jack Center, Jafar Abdi, Michael, Nour Saeed

1.0.0 (2021-09-29)
------------------
* Use ControllerManager node clock for control loop timepoints (`#542 <https://github.com/ros-controls/ros2_control/issues/542>`_)
* Per controller update rate(`#513 <https://github.com/ros-controls/ros2_control/issues/513>`_)
* added dt to controller interface and controller manager `#438 <https://github.com/ros-controls/ros2_control/issues/438>`_ (`#520 <https://github.com/ros-controls/ros2_control/issues/520>`_)
* Update nomenclature in CM for better code and output understanding (`#517 <https://github.com/ros-controls/ros2_control/issues/517>`_)
* Methods controlling the lifecycle of controllers all have on\_ prefix
* Controller Manager should not crash when trying to start finalized or unconfigured controller (`#461 <https://github.com/ros-controls/ros2_control/issues/461>`_)
* Fix deprecation warning from rclcpp::Duration (`#511 <https://github.com/ros-controls/ros2_control/issues/511>`_)
* Remove BOOST compiler definitions for pluginlib from CMakeLists (`#514 <https://github.com/ros-controls/ros2_control/issues/514>`_)
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* Refactor INSTANTIATE_TEST_CASE_P -> INSTANTIATE_TEST_SUITE_P (`#515 <https://github.com/ros-controls/ros2_control/issues/515>`_)
  Also removed the duplicated format & compiler fixes as on Galactic this shouldn't be an issue
* rename get_current_state() to get_state() (`#512 <https://github.com/ros-controls/ros2_control/issues/512>`_)
* Fix spawner tests (`#509 <https://github.com/ros-controls/ros2_control/issues/509>`_)
* Removed deprecated CLI verbs (`#420 <https://github.com/ros-controls/ros2_control/issues/420>`_)
* Remove extensions from executable nodes (`#453 <https://github.com/ros-controls/ros2_control/issues/453>`_)
* Contributors: Bence Magyar, Denis Štogl, Dmitri Ignakov, Joseph Schornak, Márk Szitanics, Tim Clephas, bailaC, Mathias Aarbo

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
