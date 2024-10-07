^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.18.0 (2024-10-07)
-------------------
* Adapt controller Reference/StateInterfaces to New Way of Exporting (variant support) (`#1689 <https://github.com/ros-controls/ros2_control/issues/1689>`_)
* Automatic Creation of Handles in HW, Adding Getters/Setters (variant support) (`#1688 <https://github.com/ros-controls/ros2_control/issues/1688>`_)
* [RM] Execute `error` callback of component on returning ERROR or with exception (`#1730 <https://github.com/ros-controls/ros2_control/issues/1730>`_)
* Contributors: Manuel Muth, Sai Kishor Kothakota

4.17.0 (2024-09-11)
-------------------
* Log exception type when catching the exception (`#1749 <https://github.com/ros-controls/ros2_control/issues/1749>`_)
* Fix spam of logs on failed hardware component initialization (`#1719 <https://github.com/ros-controls/ros2_control/issues/1719>`_)
* [HWItfs] Add key-value-storage to the InterfaceInfo (`#1421 <https://github.com/ros-controls/ros2_control/issues/1421>`_)
* Rename `get_state` and `set_state` Functions to `get/set_lifecylce_state` (variant support) (`#1683 <https://github.com/ros-controls/ros2_control/issues/1683>`_)
* Contributors: Manuel Muth, Sai Kishor Kothakota

4.16.1 (2024-08-24)
-------------------

4.16.0 (2024-08-22)
-------------------
* Use handle_name\_ variable instead of allocating for every `get_name` call (`#1706 <https://github.com/ros-controls/ros2_control/issues/1706>`_)
* Introduce Creation of Handles with InterfaceDescription (variant support) (`#1679 <https://github.com/ros-controls/ros2_control/issues/1679>`_)
* Preparation of Handles for Variant Support (`#1678 <https://github.com/ros-controls/ros2_control/issues/1678>`_)
* [RM] Decouple read/write cycles of each component with mutex to not block other components (`#1646 <https://github.com/ros-controls/ros2_control/issues/1646>`_)
* Contributors: Manuel Muth, Sai Kishor Kothakota

4.15.0 (2024-08-05)
-------------------
* [RM] Add `get_hardware_info` method to the Hardware Components (`#1643 <https://github.com/ros-controls/ros2_control/issues/1643>`_)
* add missing rclcpp logging include for Humble compatibility build (`#1635 <https://github.com/ros-controls/ros2_control/issues/1635>`_)
* Contributors: Sai Kishor Kothakota

4.14.0 (2024-07-23)
-------------------
* Unused header cleanup (`#1627 <https://github.com/ros-controls/ros2_control/issues/1627>`_)
* [ResourceManager] Make destructor virtual for use in derived classes (`#1607 <https://github.com/ros-controls/ros2_control/issues/1607>`_)
* Contributors: Henry Moore, Sai Kishor Kothakota

4.13.0 (2024-07-08)
-------------------
* [ResourceManager] Propagate access to logger and clock interfaces to HardwareComponent (`#1585 <https://github.com/ros-controls/ros2_control/issues/1585>`_)
* [ControllerChaining] Export state interfaces from chainable controllers (`#1021 <https://github.com/ros-controls/ros2_control/issues/1021>`_)
* Remove mimic parameter from ros2_control tag (`#1553 <https://github.com/ros-controls/ros2_control/issues/1553>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.12.0 (2024-07-01)
-------------------
* Add resources_lock\_ lock_guards to avoid race condition when loading robot_description through topic (`#1451 <https://github.com/ros-controls/ros2_control/issues/1451>`_)
* [RM] Rename `load_urdf` method to `load_and_initialize_components` and add error handling there to avoid stack crashing when error happens. (`#1354 <https://github.com/ros-controls/ros2_control/issues/1354>`_)
* Small improvements to the error output in component parser to make debugging easier. (`#1580 <https://github.com/ros-controls/ros2_control/issues/1580>`_)
* Fix link to gazebosim.org (`#1563 <https://github.com/ros-controls/ros2_control/issues/1563>`_)
* Add doc page about joint kinematics (`#1497 <https://github.com/ros-controls/ros2_control/issues/1497>`_)
* Bump version of pre-commit hooks (`#1556 <https://github.com/ros-controls/ros2_control/issues/1556>`_)
* [Feature] Hardware Components Grouping (`#1458 <https://github.com/ros-controls/ros2_control/issues/1458>`_)
* Contributors: Christoph Fröhlich, Dr. Denis, Sai Kishor Kothakota, github-actions[bot]

4.11.0 (2024-05-14)
-------------------
* Add find_package for ament_cmake_gen_version_h (`#1534 <https://github.com/ros-controls/ros2_control/issues/1534>`_)
* Parse URDF soft_limits into the HardwareInfo structure (`#1488 <https://github.com/ros-controls/ros2_control/issues/1488>`_)
* Contributors: Christoph Fröhlich, adriaroig

4.10.0 (2024-05-08)
-------------------
* Add hardware components exception handling in resource manager (`#1508 <https://github.com/ros-controls/ros2_control/issues/1508>`_)
* Working async controllers and components [not synchronized] (`#1041 <https://github.com/ros-controls/ros2_control/issues/1041>`_)
* Parse URDF joint hard limits into the HardwareInfo structure (`#1472 <https://github.com/ros-controls/ros2_control/issues/1472>`_)
* Add fallback controllers list to the ControllerInfo (`#1503 <https://github.com/ros-controls/ros2_control/issues/1503>`_)
* Add more common hardware interface type constants (`#1500 <https://github.com/ros-controls/ros2_control/issues/1500>`_)
* Contributors: Márk Szitanics, Sai Kishor Kothakota

4.9.0 (2024-04-30)
------------------
* Add missing calculate_dynamics (`#1498 <https://github.com/ros-controls/ros2_control/issues/1498>`_)
* Component parser: Get mimic information from URDF (`#1256 <https://github.com/ros-controls/ros2_control/issues/1256>`_)
* Contributors: Christoph Fröhlich

4.8.0 (2024-03-27)
------------------
* generate version.h file per package using the ament_generate_version_header  (`#1449 <https://github.com/ros-controls/ros2_control/issues/1449>`_)
* Contributors: Sai Kishor Kothakota

4.7.0 (2024-03-22)
------------------
* Codeformat from new pre-commit config (`#1433 <https://github.com/ros-controls/ros2_control/issues/1433>`_)
* Contributors: Christoph Fröhlich

4.6.0 (2024-03-02)
------------------
* Add -Werror=missing-braces to compile options (`#1423 <https://github.com/ros-controls/ros2_control/issues/1423>`_)
* [CI] Code coverage + pre-commit (`#1413 <https://github.com/ros-controls/ros2_control/issues/1413>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.5.0 (2024-02-12)
------------------
* Add missing export macros in lexical_casts.hpp (`#1382 <https://github.com/ros-controls/ros2_control/issues/1382>`_)
* Move hardware interface README content to sphinx documentation (`#1342 <https://github.com/ros-controls/ros2_control/issues/1342>`_)
* [Doc] Add documentation about initial_value regarding mock_hw (`#1352 <https://github.com/ros-controls/ros2_control/issues/1352>`_)
* Contributors: Felix Exner (fexner), Mateus Menezes, Silvio Traversaro

4.4.0 (2024-01-31)
------------------
* Move `test_components` to own package (`#1325 <https://github.com/ros-controls/ros2_control/issues/1325>`_)
* Fix controller parameter loading issue in different cases (`#1293 <https://github.com/ros-controls/ros2_control/issues/1293>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.3.0 (2024-01-20)
------------------
* [RM] Fix crash for missing urdf in resource manager (`#1301 <https://github.com/ros-controls/ros2_control/issues/1301>`_)
* Add additional checks for non existing and not available interfaces. (`#1218 <https://github.com/ros-controls/ros2_control/issues/1218>`_)
* Adding backward compatibility for string-to-double conversion (`#1284 <https://github.com/ros-controls/ros2_control/issues/1284>`_)
* [Doc] Make interface comments clearer in the doc strings. (`#1288 <https://github.com/ros-controls/ros2_control/issues/1288>`_)
* Fix return of ERROR and calls of cleanup when system is unconfigured of finalized (`#1279 <https://github.com/ros-controls/ros2_control/issues/1279>`_)
* fix the multiple definitions of lexical casts methods (`#1281 <https://github.com/ros-controls/ros2_control/issues/1281>`_)
* [ResourceManager] adds test for uninitialized hardware (`#1243 <https://github.com/ros-controls/ros2_control/issues/1243>`_)
* Use portable version for string-to-double conversion (`#1257 <https://github.com/ros-controls/ros2_control/issues/1257>`_)
* Fix typo in docs (`#1219 <https://github.com/ros-controls/ros2_control/issues/1219>`_)
* Contributors: Christoph Fröhlich, Dr. Denis, Maximilian Schik, Sai Kishor Kothakota, Stephanie Eng, bailaC

4.2.0 (2023-12-12)
------------------

4.1.0 (2023-11-30)
------------------
* Add few warning compiler options to error (`#1181 <https://github.com/ros-controls/ros2_control/issues/1181>`_)
* Contributors: Sai Kishor Kothakota

4.0.0 (2023-11-21)
------------------
* [MockHardware] Remove all deprecated options and deprecated plugins from the library. (`#1150 <https://github.com/ros-controls/ros2_control/issues/1150>`_)
* Contributors: Dr. Denis

3.21.0 (2023-11-06)
-------------------
* [MockHardware] Fix the issues where hardware with multiple interfaces can not be started because of a logical bug added when adding dynamics calculation functionality. (`#1151 <https://github.com/ros-controls/ros2_control/issues/1151>`_)
* Fix potential deadlock in ResourceManager (`#925 <https://github.com/ros-controls/ros2_control/issues/925>`_)
* Contributors: Christopher Wecht, Dr. Denis

3.20.0 (2023-10-31)
-------------------
* [ResourceManager] deactivate hardware from read/write return value (`#884 <https://github.com/ros-controls/ros2_control/issues/884>`_)
* Contributors: Felix Exner (fexner)

3.19.1 (2023-10-04)
-------------------

3.19.0 (2023-10-03)
-------------------
* [MockHardware] Added dynamic simulation functionality. (`#1028 <https://github.com/ros-controls/ros2_control/issues/1028>`_)
* Add GPIO tag description to docs (`#1109 <https://github.com/ros-controls/ros2_control/issues/1109>`_)
* Contributors: Christoph Fröhlich, Dr. Denis

3.18.0 (2023-08-17)
-------------------

3.17.0 (2023-08-07)
-------------------
* Add checks if hardware is initialized. (`#1054 <https://github.com/ros-controls/ros2_control/issues/1054>`_)
* Contributors: Dr. Denis

3.16.0 (2023-07-09)
-------------------

3.15.0 (2023-06-23)
-------------------
* Enable setting of initial state in HW compoments (`#1046 <https://github.com/ros-controls/ros2_control/issues/1046>`_)
* Ensure instantiation of hardware classes work for python bindings (`#1058 <https://github.com/ros-controls/ros2_control/issues/1058>`_)
* Contributors: Dr. Denis, Olivier Stasse

3.14.0 (2023-06-14)
-------------------
* Add -Wconversion flag to protect future developments (`#1053 <https://github.com/ros-controls/ros2_control/issues/1053>`_)
* [CM] Use `robot_description` topic instead of parameter and don't crash on empty URDF 🦿 (`#940 <https://github.com/ros-controls/ros2_control/issues/940>`_)
* [MockHardware] Enable disabling of command to simulate HW failures. (`#1027 <https://github.com/ros-controls/ros2_control/issues/1027>`_)
* enable ReflowComments to also use ColumnLimit on comments (`#1037 <https://github.com/ros-controls/ros2_control/issues/1037>`_)
* Docs: Use branch name substitution for all links (`#1031 <https://github.com/ros-controls/ros2_control/issues/1031>`_)
* [URDF Parser] Allow empty urdf tag, e.g., parameter (`#1017 <https://github.com/ros-controls/ros2_control/issues/1017>`_)
* Use consequently 'mock' instead of 'fake'. (`#1026 <https://github.com/ros-controls/ros2_control/issues/1026>`_)
* Contributors: Christoph Fröhlich, Dr. Denis, Felix Exner (fexner), Manuel Muth, Sai Kishor Kothakota, gwalck

3.13.0 (2023-05-18)
-------------------
* Add class for thread management of async hw interfaces (`#981 <https://github.com/ros-controls/ros2_control/issues/981>`_)
* Fix github links on control.ros.org (`#1019 <https://github.com/ros-controls/ros2_control/issues/1019>`_)
* Update precommit libraries(`#1020 <https://github.com/ros-controls/ros2_control/issues/1020>`_)
* Implement parse_bool and refactor a few (`#1014 <https://github.com/ros-controls/ros2_control/issues/1014>`_)
* docs: Fix link to hardware_components (`#1009 <https://github.com/ros-controls/ros2_control/issues/1009>`_)
* Contributors: Alejandro Bordallo, Christoph Fröhlich, Felix Exner (fexner), Márk Szitanics, mosfet80

3.12.2 (2023-04-29)
-------------------

3.12.1 (2023-04-14)
-------------------

3.12.0 (2023-04-02)
-------------------

3.11.0 (2023-03-22)
-------------------
* Check for missing hardware interfaces that use the gpio tag. (`#975 <https://github.com/ros-controls/ros2_control/issues/975>`_)
* Contributors: Ryan Sandzimier

3.10.0 (2023-03-16)
-------------------
* Split transmission interfaces (`#938 <https://github.com/ros-controls/ros2_control/issues/938>`_)
* Contributors: Noel Jiménez García

3.9.1 (2023-03-09)
------------------

3.9.0 (2023-02-28)
------------------

3.8.0 (2023-02-10)
------------------
* Fix CMake install so overriding works (`#926 <https://github.com/ros-controls/ros2_control/issues/926>`_)
* Async params (`#927 <https://github.com/ros-controls/ros2_control/issues/927>`_)
* Contributors: Márk Szitanics, Tyler Weaver

3.7.0 (2023-01-24)
------------------
* Make double parsing locale independent (`#921 <https://github.com/ros-controls/ros2_control/issues/921>`_)
* Contributors: Henning Kayser

3.6.0 (2023-01-12)
------------------
* 🔧 Fixes and updated on pre-commit hooks and their action (`#890 <https://github.com/ros-controls/ros2_control/issues/890>`_)
* Contributors: Denis Štogl

3.5.1 (2023-01-06)
------------------

3.5.0 (2022-12-06)
------------------
* ResourceManager doesn't always log an error on shutdown anymore (`#867 <https://github.com/ros-controls/ros2_control/issues/867>`_)
* Rename class type to plugin name #api-breaking #abi-breaking (`#780 <https://github.com/ros-controls/ros2_control/issues/780>`_)
* Contributors: Bence Magyar, Christopher Wecht

3.4.0 (2022-11-27)
------------------

3.3.0 (2022-11-15)
------------------
* [MockHardware] Enalbe initialization non-joint components(`#822 <https://github.com/ros-controls/ros2_control/issues/822>`_)
* Contributors: Felix Exner

3.2.0 (2022-10-15)
------------------
* [MockComponents] Rename 'fake_sensor_commands' to 'mock_sensor_commands' (`#782 <https://github.com/ros-controls/ros2_control/issues/782>`_)
* fix broken links (issue `#831 <https://github.com/ros-controls/ros2_control/issues/831>`_) (`#833 <https://github.com/ros-controls/ros2_control/issues/833>`_)
* Contributors: Kvk Praneeth, Manuel Muth, Bence Magyar, Denis Štogl

3.1.0 (2022-10-05)
------------------
* Cleanup Resource Manager a bit to increase clarity. (`#816 <https://github.com/ros-controls/ros2_control/issues/816>`_)
* Handle hardware errors in Resource Manager (`#805 <https://github.com/ros-controls/ros2_control/issues/805>`_)
  * Add code for deactivating controller when hardware gets an error on read and write.
* Contributors: Denis Štogl

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

2.12.1 (2022-07-14)
-------------------
* Fix fake components deprecation and add test for it (`#771 <https://github.com/ros-controls/ros2_control/issues/771>`_)
* Contributors: Bence Magyar

2.12.0 (2022-07-09)
-------------------
* Hardware interface specific update rate and best practices about it (`#716 <https://github.com/ros-controls/ros2_control/issues/716>`_)
* Deprecate fake components, long live mock components (`#762 <https://github.com/ros-controls/ros2_control/issues/762>`_)
* Contributors: Bence Magyar, Lovro Ivanov

2.11.0 (2022-07-03)
-------------------
* [Interfaces] Improved ```get_name()``` method of hardware interfaces #api-breaking (`#737 <https://github.com/ros-controls/ros2_control/issues/737>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Remove ament autolint (`#749 <https://github.com/ros-controls/ros2_control/issues/749>`_)
* Full functionality of chainable controllers in controller manager (`#667 <https://github.com/ros-controls/ros2_control/issues/667>`_)
  * auto-switching of chained mode in controllers
  * interface-matching approach for managing chaining controllers
* Contributors: Bence Magyar, Denis Štogl, Lucas Schulze

2.10.0 (2022-06-18)
-------------------
* Make RHEL CI happy! (`#730 <https://github.com/ros-controls/ros2_control/issues/730>`_)
* CMakeLists cleanup (`#733 <https://github.com/ros-controls/ros2_control/issues/733>`_)
* Refactored error handling when hardware name is duplicated (`#724 <https://github.com/ros-controls/ros2_control/issues/724>`_)
* Update to clang format 12 (`#731 <https://github.com/ros-controls/ros2_control/issues/731>`_)
* Contributors: Andy Zelenak, Bence Magyar, Kvk Praneeth, Márk Szitanics

2.9.0 (2022-05-19)
------------------
* Resource Manager extension to support management of reference interfaces from chained controllers. (`#664 <https://github.com/ros-controls/ros2_control/issues/664>`_)
  * Extend resource manager to manage reference interfaces from controllers.
  * Adjust interface between CM and RM for managing controllers' reference interfaces.
* Contributors: Denis Štogl

2.8.0 (2022-05-13)
------------------
* Pass time and period to read() and write() (`#715 <https://github.com/ros-controls/ros2_control/issues/715>`_)
* Contributors: Bence Magyar

2.7.0 (2022-04-29)
------------------
* Make URDF available to HW components on initialize (`#709 <https://github.com/ros-controls/ros2_control/issues/709>`_)
* Contributors: Bence Magyar

2.6.0 (2022-04-20)
------------------
* Error if a hardware name is duplicated (`#672 <https://github.com/ros-controls/ros2_control/issues/672>`_)
* Port four bar linkage and differential transmission loaders from ROS1 (`#656 <https://github.com/ros-controls/ros2_control/issues/656>`_)
* Contributors: Andy Zelenak, Márk Szitanics

2.5.0 (2022-03-25)
------------------
* Require lifecycle-msgs in hardware_interface package (`#675 <https://github.com/ros-controls/ros2_control/issues/675>`_) (`#678 <https://github.com/ros-controls/ros2_control/issues/678>`_)
* Using should be inside namespace and not global scope. (`#673 <https://github.com/ros-controls/ros2_control/issues/673>`_)
* Modernize C++: Use for-each loops in Resource Manager. (`#659 <https://github.com/ros-controls/ros2_control/issues/659>`_)
* Enable controller manager services to control hardware lifecycle #abi-breaking (`#637 <https://github.com/ros-controls/ros2_control/issues/637>`_)
  * Implement CM services for hardware lifecycle management.
  * Added default behavior to activate all controller and added description of CM parameters.
* Contributors: Denis Štogl

2.4.0 (2022-02-23)
------------------
* Fix transmission loader tests (`#642 <https://github.com/ros-controls/ros2_control/issues/642>`_)
* Contributors: Bence Magyar, Denis Štogl

2.3.0 (2022-02-18)
------------------
* Add a warning if an initial_value is not found for any interface (`#623 <https://github.com/ros-controls/ros2_control/issues/623>`_)
* Contributors: AndyZe

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
* Doc 📓: Add detailed explanation about writing new hardware interface.  (`#615 <https://github.com/ros-controls/ros2_control/issues/615>`_)
* Contributors: Denis Štogl

2.1.0 (2022-01-11)
------------------
* Removing 'auto' from function definition to support pre c++ 20 (`#608 <https://github.com/ros-controls/ros2_control/issues/608>`_)
* Support of "initial_value" for the 'FakeSystem' (`#598 <https://github.com/ros-controls/ros2_control/issues/598>`_)
* Contributors: bailaC, Denis Štogl

2.0.0 (2021-12-29)
------------------
* Adding support for 'initial_value' parameter. (`#593 <https://github.com/ros-controls/ros2_control/issues/593>`_)
* fix copy paste error in documentation (`#594 <https://github.com/ros-controls/ros2_control/issues/594>`_)
* Use lambda functions in ros2_control generic_system for repetitive tasks (`#579 <https://github.com/ros-controls/ros2_control/issues/579>`_)
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* Extend FakeHardware to support <gpio>-tag (`#574 <https://github.com/ros-controls/ros2_control/issues/574>`_)
* Contributors: Michael, bailaC, Denis Štogl

1.2.0 (2021-11-05)
------------------
* Import and Initialize components (`#566 <https://github.com/ros-controls/ros2_control/issues/566>`_)
* Contributors: Alejandro Hernández Cordero

1.1.0 (2021-10-25)
------------------
* Handle errors of hardware that happen on read and write. (`#546 <https://github.com/ros-controls/ros2_control/issues/546>`_)
* Contributors: Denis Štogl, Mathias Aarbo

1.0.0 (2021-09-29)
------------------
* Hardware components extension for lifecycle support (`#503 <https://github.com/ros-controls/ros2_control/issues/503>`_)
* add M_PI macro for windows in test_component_parser.cpp (`#502 <https://github.com/ros-controls/ros2_control/issues/502>`_)
* Extend GenericSystem by adding mapping of position with offset to custom interface. (`#469 <https://github.com/ros-controls/ros2_control/issues/469>`_)
* Remove BOOST compiler definitions for pluginlib from CMakeLists (`#514 <https://github.com/ros-controls/ros2_control/issues/514>`_)
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* Contributors: Bence Magyar, Denis Štogl, dzyGIT

0.8.0 (2021-08-28)
------------------
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Fixup doc typo (`#492 <https://github.com/ros-controls/ros2_control/issues/492>`_)
* Add docs for fake components (`#466 <https://github.com/ros-controls/ros2_control/issues/466>`_)
* sort interfaces in resource manager (`#483 <https://github.com/ros-controls/ros2_control/issues/483>`_)
* fix format (`#484 <https://github.com/ros-controls/ros2_control/issues/484>`_)
* Transmission parsing v2 (`#471 <https://github.com/ros-controls/ros2_control/issues/471>`_)
  * move parsing responsibility to hardware_interface
  * parse transmission type
  * Cleanup unused parser
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Extended GenericSystem with state offset options for testing some special control cases. (`#350 <https://github.com/ros-controls/ros2_control/issues/350>`_)
  * Extended GenericSystem with state offset options for testing some special control cases.
  * Better parameter name
  * Apply offset only to position interfaces.
* Added GPIO parsing and test (`#436 <https://github.com/ros-controls/ros2_control/issues/436>`_)
* Fixes for windows (`#443 <https://github.com/ros-controls/ros2_control/issues/443>`_)
  * Fix building on windows
  * Fix MSVC linker error when building tests
  * Fix hang when loading controller on windows
  * Use better log for configuring controller
  * Be consistent with visibility control
  * Use try_lock throw exception on failure
* Contributors: Akash, Bence Magyar, Denis Štogl, Karsten Knese, Mathias Arbo, Jafar Abdi

0.7.1 (2021-06-15)
------------------
* [FakeSystem] Set default command interface to NaN (`#424 <https://github.com/ros-controls/ros2_control/issues/424>`_)
* Contributors: Denis Štogl, Bence Magyar

0.7.0 (2021-06-06)
------------------
* Add FTS as first semantic components to simplify controllers. (`#370 <https://github.com/ros-controls/ros2_control/issues/370>`_)
* Contributors: bailaC, Denis Štogl, Jordan Palacios

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------
* Remove the with_value_ptr and class templatization for ReadOnlyHandle (`#379 <https://github.com/ros-controls/ros2_control/issues/379>`_)
* fake_components: Add mimic joint to generic system (`#409 <https://github.com/ros-controls/ros2_control/issues/409>`_)
* List controller claimed interfaces (`#407 <https://github.com/ros-controls/ros2_control/issues/407>`_)
* Contributors: El Jawad Alaa, Jafar Abdi, Jordan Palacios, Bence Magyar

0.5.0 (2021-05-03)
------------------
* Make hardware interface types as const char array rather than const char pointer (`#408 <https://github.com/ros-controls/ros2_control/issues/408>`_)
* use auto instead of uint (`#398 <https://github.com/ros-controls/ros2_control/issues/398>`_)
* hardware_interface mode switching using prepareSwitch doSwitch approach (`#348 <https://github.com/ros-controls/ros2_control/issues/348>`_)
* avoid deprecations (`#393 <https://github.com/ros-controls/ros2_control/issues/393>`_)
* move deprecation note before function definition instead of inside (`#381 <https://github.com/ros-controls/ros2_control/issues/381>`_)
* Replace standard interfaces' hard-coded strings by constants (`#376 <https://github.com/ros-controls/ros2_control/issues/376>`_)
* add deprecation note for with_value_ptr (`#378 <https://github.com/ros-controls/ros2_control/issues/378>`_)
* Contributors: El Jawad Alaa, Jafar Abdi, Karsten Knese, Mateus Amarante, Mathias Hauan Arbo, Bence Magyar

0.4.0 (2021-04-07)
------------------
* [ros2_control_test_assets] Fix typo (`#371 <https://github.com/ros-controls/ros2_control/issues/371>`_)
* uint -> size_t, 0u and auto (`#346 <https://github.com/ros-controls/ros2_control/issues/346>`_)
* Contributors: Karsten Knese, Yutaka Kondo

0.3.0 (2021-03-21)
------------------
* Capatalized error message and put the controllers name and resource name inside quote (`#338 <https://github.com/ros-controls/ros2_control/issues/338>`_)
* Parse True and true in fakesystem, touch up variable name
* Contributors: Denis Štogl, suab321321

0.2.1 (2021-03-02)
------------------
* Remove unused include (`#336 <https://github.com/ros-controls/ros2_control/issues/336>`_)
* Contributors: Bence Magyar

0.2.0 (2021-02-26)
------------------
* Add "Fake" components for simple integration of framework (`#323 <https://github.com/ros-controls/ros2_control/issues/323>`_)
* Contributors: Denis Štogl

0.1.6 (2021-02-05)
------------------
* correct hardware interface validation in resource manager. (`#317 <https://github.com/ros-controls/ros2_control/issues/317>`_)
* Contributors: Karsten Knese

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------
* Add test assets package (`#289 <https://github.com/ros-controls/ros2_control/issues/289>`_)
* update doxygen style according to ros2 core standard (`#300 <https://github.com/ros-controls/ros2_control/issues/300>`_)
* Move test_components from test_robot_hardware to hardware_interface package (`#288 <https://github.com/ros-controls/ros2_control/issues/288>`_)
* Contributors: Denis Štogl, João Victor Torres Borges

0.1.3 (2021-01-21)
------------------

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
* Added starting of resources into CM and RM (`#240 <https://github.com/ros-controls/ros2_control/issues/240>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* Use constants instead of strings in tests (`#241 <https://github.com/ros-controls/ros2_control/issues/241>`_)
* resource loaning (`#224 <https://github.com/ros-controls/ros2_control/issues/224>`_)
* Allocate memory for components and handles (`#207 <https://github.com/ros-controls/ros2_control/issues/207>`_)
* rename command/state handles to command/state interfaces (`#223 <https://github.com/ros-controls/ros2_control/issues/223>`_)
* Remodel component interfaces (`#203 <https://github.com/ros-controls/ros2_control/issues/203>`_)
* adapt component parser to new xml schema (`#209 <https://github.com/ros-controls/ros2_control/issues/209>`_)
* remove logical components, move hardware resources (`#201 <https://github.com/ros-controls/ros2_control/issues/201>`_)
* Replace rclcpp by rcutils logging tools in hardware_interface pkg (`#205 <https://github.com/ros-controls/ros2_control/issues/205>`_)
* Add a struct for Interface information, update the test URDF (`#167 <https://github.com/ros-controls/ros2_control/issues/167>`_)
* Add virtual modifier to the functions of Joint and Sensor component (`#178 <https://github.com/ros-controls/ros2_control/issues/178>`_)
* Hide component parser api (`#157 <https://github.com/ros-controls/ros2_control/issues/157>`_)
* Remove old joint state and joint command handles (`#134 <https://github.com/ros-controls/ros2_control/issues/134>`_)
* New version of component parser (`#127 <https://github.com/ros-controls/ros2_control/issues/127>`_)
* Dynamic joint handles (`#125 <https://github.com/ros-controls/ros2_control/issues/125>`_)
* Hardware component interfaces (`#121 <https://github.com/ros-controls/ros2_control/issues/121>`_)
* Add ActuatorHandle and Implement string-based interface handle-handling using DynamicJointState message (`#112 <https://github.com/ros-controls/ros2_control/issues/112>`_)
* Change Hardware return type to enum class (`#114 <https://github.com/ros-controls/ros2_control/issues/114>`_)
* Replace RCUTILS\_ with RCLCPP\_ for logging (`#62 <https://github.com/ros-controls/ros2_control/issues/62>`_)
* import hardware_interface
* Contributors: Andreas Klintberg, Andy Zelenak, Bence Magyar, Colin MacKenzie, Denis Štogl, Jafar Abdi, Jordan Palacios, Karsten Knese, Mateus Amarante, Matthew Reynolds, Victor Lopez, Yutaka Kondo
