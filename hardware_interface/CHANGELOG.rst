^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.43.1 (2024-09-11)
-------------------

2.43.0 (2024-08-22)
-------------------

2.42.0 (2024-07-23)
-------------------
* Small improvements to the error output in component parser to make debugging easier. (backport `#1580 <https://github.com/ros-controls/ros2_control/issues/1580>`_) (`#1581 <https://github.com/ros-controls/ros2_control/issues/1581>`_)
* Update joints_userdoc.rst (`#1604 <https://github.com/ros-controls/ros2_control/issues/1604>`_)
* Fix link to gazebosim.org (`#1563 <https://github.com/ros-controls/ros2_control/issues/1563>`_) (`#1564 <https://github.com/ros-controls/ros2_control/issues/1564>`_)
* Add doc page about joint kinematics (`#1497 <https://github.com/ros-controls/ros2_control/issues/1497>`_) (`#1559 <https://github.com/ros-controls/ros2_control/issues/1559>`_)
* Bump version of pre-commit hooks (backport `#1556 <https://github.com/ros-controls/ros2_control/issues/1556>`_) (`#1557 <https://github.com/ros-controls/ros2_control/issues/1557>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

2.41.0 (2024-04-30)
-------------------
* Add missing calculate_dynamics (`#1498 <https://github.com/ros-controls/ros2_control/issues/1498>`_) (`#1511 <https://github.com/ros-controls/ros2_control/issues/1511>`_)
* [Doc] Add documentation about initial_value regarding mock_hw (`#1352 <https://github.com/ros-controls/ros2_control/issues/1352>`_) (`#1513 <https://github.com/ros-controls/ros2_control/issues/1513>`_)
* Move migration notes (`#1481 <https://github.com/ros-controls/ros2_control/issues/1481>`_) (`#1515 <https://github.com/ros-controls/ros2_control/issues/1515>`_)
* Bump version of pre-commit hooks (backport `#1430 <https://github.com/ros-controls/ros2_control/issues/1430>`_) (`#1434 <https://github.com/ros-controls/ros2_control/issues/1434>`_)
* Contributors: mergify[bot]

2.40.0 (2024-03-02)
-------------------
* [CI] Code coverage + pre-commit (backport `#1413 <https://github.com/ros-controls/ros2_control/issues/1413>`_) (`#1414 <https://github.com/ros-controls/ros2_control/issues/1414>`_)
* Move hardware interface README content to sphinx documentation (`#1342 <https://github.com/ros-controls/ros2_control/issues/1342>`_) (`#1380 <https://github.com/ros-controls/ros2_control/issues/1380>`_)
* Contributors: mergify[bot]

2.39.1 (2024-02-14)
-------------------
* readd activate_all_components method to not break API (`#1392 <https://github.com/ros-controls/ros2_control/issues/1392>`_)
* Contributors: Sai Kishor Kothakota

2.39.0 (2024-02-12)
-------------------
* Move `test_components` to own package (backport `#1325 <https://github.com/ros-controls/ros2_control/issues/1325>`_) (`#1340 <https://github.com/ros-controls/ros2_control/issues/1340>`_)
* Contributors: mergify[bot]

2.38.0 (2024-01-25)
-------------------
* [CM] Fix controller parameter loading issue in different cases (`#1293 <https://github.com/ros-controls/ros2_control/issues/1293>`_) (`#1332 <https://github.com/ros-controls/ros2_control/issues/1332>`_)
* Enable setting of initial state in HW components (backport `#1046 <https://github.com/ros-controls/ros2_control/issues/1046>`_) (`#1064 <https://github.com/ros-controls/ros2_control/issues/1064>`_)
* Contributors: Sai Kishor Kothakota, mergify[bot]

2.37.0 (2024-01-20)
-------------------
* [RM] Fix crash for missing urdf in resource manager (`#1301 <https://github.com/ros-controls/ros2_control/issues/1301>`_) (`#1316 <https://github.com/ros-controls/ros2_control/issues/1316>`_)
* Add additional checks for non existing and not available interfaces. (backport `#1218 <https://github.com/ros-controls/ros2_control/issues/1218>`_) (`#1291 <https://github.com/ros-controls/ros2_control/issues/1291>`_)
* Fix return of ERROR and calls of cleanup when system is unconfigured of finalized (`#1279 <https://github.com/ros-controls/ros2_control/issues/1279>`_) (`#1286 <https://github.com/ros-controls/ros2_control/issues/1286>`_)
* fix the multiple definitions of lexical casts methods (`#1281 <https://github.com/ros-controls/ros2_control/issues/1281>`_) (`#1282 <https://github.com/ros-controls/ros2_control/issues/1282>`_)
* Contributors: Sai Kishor Kothakota, mergify[bot]

2.36.1 (2024-01-08)
-------------------
* [ResourceManager] adds test for uninitialized hardware (`#1243 <https://github.com/ros-controls/ros2_control/issues/1243>`_) (`#1274 <https://github.com/ros-controls/ros2_control/issues/1274>`_)
* Use portable version for string-to-double conversion (backport `#1257 <https://github.com/ros-controls/ros2_control/issues/1257>`_) (`#1268 <https://github.com/ros-controls/ros2_control/issues/1268>`_)
* Fix typo in docs (`#1219 <https://github.com/ros-controls/ros2_control/issues/1219>`_) (`#1221 <https://github.com/ros-controls/ros2_control/issues/1221>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

2.36.0 (2023-12-12)
-------------------
* Cleanup Resource Manager a bit to increase clarity. (backport `#816 <https://github.com/ros-controls/ros2_control/issues/816>`_) (`#1191 <https://github.com/ros-controls/ros2_control/issues/1191>`_)
* Handle hardware errors in Resource Manager (`#805 <https://github.com/ros-controls/ros2_control/issues/805>`_) (`#837 <https://github.com/ros-controls/ros2_control/issues/837>`_) #ABI-breaking
* Contributors: mergify[bot]

2.35.1 (2023-11-27)
-------------------
* [MockHardware] Fix the issues where hardware with multiple interfaces can not be started because of a logical bug added when adding dynamics calculation functionality. (`#1151 <https://github.com/ros-controls/ros2_control/issues/1151>`_) (`#1178 <https://github.com/ros-controls/ros2_control/issues/1178>`_)
* Contributors: Dr Denis

2.35.0 (2023-11-14)
-------------------
* [CM] Use `robot_description` topic instead of parameter and don't crash on empty URDF 🦿 (backport `#940 <https://github.com/ros-controls/ros2_control/issues/940>`_) (`#1052 <https://github.com/ros-controls/ros2_control/issues/1052>`_)
* Contributors: mergify[bot]

2.34.0 (2023-11-08)
-------------------

2.33.0 (2023-10-11)
-------------------
* [MockHardware] Added dynamic simulation functionality. (`#1028 <https://github.com/ros-controls/ros2_control/issues/1028>`_) (`#1125 <https://github.com/ros-controls/ros2_control/issues/1125>`_)
* Contributors: mergify[bot]

2.32.0 (2023-10-03)
-------------------
* Add GPIO tag description to docs (`#1109 <https://github.com/ros-controls/ros2_control/issues/1109>`_) (`#1120 <https://github.com/ros-controls/ros2_control/issues/1120>`_)
* Contributors: Christoph Froehlich

2.31.0 (2023-09-11)
-------------------

2.30.0 (2023-08-14)
-------------------
* Add checks if hardware is initialized. (backport `#1054 <https://github.com/ros-controls/ros2_control/issues/1054>`_) (`#1081 <https://github.com/ros-controls/ros2_control/issues/1081>`_)
* Contributors: Denis Stogl

2.29.0 (2023-07-09)
-------------------

2.28.0 (2023-06-23)
-------------------
* Ensure instantiation of hardware classes work for python bindings (`#1058 <https://github.com/ros-controls/ros2_control/issues/1058>`_) (`#1062 <https://github.com/ros-controls/ros2_control/issues/1062>`_)
* Contributors: Olivier Stasse

2.27.0 (2023-06-14)
-------------------
* [MockHardware] Enable disabling of command to simulate HW failures. (backport `#1027 <https://github.com/ros-controls/ros2_control/issues/1027>`_) (`#1050 <https://github.com/ros-controls/ros2_control/issues/1050>`_)
* Empty urdf tag humble (backport of `#1017 <https://github.com/ros-controls/ros2_control/issues/1017>`_) (`#1036 <https://github.com/ros-controls/ros2_control/issues/1036>`_)
* [Humble] enable ReflowComments to also use ColumnLimit on comments (`#1038 <https://github.com/ros-controls/ros2_control/issues/1038>`_)
* Issue 339: Implement parse_bool and refactor a few (backport `#1014 <https://github.com/ros-controls/ros2_control/issues/1014>`_) (`#1018 <https://github.com/ros-controls/ros2_control/issues/1018>`_)
* Contributors: Felix Exner (fexner), Sai Kishor Kothakota, Christoph Fröhlich, Bence Magyar, Alejandro Bordallo

2.26.0 (2023-05-20)
-------------------
* docs: Fix link to hardware_components (`#1009 <https://github.com/ros-controls/ros2_control/issues/1009>`_) (`#1011 <https://github.com/ros-controls/ros2_control/issues/1011>`_)
* Contributors: Christoph Fröhlich

2.25.3 (2023-04-29)
-------------------

2.25.2 (2023-04-20)
-------------------
* Also initialize non-joint components (backport `#822 <https://github.com/ros-controls/ros2_control/issues/822>`_) (`#991 <https://github.com/ros-controls/ros2_control/issues/991>`_)
* Contributors: Felix Exner, Denis Štogl

2.25.1 (2023-04-14)
-------------------

2.25.0 (2023-04-02)
-------------------
* Check for missing hardware interfaces that use the gpio tag. (`#953 <https://github.com/ros-controls/ros2_control/issues/953>`_)
* Split transmission interfaces (backport `#938 <https://github.com/ros-controls/ros2_control/issues/938>`_) (`#968 <https://github.com/ros-controls/ros2_control/issues/968>`_)
* Contributors: Ryan Sandzimier, Noel Jiménez García, Bence Magyar

2.24.1 (2023-03-09)
-------------------
* Revert "Make double parsing locale independent (`#921 <https://github.com/ros-controls/ros2_control/issues/921>`_)" (`#966 <https://github.com/ros-controls/ros2_control/issues/966>`_)
* Contributors: Bence Magyar

2.24.0 (2023-02-28)
-------------------

2.23.0 (2023-02-20)
-------------------

2.22.0 (2023-01-31)
-------------------
* Make double parsing locale independent (`#921 <https://github.com/ros-controls/ros2_control/issues/921>`_) (`#924 <https://github.com/ros-controls/ros2_control/issues/924>`_)
* Contributors: Henning Kayser

2.21.0 (2023-01-24)
-------------------

2.20.0 (2023-01-12)
-------------------
* 🔧 Fixes and updated on pre-commit hooks and their action (backport `#890 <https://github.com/ros-controls/ros2_control/issues/890>`_) (`#895 <https://github.com/ros-controls/ros2_control/issues/895>`_)
* Contributors: Denis Štogl

2.19.0 (2023-01-06)
-------------------
* ResourceManager doesn't always log an error on shutdown anymore (`#867 <https://github.com/ros-controls/ros2_control/issues/867>`_) (`#871 <https://github.com/ros-controls/ros2_control/issues/871>`_)
* Contributors: Christopher Wecht

2.18.0 (2022-12-03)
-------------------

2.17.0 (2022-11-27)
-------------------

2.16.0 (2022-10-17)
-------------------
* fix broken links (issue `#831 <https://github.com/ros-controls/ros2_control/issues/831>`_) (`#833 <https://github.com/ros-controls/ros2_control/issues/833>`_) (`#845 <https://github.com/ros-controls/ros2_control/issues/845>`_)
* Contributors: Manuel Muth

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
