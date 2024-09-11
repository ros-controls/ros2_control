^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_control_test_assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.43.1 (2024-09-11)
-------------------

2.43.0 (2024-08-22)
-------------------

2.42.0 (2024-07-23)
-------------------

2.41.0 (2024-04-30)
-------------------

2.40.0 (2024-03-02)
-------------------

2.39.1 (2024-02-14)
-------------------

2.39.0 (2024-02-12)
-------------------

2.38.0 (2024-01-25)
-------------------

2.37.0 (2024-01-20)
-------------------

2.36.1 (2024-01-08)
-------------------
* [ResourceManager] adds test for uninitialized hardware (`#1243 <https://github.com/ros-controls/ros2_control/issues/1243>`_) (`#1274 <https://github.com/ros-controls/ros2_control/issues/1274>`_)
* Contributors: mergify[bot]

2.36.0 (2023-12-12)
-------------------

2.35.1 (2023-11-27)
-------------------

2.35.0 (2023-11-14)
-------------------

2.34.0 (2023-11-08)
-------------------

2.33.0 (2023-10-11)
-------------------

2.32.0 (2023-10-03)
-------------------

2.31.0 (2023-09-11)
-------------------

2.30.0 (2023-08-14)
-------------------

2.29.0 (2023-07-09)
-------------------

2.28.0 (2023-06-23)
-------------------

2.27.0 (2023-06-14)
-------------------
* Empty urdf tag humble (backport of `#1017 <https://github.com/ros-controls/ros2_control/issues/1017>`_) (`#1036 <https://github.com/ros-controls/ros2_control/issues/1036>`_)
* Contributors: Felix Exner (fexner)

2.26.0 (2023-05-20)
-------------------

2.25.3 (2023-04-29)
-------------------

2.25.2 (2023-04-20)
-------------------

2.25.1 (2023-04-14)
-------------------

2.25.0 (2023-04-02)
-------------------

2.24.1 (2023-03-09)
-------------------

2.24.0 (2023-02-28)
-------------------

2.23.0 (2023-02-20)
-------------------

2.22.0 (2023-01-31)
-------------------

2.21.0 (2023-01-24)
-------------------

2.20.0 (2023-01-12)
-------------------

2.19.0 (2023-01-06)
-------------------

2.18.0 (2022-12-03)
-------------------

2.17.0 (2022-11-27)
-------------------

2.16.0 (2022-10-17)
-------------------

2.15.0 (2022-09-19)
-------------------

2.14.0 (2022-09-04)
-------------------

2.13.0 (2022-08-03)
-------------------

2.12.1 (2022-07-14)
-------------------

2.12.0 (2022-07-09)
-------------------

2.11.0 (2022-07-03)
-------------------
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Remove ament autolint (`#749 <https://github.com/ros-controls/ros2_control/issues/749>`_)
* Contributors: Bence Magyar

2.10.0 (2022-06-18)
-------------------
* Make RHEL CI happy! (`#730 <https://github.com/ros-controls/ros2_control/issues/730>`_)
* CMakeLists cleanup (`#733 <https://github.com/ros-controls/ros2_control/issues/733>`_)
* Contributors: Andy Zelenak, Márk Szitanics

2.9.0 (2022-05-19)
------------------

2.8.0 (2022-05-13)
------------------

2.7.0 (2022-04-29)
------------------

2.6.0 (2022-04-20)
------------------

2.5.0 (2022-03-25)
------------------

2.4.0 (2022-02-23)
------------------

2.3.0 (2022-02-18)
------------------

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
* Adding support for 'initial_value' parameter. (`#593 <https://github.com/ros-controls/ros2_control/issues/593>`_)
* Contributors: bailaC

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------
* Moving diffbot_urdf to ros2_control_test_assets. (`#558 <https://github.com/ros-controls/ros2_control/issues/558>`_)
* Contributors: bailaC

1.0.0 (2021-09-29)
------------------
* Remove unnecessary includes (`#518 <https://github.com/ros-controls/ros2_control/issues/518>`_)
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* Contributors: Bence Magyar, Denis Štogl

0.8.0 (2021-08-28)
------------------
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Transmission parsing v2 (`#471 <https://github.com/ros-controls/ros2_control/issues/471>`_)
* Added GPIO parsing and test (`#436 <https://github.com/ros-controls/ros2_control/issues/436>`_)
* Contributors: Bence Magyar, Denis Štogl, Mathias Arbo

0.7.1 (2021-06-15)
------------------

0.7.0 (2021-06-06)
------------------

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------

0.5.0 (2021-05-03)
------------------

0.4.0 (2021-04-07)
------------------
* [ros2_control_test_assets] Fix typo (`#371 <https://github.com/ros-controls/ros2_control/issues/371>`_)
* Contributors: Yutaka Kondo

0.3.0 (2021-03-21)
------------------

0.2.1 (2021-03-02)
------------------

0.2.0 (2021-02-26)
------------------
* Add "Fake" components for simple integration of framework (`#323 <https://github.com/ros-controls/ros2_control/issues/323>`_)
* Moved example URDFs for parser/scenario tests to assets package (`#316 <https://github.com/ros-controls/ros2_control/issues/316>`_)
* Contributors: Denis Štogl

0.1.6 (2021-02-05)
------------------
* correct hardware interface validation in resource manager. (`#317 <https://github.com/ros-controls/ros2_control/issues/317>`_)
* Add missing test dep (`#321 <https://github.com/ros-controls/ros2_control/issues/321>`_)
* Contributors: Bence Magyar, Karsten Knese

0.1.5 (2021-02-04)
------------------
* Add missing buildtool dep (`#319 <https://github.com/ros-controls/ros2_control/issues/319>`_)
* Contributors: Bence Magyar

0.1.4 (2021-02-03)
------------------
* Add test assets package (`#289 <https://github.com/ros-controls/ros2_control/issues/289>`_)
* Contributors: Denis Štogl

0.1.3 (2021-01-21)
------------------

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
