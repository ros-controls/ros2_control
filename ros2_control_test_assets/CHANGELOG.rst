^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_control_test_assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
