^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface_testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.2.0 (2025-06-07)
------------------

5.1.0 (2025-05-24)
------------------
* [RM] Isolate start and stop interfaces for each Hardware Component (`#2120 <https://github.com/ros-controls/ros2_control/issues/2120>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#2266 <https://github.com/ros-controls/ros2_control/issues/2266>`_)
* Contributors: Sai Kishor Kothakota

5.0.0 (2025-05-21)
------------------
* Statically allocate string concatenations using FMT formatting (`#2205 <https://github.com/ros-controls/ros2_control/issues/2205>`_)
* Suppress the deprecation warnings of the hardware_interface API (`#2223 <https://github.com/ros-controls/ros2_control/issues/2223>`_)
* Contributors: Sai Kishor Kothakota, mini-1235

4.29.0 (2025-05-04)
-------------------
* [Diagnostics] Add diagnostics of execution time and periodicity of the hardware components (`#2086 <https://github.com/ros-controls/ros2_control/issues/2086>`_)
* Contributors: Sai Kishor Kothakota

4.28.1 (2025-04-17)
-------------------
* Use previous command to enforce the joint limits on position interfaces (`#2183 <https://github.com/ros-controls/ros2_control/issues/2183>`_)
* Contributors: Sai Kishor Kothakota

4.28.0 (2025-04-10)
-------------------
* Integrate joint limit enforcement into `ros2_control` framework functional with Async controllers and components  (`#2047 <https://github.com/ros-controls/ros2_control/issues/2047>`_)
* Make all packages use gmock, not gtest (`#2162 <https://github.com/ros-controls/ros2_control/issues/2162>`_)
* Bump version of pre-commit hooks (`#2156 <https://github.com/ros-controls/ros2_control/issues/2156>`_)
* Use ros2_control_cmake (`#2134 <https://github.com/ros-controls/ros2_control/issues/2134>`_)
* [Handle] Add support for booleans in the handles (`#2065 <https://github.com/ros-controls/ros2_control/issues/2065>`_)
* Improve package descriptions & update maintainers (`#2103 <https://github.com/ros-controls/ros2_control/issues/2103>`_)
* [RM] Fix skipped cycles by adjusting `rw_rate` handling (`#2091 <https://github.com/ros-controls/ros2_control/issues/2091>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, RobertWilbrandt, Sai Kishor Kothakota, Soham Patil, github-actions[bot]

4.27.0 (2025-03-01)
-------------------
* [Handle] Use `get_optional` instead of `get_value<double>` (`#2061 <https://github.com/ros-controls/ros2_control/issues/2061>`_)
* Add new `get_value` API for Handles and Interfaces (`#1976 <https://github.com/ros-controls/ros2_control/issues/1976>`_)
* [CM] Fix the controller deactivation on the control cycles returning ERROR  (`#1756 <https://github.com/ros-controls/ros2_control/issues/1756>`_)
* Contributors: Sai Kishor Kothakota

4.26.0 (2025-02-07)
-------------------
* Fix memory leak in the ros2_control (`#2033 <https://github.com/ros-controls/ros2_control/issues/2033>`_)
* Contributors: Sai Kishor Kothakota

4.25.0 (2025-01-29)
-------------------

4.24.0 (2025-01-13)
-------------------

4.23.0 (2024-12-29)
-------------------

4.22.0 (2024-12-20)
-------------------
* Propagate read/write rate to the HardwareInfo properly (`#1928 <https://github.com/ros-controls/ros2_control/issues/1928>`_)
* Async Hardware Components (`#1567 <https://github.com/ros-controls/ros2_control/issues/1567>`_)
* Contributors: Sai Kishor Kothakota

4.21.0 (2024-12-06)
-------------------
* [Feature] Choose different read and write rate for the hardware components (`#1570 <https://github.com/ros-controls/ros2_control/issues/1570>`_)
* Contributors: Sai Kishor Kothakota

4.20.0 (2024-11-08)
-------------------

4.19.0 (2024-10-26)
-------------------

4.18.0 (2024-10-07)
-------------------
* Adapt controller Reference/StateInterfaces to New Way of Exporting (variant support) (`#1689 <https://github.com/ros-controls/ros2_control/issues/1689>`_)
* Contributors: Manuel Muth

4.17.0 (2024-09-11)
-------------------

4.16.1 (2024-08-24)
-------------------

4.16.0 (2024-08-22)
-------------------

4.15.0 (2024-08-05)
-------------------
* [RM] Add `get_hardware_info` method to the Hardware Components (`#1643 <https://github.com/ros-controls/ros2_control/issues/1643>`_)
* Contributors: Sai Kishor Kothakota

4.14.0 (2024-07-23)
-------------------
* Unused header cleanup (`#1627 <https://github.com/ros-controls/ros2_control/issues/1627>`_)
* Fix typos in test_resource_manager.cpp (`#1609 <https://github.com/ros-controls/ros2_control/issues/1609>`_)
* Contributors: Henry Moore, Parker Drouillard

4.13.0 (2024-07-08)
-------------------
* [ResourceManager] Propagate access to logger and clock interfaces to HardwareComponent (`#1585 <https://github.com/ros-controls/ros2_control/issues/1585>`_)
* Contributors: Sai Kishor Kothakota

4.12.0 (2024-07-01)
-------------------
* [RM] Rename `load_urdf` method to `load_and_initialize_components` and add error handling there to avoid stack crashing when error happens. (`#1354 <https://github.com/ros-controls/ros2_control/issues/1354>`_)
* Contributors: Dr. Denis

4.11.0 (2024-05-14)
-------------------

4.10.0 (2024-05-08)
-------------------

4.9.0 (2024-04-30)
------------------
* Component parser: Get mimic information from URDF (`#1256 <https://github.com/ros-controls/ros2_control/issues/1256>`_)
* Contributors: Christoph Fröhlich

4.8.0 (2024-03-27)
------------------

4.7.0 (2024-03-22)
------------------

4.6.0 (2024-03-02)
------------------
* Add -Werror=missing-braces to compile options (`#1423 <https://github.com/ros-controls/ros2_control/issues/1423>`_)
* Contributors: Sai Kishor Kothakota

4.5.0 (2024-02-12)
------------------

4.4.0 (2024-01-31)
------------------
* Fix version
* Move `test_components` to own package (`#1325 <https://github.com/ros-controls/ros2_control/issues/1325>`_)
* Contributors: Bence Magyar, Christoph Fröhlich
