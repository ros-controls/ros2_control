^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_limits
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.28.1 (2025-04-17)
-------------------
* Use previous command to enforce the joint limits on position interfaces (`#2183 <https://github.com/ros-controls/ros2_control/issues/2183>`_)
* Fix the joint limits enforcement with `position` and `velocity` (`#2182 <https://github.com/ros-controls/ros2_control/issues/2182>`_)
* Contributors: Sai Kishor Kothakota

4.28.0 (2025-04-10)
-------------------
* Integrate joint limit enforcement into `ros2_control` framework functional with Async controllers and components  (`#2047 <https://github.com/ros-controls/ros2_control/issues/2047>`_)
* Change to `ament_add_gmock` in joint_limits (`#2165 <https://github.com/ros-controls/ros2_control/issues/2165>`_)
* Make all packages use gmock, not gtest (`#2162 <https://github.com/ros-controls/ros2_control/issues/2162>`_)
* Use ros2_control_cmake (`#2134 <https://github.com/ros-controls/ros2_control/issues/2134>`_)
* Improve package descriptions & update maintainers (`#2103 <https://github.com/ros-controls/ros2_control/issues/2103>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, Sai Kishor Kothakota, Soham Patil

4.27.0 (2025-03-01)
-------------------

4.26.0 (2025-02-07)
-------------------

4.25.0 (2025-01-29)
-------------------
* Define _USE_MATH_DEFINES in joint_soft_limiter.cpp to ensure that M_PI is defined (`#2001 <https://github.com/ros-controls/ros2_control/issues/2001>`_)
* Use actual position when limiting desired position (`#1988 <https://github.com/ros-controls/ros2_control/issues/1988>`_)
* Contributors: Sai Kishor Kothakota, Silvio Traversaro

4.24.0 (2025-01-13)
-------------------
* Return strong type for joint_limits helpers (`#1981 <https://github.com/ros-controls/ros2_control/issues/1981>`_)
* Trigger shutdown transition in destructor (`#1979 <https://github.com/ros-controls/ros2_control/issues/1979>`_)
* Add joint limiter interface plugins to enforce limits defined in the URDF (`#1526 <https://github.com/ros-controls/ros2_control/issues/1526>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota, Wiktor Bajor

4.23.0 (2024-12-29)
-------------------
* Remove boilerplate visibility macros (`#1972 <https://github.com/ros-controls/ros2_control/issues/1972>`_)
* Contributors: Bence Magyar

4.22.0 (2024-12-20)
-------------------

4.21.0 (2024-12-06)
-------------------
* Use the .hpp headers from realtime_tools package (`#1916 <https://github.com/ros-controls/ros2_control/issues/1916>`_)
* Contributors: Christoph Fröhlich

4.20.0 (2024-11-08)
-------------------

4.19.0 (2024-10-26)
-------------------

4.18.0 (2024-10-07)
-------------------

4.17.0 (2024-09-11)
-------------------

4.16.1 (2024-08-24)
-------------------

4.16.0 (2024-08-22)
-------------------

4.15.0 (2024-08-05)
-------------------

4.14.0 (2024-07-23)
-------------------
* Unused header cleanup (`#1627 <https://github.com/ros-controls/ros2_control/issues/1627>`_)
* Contributors: Henry Moore

4.13.0 (2024-07-08)
-------------------
* [JointLimits] Add Saturation Joint Limiter that uses clamping method (`#971 <https://github.com/ros-controls/ros2_control/issues/971>`_)
* Contributors: Dr. Denis

4.12.0 (2024-07-01)
-------------------
* Reactivate generate_version_header (`#1544 <https://github.com/ros-controls/ros2_control/issues/1544>`_)
* Bump version of pre-commit hooks (`#1556 <https://github.com/ros-controls/ros2_control/issues/1556>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.11.0 (2024-05-14)
-------------------
* Fix dependencies for source build (`#1533 <https://github.com/ros-controls/ros2_control/issues/1533>`_)
* Add find_package for ament_cmake_gen_version_h (`#1534 <https://github.com/ros-controls/ros2_control/issues/1534>`_)
* Contributors: Christoph Fröhlich

4.10.0 (2024-05-08)
-------------------

4.9.0 (2024-04-30)
------------------

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
* Contributors: Sai Kishor Kothakota

4.5.0 (2024-02-12)
------------------

4.4.0 (2024-01-31)
------------------
* [Format] Correct formatting of JointLimits URDF file. (`#1339 <https://github.com/ros-controls/ros2_control/issues/1339>`_)
* Add header to import limits from standard URDF definition (`#1298 <https://github.com/ros-controls/ros2_control/issues/1298>`_)
* Contributors: Dr. Denis, Sai Kishor Kothakota

4.3.0 (2024-01-20)
------------------

4.2.0 (2023-12-12)
------------------

4.1.0 (2023-11-30)
------------------
* Add few warning compiler options to error (`#1181 <https://github.com/ros-controls/ros2_control/issues/1181>`_)
* Contributors: Sai Kishor Kothakota

4.0.0 (2023-11-21)
------------------

3.21.0 (2023-11-06)
-------------------

3.20.0 (2023-10-31)
-------------------

3.19.1 (2023-10-04)
-------------------

3.19.0 (2023-10-03)
-------------------

3.18.0 (2023-08-17)
-------------------

3.17.0 (2023-08-07)
-------------------

3.16.0 (2023-07-09)
-------------------

3.15.0 (2023-06-23)
-------------------

3.14.0 (2023-06-14)
-------------------
* Add -Wconversion flag to protect future developments (`#1053 <https://github.com/ros-controls/ros2_control/issues/1053>`_)
* enable ReflowComments to also use ColumnLimit on comments (`#1037 <https://github.com/ros-controls/ros2_control/issues/1037>`_)
* Contributors: Sai Kishor Kothakota, gwalck

3.13.0 (2023-05-18)
-------------------

3.12.2 (2023-04-29)
-------------------

3.12.1 (2023-04-14)
-------------------

3.12.0 (2023-04-02)
-------------------
* Extend joint limits structure with deceleration limits. (`#977 <https://github.com/ros-controls/ros2_control/issues/977>`_)
* Contributors: Dr. Denis

3.11.0 (2023-03-22)
-------------------

3.10.0 (2023-03-16)
-------------------

3.9.1 (2023-03-09)
------------------

3.9.0 (2023-02-28)
------------------

3.8.0 (2023-02-10)
------------------
* Fix CMake install so overriding works (`#926 <https://github.com/ros-controls/ros2_control/issues/926>`_)
* 🖤 Add Black formatter for Python files. (`#936 <https://github.com/ros-controls/ros2_control/issues/936>`_)
* Contributors: Dr. Denis, Tyler Weaver

3.7.0 (2023-01-24)
------------------

3.6.0 (2023-01-12)
------------------

3.5.1 (2023-01-06)
------------------

3.5.0 (2022-12-06)
------------------

3.4.0 (2022-11-27)
------------------

3.3.0 (2022-11-15)
------------------

3.2.0 (2022-10-15)
------------------

3.1.0 (2022-10-05)
------------------

3.0.0 (2022-09-19)
------------------

2.15.0 (2022-09-19)
-------------------

2.14.0 (2022-09-04)
-------------------

2.13.0 (2022-08-03)
-------------------
* Make output of joint limits nicer. (`#788 <https://github.com/ros-controls/ros2_control/issues/788>`_)
* Contributors: Denis Štogl

2.12.1 (2022-07-14)
-------------------

2.12.0 (2022-07-09)
-------------------
* Move Joint Limits structures for use in controllers (`#462 <https://github.com/ros-controls/ros2_control/issues/462>`_)
* Contributors: Denis Štogl, Andy Zelenak, Bence Magyar

2.11.0 (2022-07-03)
-------------------

2.10.0 (2022-06-18)
-------------------

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

2.1.0 (2022-01-11)
------------------

2.0.0 (2021-12-29)
------------------

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------

1.0.0 (2021-09-29)
------------------

0.8.0 (2021-08-28)
------------------

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

0.3.0 (2021-03-21)
------------------

0.2.1 (2021-03-02)
------------------

0.2.0 (2021-02-26)
------------------

0.1.6 (2021-02-05)
------------------

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------

0.1.3 (2021-01-21)
------------------

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.1.0 (2020-12-22)
------------------
