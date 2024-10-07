^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

4.13.0 (2024-07-08)
-------------------
* [ControllerChaining] Export state interfaces from chainable controllers (`#1021 <https://github.com/ros-controls/ros2_control/issues/1021>`_)
* Contributors: Sai Kishor Kothakota

4.12.0 (2024-07-01)
-------------------

4.11.0 (2024-05-14)
-------------------

4.10.0 (2024-05-08)
-------------------

4.9.0 (2024-04-30)
------------------

4.8.0 (2024-03-27)
------------------

4.7.0 (2024-03-22)
------------------

4.6.0 (2024-03-02)
------------------

4.5.0 (2024-02-12)
------------------

4.4.0 (2024-01-31)
------------------

4.3.0 (2024-01-20)
------------------

4.2.0 (2023-12-12)
------------------

4.1.0 (2023-11-30)
------------------

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

3.13.0 (2023-05-18)
-------------------

3.12.2 (2023-04-29)
-------------------

3.12.1 (2023-04-14)
-------------------

3.12.0 (2023-04-02)
-------------------

3.11.0 (2023-03-22)
-------------------

3.10.0 (2023-03-16)
-------------------

3.9.1 (2023-03-09)
------------------

3.9.0 (2023-02-28)
------------------
* Remove deprecations from CLI and controller_manager (`#948 <https://github.com/ros-controls/ros2_control/issues/948>`_)
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* Contributors: Christoph Fröhlich

3.8.0 (2023-02-10)
------------------
* Fix CMake install so overriding works (`#926 <https://github.com/ros-controls/ros2_control/issues/926>`_)
* Contributors: Tyler Weaver

3.7.0 (2023-01-24)
------------------

3.6.0 (2023-01-12)
------------------

3.5.1 (2023-01-06)
------------------

3.5.0 (2022-12-06)
------------------
* Rename class type to plugin name #api-breaking #abi-breaking (`#780 <https://github.com/ros-controls/ros2_control/issues/780>`_)
* Contributors: Bence Magyar

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
* Add chained controllers information in list controllers service #abi-braking (`#758 <https://github.com/ros-controls/ros2_control/issues/758>`_)
  * add chained controllers in ros2controlcli
  * remove controller_group from service
  * added comments to ControllerState message
  * added comments to ChainedConnection message
* Contributors: Paul Gesel

2.12.1 (2022-07-14)
-------------------

2.12.0 (2022-07-09)
-------------------
* Deprecate and rename `start` and `stop` nomenclature toward user to `activate` and `deactivate` #ABI-breaking (`#755 <https://github.com/ros-controls/ros2_control/issues/755>`_)
  * Rename fields and deprecate old nomenclature.
  * Add new defines to SwitchController.srv
  * Deprecated start/stop nomenclature in all CLI commands.
  * Deprecate 'start_asap' too as other fields.
* Contributors: Denis Štogl

2.11.0 (2022-07-03)
-------------------
* Remove hybrid services in controller manager. They are just overhead. (`#761 <https://github.com/ros-controls/ros2_control/issues/761>`_)
* Update and fix CI setup (`#752 <https://github.com/ros-controls/ros2_control/issues/752>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Remove ament autolint (`#749 <https://github.com/ros-controls/ros2_control/issues/749>`_)
* Contributors: Bence Magyar, Denis Štogl

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
* Add service-skeletons for controlling hardware lifecycle. (`#585 <https://github.com/ros-controls/ros2_control/issues/585>`_)
* Contributors: Denis Štogl

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------
* controller_manager: Use command_interface_configuration for the claimed interfaces when calling list_controllers (`#544 <https://github.com/ros-controls/ros2_control/issues/544>`_)
* Contributors: Jafar Abdi, Denis Štogl

1.0.0 (2021-09-29)
------------------
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* Contributors: Bence Magyar

0.8.0 (2021-08-28)
------------------
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Contributors: Denis Štogl

0.7.1 (2021-06-15)
------------------

0.7.0 (2021-06-06)
------------------

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------
* List controller claimed interfaces (`#407 <https://github.com/ros-controls/ros2_control/issues/407>`_)
* Contributors: Jordan Palacios

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
* Add configure controller service (`#272 <https://github.com/ros-controls/ros2_control/issues/272>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* Add controller manager services (`#139 <https://github.com/ros-controls/ros2_control/issues/139>`_)
* Contributors: Bence Magyar, Denis Štogl, Karsten Knese, Victor Lopez
