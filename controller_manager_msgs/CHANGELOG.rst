^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
