^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.43.1 (2024-09-11)
-------------------

2.43.0 (2024-08-22)
-------------------

2.42.0 (2024-07-23)
-------------------
* [ControllerInterface] Avoid warning about conversion from `int64_t` to `unsigned int` (backport `#1173 <https://github.com/ros-controls/ros2_control/issues/1173>`_) (`#1631 <https://github.com/ros-controls/ros2_control/issues/1631>`_)
* Fix dependencies for source build (`#1533 <https://github.com/ros-controls/ros2_control/issues/1533>`_) (`#1535 <https://github.com/ros-controls/ros2_control/issues/1535>`_)
* Contributors: mergify[bot]

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
* add a broadcaster for range sensor (`#1091 <https://github.com/ros-controls/ros2_control/issues/1091>`_) (`#1100 <https://github.com/ros-controls/ros2_control/issues/1100>`_)
* Contributors: flochre

2.30.0 (2023-08-14)
-------------------

2.29.0 (2023-07-09)
-------------------

2.28.0 (2023-06-23)
-------------------

2.27.0 (2023-06-14)
-------------------
* [Humble] enable ReflowComments to also use ColumnLimit on comments (`#1038 <https://github.com/ros-controls/ros2_control/issues/1038>`_)
* Contributors: Sai Kishor Kothakota

2.26.0 (2023-05-20)
-------------------

2.25.3 (2023-04-29)
-------------------

2.25.2 (2023-04-20)
-------------------

2.25.1 (2023-04-14)
-------------------
* Add missing build_export_depends to controller_interface (backport `#989 <https://github.com/ros-controls/ros2_control/issues/989>`_) (`#990 <https://github.com/ros-controls/ros2_control/issues/990>`_)
* Contributors: Scott K Logan

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
* Update imu_sensor.hpp (`#893 <https://github.com/ros-controls/ros2_control/issues/893>`_) (`#896 <https://github.com/ros-controls/ros2_control/issues/896>`_)
* Contributors: flochre

2.19.0 (2023-01-06)
-------------------

2.18.0 (2022-12-03)
-------------------

2.17.0 (2022-11-27)
-------------------

2.16.0 (2022-10-17)
-------------------
* Add docs in export interface configurations for controllers. (`#804 <https://github.com/ros-controls/ros2_control/issues/804>`_) (`#842 <https://github.com/ros-controls/ros2_control/issues/842>`_)
* Contributors: Denis Štogl

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

2.12.0 (2022-07-09)
-------------------

2.11.0 (2022-07-03)
-------------------
* [Interfaces] Improved ```get_name()``` method of hardware interfaces (soft) #api-breaking (`#737 <https://github.com/ros-controls/ros2_control/issues/737>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Full functionality of chainable controllers in controller manager (`#667 <https://github.com/ros-controls/ros2_control/issues/667>`_)
  * auto-switching of chained mode in controllers
  * interface-matching approach for managing chaining controllers
* Contributors: Bence Magyar, Denis Štogl, Lucas Schulze

2.10.0 (2022-06-18)
-------------------
* CMakeLists cleanup (`#733 <https://github.com/ros-controls/ros2_control/issues/733>`_)
* Update to clang format 12 (`#731 <https://github.com/ros-controls/ros2_control/issues/731>`_)
* Make interface_list_contains_interface_type inline (`#721 <https://github.com/ros-controls/ros2_control/issues/721>`_)
* Contributors: Andy Zelenak, Bence Magyar

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

2.7.0 (2022-04-29)
------------------
* Make node private in ControllerInterface (`#699 <https://github.com/ros-controls/ros2_control/issues/699>`_)
* Contributors: Jack Center

2.6.0 (2022-04-20)
------------------
* Add CallbackReturn into controller_interface namespace for simpler usage in controllers. (`#701 <https://github.com/ros-controls/ros2_control/issues/701>`_)
* Enable namespaces for controllers. (`#693 <https://github.com/ros-controls/ros2_control/issues/693>`_)
* Add tests for ControllerInterface class and clarify use of 'update_rate' parameter. (`#662 <https://github.com/ros-controls/ros2_control/issues/662>`_)
  #behaviorchange
* Contributors: Denis Štogl

2.5.0 (2022-03-25)
------------------
* Use lifecycle nodes in controllers again (`#538 <https://github.com/ros-controls/ros2_control/issues/538>`_)
  * Add lifecycle nodes
  * Add custom 'configure' to controller interface to get 'update_rate' parameter.
  * Disable external interfaces of LifecycleNode.
* Cleaning Controller Interface from obsolete code. (`#655 <https://github.com/ros-controls/ros2_control/issues/655>`_)
* Contributors: Denis Štogl, Vatan Aksoy Tezer, Bence Magyar

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
* fix get_update_rate visibility in windows (`#586 <https://github.com/ros-controls/ros2_control/issues/586>`_)
* Use lifecycle name constants from hardware interface in controller interface (`#575 <https://github.com/ros-controls/ros2_control/issues/575>`_)
  * Use lifecycle name constants from hardware interface in controller interface
  * Remove controller_state_names.hpp since it is not needed.
* Contributors: Melvin Wang, Xi-Huang

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------
* Quick fix 🏎: make doc on helpers clearer (`#553 <https://github.com/ros-controls/ros2_control/issues/553>`_)
* Contributors: Denis Štogl

1.0.0 (2021-09-29)
------------------
* Per controller update rate (`#513 <https://github.com/ros-controls/ros2_control/issues/513>`_)
  * add update_rate member field to controller manager
* added dt to controller interface and controller manager `#438 <https://github.com/ros-controls/ros2_control/issues/438>`_ (`#520 <https://github.com/ros-controls/ros2_control/issues/520>`_)
* Methods controlling the lifecycle of controllers all have on\_ prefix
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* rename get_current_state() to get_state() (`#512 <https://github.com/ros-controls/ros2_control/issues/512>`_)
* Contributors: Bence Magyar, Denis Štogl, Dmitri Ignakov, Márk Szitanics, bailaC

0.8.0 (2021-08-28)
------------------
* Automatic parameter declaration - enable existence of undeclared parameters from overrides (`#504 <https://github.com/ros-controls/ros2_control/issues/504>`_)
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Add pre-commit setup. (`#473 <https://github.com/ros-controls/ros2_control/issues/473>`_)
* Make controller_manager set controller's use_sim_time param when use_sim_time=True (`#468 <https://github.com/ros-controls/ros2_control/issues/468>`_)
* Correct obviously wrong call in controller interface. (`#460 <https://github.com/ros-controls/ros2_control/issues/460>`_)
* virtual destructors for semantic components (`#455 <https://github.com/ros-controls/ros2_control/issues/455>`_)
* Contributors: Denis Štogl, Karsten Knese, Lovro Ivanov, Simon Honigmann

0.7.1 (2021-06-15)
------------------
* Remove forgoten debug output (`#439 <https://github.com/ros-controls/ros2_control/issues/439>`_)
* Contributors: Denis Štogl

0.7.0 (2021-06-06)
------------------
* Add imu_sensor semantic component (`#429 <https://github.com/ros-controls/ros2_control/issues/429>`_)
* Fix osx warnings (`#428 <https://github.com/ros-controls/ros2_control/issues/428>`_)
* Add FTS as first semantic components to simplify controllers. (`#370 <https://github.com/ros-controls/ros2_control/issues/370>`_)
* Contributors: bailaC, Denis Štogl, Jordan Palacios, Karsten Knese, Victor Lopez

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------
* Added labels for controller states. (`#414 <https://github.com/ros-controls/ros2_control/issues/414>`_)
* prevent variable-sized object initialization (`#411 <https://github.com/ros-controls/ros2_control/issues/411>`_)
* Contributors: Denis Štogl, Karsten Knese, Bence Magyar

0.5.0 (2021-05-03)
------------------
* Add NodeOptions parameter to init function of controller_interface (`#382 <https://github.com/ros-controls/ros2_control/issues/382>`_)
* guard around pragmas (`#397 <https://github.com/ros-controls/ros2_control/issues/397>`_)
* avoid deprecations (`#393 <https://github.com/ros-controls/ros2_control/issues/393>`_)
* Contributors: Auguste Bourgois, Karsten Knese, Bence Magyar

0.4.0 (2021-04-07)
------------------
* Replace controller_interface return type SUCCESS by OK and mark SUCCESS as deprecated (`#374 <https://github.com/ros-controls/ros2_control/issues/374>`_)
* Contributors: Mateus Amarante

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
* Don't auto-declare override parameters and fix some prints (`#276 <https://github.com/ros-controls/ros2_control/issues/276>`_)
* Add configure controller service (`#272 <https://github.com/ros-controls/ros2_control/issues/272>`_)
* get_node() throw if node is uninitialized (`#268 <https://github.com/ros-controls/ros2_control/issues/268>`_)
* Remove lifecycle node (`#261 <https://github.com/ros-controls/ros2_control/issues/261>`_)
* Use resource manager (`#236 <https://github.com/ros-controls/ros2_control/issues/236>`_)
* import controller_interface
* Contributors: Bence Magyar, Denis Štogl, Jordan Palacios, Karsten Knese, Victor Lopez
