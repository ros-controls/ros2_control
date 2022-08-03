^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2022-08-03)
-------------------

0.10.1 (2022-05-31)
-------------------
* Make interface_list_contains_interface_type inline (`#721 <https://github.com/ros-controls/ros2_control/issues/721>`_) (`#722 <https://github.com/ros-controls/ros2_control/issues/722>`_)
* Contributors: Bence Magyar

0.10.0 (2022-02-23)
-------------------

0.9.0 (2021-12-20)
------------------

0.8.1 (2021-10-25)
------------------

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
