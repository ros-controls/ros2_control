^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

2.12.1 (2022-07-14)
-------------------

2.12.0 (2022-07-09)
-------------------

2.11.0 (2022-07-03)
-------------------
* [Interfaces] Improved ```get_name()``` method of hardware interfaces #api-breaking (`#737 <https://github.com/ros-controls/ros2_control/issues/737>`_)
* Update maintainers of packages (`#753 <https://github.com/ros-controls/ros2_control/issues/753>`_)
* Remove ament autolint (`#749 <https://github.com/ros-controls/ros2_control/issues/749>`_)
* Fixup ament cpplint on 22.04 (`#747 <https://github.com/ros-controls/ros2_control/issues/747>`_)
* Contributors: Bence Magyar, Denis Štogl, Lucas Schulze

2.10.0 (2022-06-18)
-------------------
* CMakeLists cleanup (`#733 <https://github.com/ros-controls/ros2_control/issues/733>`_)
* Update to clang format 12 (`#731 <https://github.com/ros-controls/ros2_control/issues/731>`_)
* Contributors: Andy Zelenak, Bence Magyar

2.9.0 (2022-05-19)
------------------

2.8.0 (2022-05-13)
------------------

2.7.0 (2022-04-29)
------------------

2.6.0 (2022-04-20)
------------------
* Port four bar linkage and differential transmission loaders from ROS1 (`#656 <https://github.com/ros-controls/ros2_control/issues/656>`_)
* Contributors: Márk Szitanics

2.5.0 (2022-03-25)
------------------

2.4.0 (2022-02-23)
------------------
* Fix transmission loader tests (`#642 <https://github.com/ros-controls/ros2_control/issues/642>`_)
* Contributors: Bence Magyar, Denis Štogl

2.3.0 (2022-02-18)
------------------
* Port transmission loader plugins from ROS1 (`#633 <https://github.com/ros-controls/ros2_control/issues/633>`_)
* Contributors: Márk Szitanics, Bence Magyar

2.2.0 (2022-01-24)
------------------

2.1.0 (2022-01-11)
------------------

2.0.0 (2021-12-29)
------------------
* simple transmission configure multiple definition fix (`#571 <https://github.com/ros-controls/ros2_control/issues/571>`_)
* Contributors: niiquaye

1.2.0 (2021-11-05)
------------------

1.1.0 (2021-10-25)
------------------

1.0.0 (2021-09-29)
------------------
* Do not manually set C++ version to 14 (`#516 <https://github.com/ros-controls/ros2_control/issues/516>`_)
* Refactor INSTANTIATE_TEST_CASE_P -> INSTANTIATE_TEST_SUITE_P (`#515 <https://github.com/ros-controls/ros2_control/issues/515>`_)
* Contributors: Bence Magyar

0.8.0 (2021-08-28)
------------------
* Use clang format as code formatter (`#491 <https://github.com/ros-controls/ros2_control/issues/491>`_)
* Transmission parsing v2 (`#471 <https://github.com/ros-controls/ros2_control/issues/471>`_)
  * move parsing responsibility to hardware_interface
  * parse transmission type
  * Cleanup unused parser
* Contributors: Bence Magyar, Denis Štogl

0.7.1 (2021-06-15)
------------------

0.7.0 (2021-06-06)
------------------

0.6.1 (2021-05-31)
------------------

0.6.0 (2021-05-23)
------------------
* Remove the with_value_ptr and class templatization for ReadOnlyHandle (`#379 <https://github.com/ros-controls/ros2_control/issues/379>`_)
* Fix transmission interface test on OSX (`#419 <https://github.com/ros-controls/ros2_control/issues/419>`_)
* Fix failing test on rolling (`#416 <https://github.com/ros-controls/ros2_control/issues/416>`_)
* Contributors: El Jawad Alaa, Karsten Knese, Vatan Aksoy Tezer, Bence Magyar

0.5.0 (2021-05-03)
------------------
* Replace standard interfaces' hard-coded strings by constants (`#376 <https://github.com/ros-controls/ros2_control/issues/376>`_)
* Contributors: Mateus Amarante

0.4.0 (2021-04-07)
------------------

0.3.0 (2021-03-21)
------------------

0.2.1 (2021-03-02)
------------------

0.2.0 (2021-02-26)
------------------
* Add four bar linkage transmission (`#307 <https://github.com/ros-controls/ros2_control/issues/307>`_)
* Contributors: Bence Magyar

0.1.6 (2021-02-05)
------------------

0.1.5 (2021-02-04)
------------------

0.1.4 (2021-02-03)
------------------
* Add differential transmission (`#303 <https://github.com/ros-controls/ros2_control/issues/303>`_)
* update doxygen style according to ros2 core standard (`#300 <https://github.com/ros-controls/ros2_control/issues/300>`_)
* Add supporting images for simple transmission documentation (`#304 <https://github.com/ros-controls/ros2_control/issues/304>`_)
* Contributors: Bence Magyar, João Victor Torres Borges

0.1.3 (2021-01-21)
------------------
* Remove parser from install until reworked (`#301 <https://github.com/ros-controls/ros2_control/issues/301>`_)
* Fix building on macOS with clang (`#292 <https://github.com/ros-controls/ros2_control/issues/292>`_)
* Add simple transmission class (`#245 <https://github.com/ros-controls/ros2_control/issues/245>`_)
* Contributors: Bence Magyar, Karsten Knese

0.1.2 (2021-01-06)
------------------

0.1.1 (2020-12-23)
------------------

0.0.1 (2020-12-22)
------------------
* Transmission interface URDF parsing (imported from ddengster) (`#182 <https://github.com/ros-controls/ros2_control/issues/182>`_)
* Transmission parsing from urdf (`#92 <https://github.com/ros-controls/ros2_control/issues/92>`_)
* Contributors: Bence Magyar, Colin MacKenzie, Edwin Fan, Karsten Knese, Yutaka Kondo
