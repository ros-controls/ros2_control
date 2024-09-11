^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.43.1 (2024-09-11)
-------------------

2.43.0 (2024-08-22)
-------------------
* Fix flaky transmission_interface tests by making them deterministic. (backport `#1665 <https://github.com/ros-controls/ros2_control/issues/1665>`_) (`#1670 <https://github.com/ros-controls/ros2_control/issues/1670>`_)
* Contributors: mergify[bot]

2.42.0 (2024-07-23)
-------------------

2.41.0 (2024-04-30)
-------------------
* rosdoc2 for transmission_interface (`#1496 <https://github.com/ros-controls/ros2_control/issues/1496>`_) (`#1509 <https://github.com/ros-controls/ros2_control/issues/1509>`_)
* Contributors: mergify[bot]

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
* Improve transmission tests (`#1238 <https://github.com/ros-controls/ros2_control/issues/1238>`_) (`#1241 <https://github.com/ros-controls/ros2_control/issues/1241>`_)
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

2.25.0 (2023-04-02)
-------------------
* Split transmission interfaces (backport `#938 <https://github.com/ros-controls/ros2_control/issues/938>`_) (`#968 <https://github.com/ros-controls/ros2_control/issues/968>`_)
* Contributors: Noel Jiménez García, Bence Magyar

2.24.1 (2023-03-09)
-------------------
* Fix missing include (`#963 <https://github.com/ros-controls/ros2_control/issues/963>`_) (`#965 <https://github.com/ros-controls/ros2_control/issues/965>`_)
* Contributors: Bence Magyar

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
