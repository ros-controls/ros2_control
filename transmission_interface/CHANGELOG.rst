^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
