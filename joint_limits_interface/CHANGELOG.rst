^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_limits_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.16.0 (2020-01-27)
-------------------
* Merge pull request `#413 <https://github.com/ros-controls/ros_control/issues/413>`_ from matthew-reynolds/range-for
  Use range-based for loop
* Use more meaningful pair iterator names
* Merge pull request `#404 <https://github.com/ros-controls/ros_control/issues/404>`_ from matthew-reynolds/catkin-lint
  Update CMakeLists.txt and package.xml
* Use range-based for loops in joint_limits_interface
* Update dependencies
  - Dependencies needed to compile are <build_depend>
  - Dependencies used in public headers are <build_export_depend>
  - Dependencies needed to link or run are <exec_depend>
* Merge branch 'melodic-devel' into catkin-lint
* Update package dependencies
* Remove liburdfdom-dev
* Add missing roscpp & rospy dependencies
* Remove rosunit test_depend from package.xml
* Merge pull request `#405 <https://github.com/ros-controls/ros_control/issues/405>`_ from matthew-reynolds/use-nullptr
  Use nullptr
* Prefer 0.0 for floating point literals
* Merge pull request `#406 <https://github.com/ros-controls/ros_control/issues/406>`_ from matthew-reynolds/pragma-once
  Use #pragma once
* Replace header guard with #pragma once
* Merge pull request `#395 <https://github.com/ros-controls/ros_control/issues/395>`_ from pal-robotics-forks/extend-interfaces-melodic
  Extend interfaces
* joint_limits: use an open-loop policy for velocity staturation
  The feedback from the controller is way too slow to be used on an
  actual robot. A robot that had 15 rad.s^-2 on each wheel as
  an acceleration limit could not even reach 2 rad.s^-2
  This is in line with ros_controllers`#23 <https://github.com/ros-controls/ros_control/issues/23>`_
* Apply consistent style to CMakeLists.txt files
* Apply consistent style to package.xml files
* Merge pull request `#398 <https://github.com/ros-controls/ros_control/issues/398>`_ from matthew-reynolds/revert-cmake
  Revert CMake include_directories as SYSTEM
* Revert CMake include_directories as SYSTEM
* Merge pull request `#396 <https://github.com/ros-controls/ros_control/issues/396>`_ from pal-robotics-forks/small-fixes
  Small fixes
* Fix shadowed variables
* -Werror=overloaded-virtual and initialization of fields in constructor
* Contributors: Bence Magyar, Daniel Pinyol, Matt Reynolds, Paul Mathieu, Victor Lopez

0.15.1 (2018-09-30)
-------------------

0.15.0 (2018-05-28)
-------------------

0.14.2 (2018-04-26)
-------------------
* Update maintainers
* Fix catkin_lint errors and warnings
* Contributors: Bence Magyar

0.14.1 (2018-04-16)
-------------------

0.14.0 (2018-03-26)
-------------------

0.13.0 (2017-12-23)
-------------------
* Add method to populate SoftJointLimits from ROS parameter server. (`#292 <https://github.com/ros-controls/ros_control/issues/292>`_)
* Contributors: Miguel Prada

0.12.0 (2017-08-05)
-------------------

0.11.5 (2017-06-28)
-------------------
* Throw error if EffortJointSaturationHandle is missing effort or velocity limits
* Contributors: Bence Magyar, Dave Coleman

0.11.4 (2017-02-14)
-------------------

0.11.3 (2016-12-07)
-------------------
* Add urdf compatibility header
* Contributors: Bence Magyar

0.11.2 (2016-11-28)
-------------------
* Add Enrique and Bence to maintainer list
* Clean up export leftovers from rosbuild
* Convert to format2, fix dependency in cmake
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr
* Contributors: Bence Magyar

0.11.1 (2016-08-18)
-------------------

0.11.0 (2016-05-23)
-------------------

0.10.1 (2016-04-23)
-------------------
* Fix catkin_package
  * Don't export local include dirs.
  * Fix dependency on urdfdom (Thanks to Mathias Lüdtke).
* Contributors: Jochen Sprickerhof

0.10.0 (2015-11-20)
-------------------

0.9.3 (2015-05-05)
------------------

0.9.2 (2015-05-04)
------------------
* Reset functionality for stateful position joint limit handles
* Contributors: Mathias Lüdtke

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Buildsystem and documentation fixes
* Add inline keyword to free header functions
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, shadowmanos

0.8.2 (2014-06-25)
------------------
* Propagate urdfdom changes to CMakeLists.txt
  urdfdom is now standalone, so it must be find_package'd independently.
* Fix rostest, which was not being built correctly.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-06-24)
------------------
* Use upstream liburdfdom-dev package.
  Refs `ros/rosdistro#4633 <https://github.com/ros/rosdistro/issues/4633>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Remove rosbuild artifacts. Fix `#154 <https://github.com/ros-controls/ros_control/issues/154>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------
* Fix dependency specification in CMake script to allow isolated builds.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Updated the interface list.
* Added the PositionJointSaturationInterface and VelocitySoftLimitsInterface
  classes. There are now saturation and soft limit classes for effort-controlled,
  position-controlled, and velocity-controlled joints.
* Contributors: Jim Rothrock

0.5.8 (2013-10-11)
------------------
* Merge pull request `#121 <https://github.com/ros-controls/ros_control/issues/121>`_ from pal-robotics/hydro-devel
  Fixes for next minor release
* Added the EffortJointSaturationHandle and EffortJointSaturationInterface
  classes. They are used with joints that do not have soft limits specified in
  their URDF files.
* Minor documentation precision.
* Make position joint limits handle opn loop.
  - Lowers the entry barrier for simple robots without velocity measurements,
  poor control tracking or with a slow update rate.
* Update README.md
* Create README.md
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Fix license header in some files.
* Renamed joint_limits_interface manifext.xml

0.5.7 (2013-07-30)
------------------

* Updated changelogs
* Add angle_wraparound joint limit property.
  For full compatibility with MoveIt!'s joint limit specification.
  Note that we still have the extra effort and jerk specification.

0.5.6 (2013-07-29)
------------------

0.5.5 (2013-07-23)
------------------

0.5.4 (2013-07-23)
------------------

0.5.3 (2013-07-22)
------------------

0.5.2 (2013-07-22)
------------------
* Fixed gtests for joint_limits_interface in catkin
* Merge pull request `#93 <https://github.com/davetcoleman/ros_control/issues/93>`_ from pal-robotics/master
  joint_limits_interface broken in Groocy and Hydro
* Fix for joint_limits tests in catkin
* Restore urdf dependencies.
  Add conditional compilation for Fuerte and Groovy+ distros.

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------
* Made joint_limits_interface match hydro version number
* Removed urdf_interface dependencies
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Better documentation of YAML joint limits spec.
  - Add cross-references in doc main page.
* Documentation improvements.
  - More consistency between transmission and joint limits interfaces doc.
  - Make explicit that these interfaces are not meant to be used by controllers,
  but by the robot abstraction.
* build dependency rostest added to package.xml and rostest added to CMakeLists.txt
* Added dependency for rostest to fix build error
* Fix compiler warnings (-Wreorder)
* Minor doc structure improvements.
* Add main page to joint_limits_interface doc.
* Remove temporary file from version control.
* Add attribution for soft_limits code.
  - Soft-limits enforcing is based on a previous implementation by Willow Garage.
  Add them in the copyright holders list.
* Lower severity of log message.
* Allow unsetting limits specification from rosparam.
  - Update tests.
* Add .gitignore
* Add joint limits parsing from rosparam + unit test.
* Add max_jerk to limits specification.
* Minor maintenance fixes.
* Add documentation.
* Extensive file, namespace, class renaming.

0.4.0 (2013-06-25)
------------------
