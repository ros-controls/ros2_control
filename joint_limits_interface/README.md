## Joint Limits Interface ##

### Overview ###

**joint_limits_interface** contains data structures for representing joint limits, methods for populating them from common
formats such as URDF and the ROS parameter server, and methods for enforcing limits on different kinds of hardware interfaces.

The **joint_limits_interface** is *not* used by controllers themselves (it does not implement a `HardwareInterface`) but
instead operates after the controllers have updated, in the `write()` method (or equivalent) of the robot abstraction.
Enforcing limits will *overwrite* the commands set by the controllers, it does not operate on a separate raw data buffer.

There are two main elements involved in setting up a joint limits interface:

 - **Joint limits representation**
   - **JointLimits** Position, velocity, acceleration, jerk and effort.
   - **SoftJointLimits** Soft position limits, `k_p`, `k_v` (as described [here](http://www.ros.org/wiki/pr2_controller_manager/safety_limits)).
   - **Loading from URDF** There are convenience methods for loading joint limits information
     (position, velocity and effort), as well as soft joint limits information from the URDF.
   - **Loading from ROS params** There are convenience methods for loading joint limits from the ROS parameter server
     (position, velocity, acceleration, jerk and effort).Parameter specification is the same used in MoveIt,
     with the addition that we also parse jerk and effort limits.

 - **Joint limits interface**

  - For **effort-controlled** joints, the soft-limits implementation from the PR2 has been ported.
  - For **position-controlled** joints, a modified version of the PR2 soft limits has been implemented.
  - For **velocity-controlled** joints, simple saturation based on acceleration and velocity limits has been implemented.

### Examples ###
Please refer to the  [joint_limits_interface](https://github.com/ros-controls/ros_control/wiki/joint_limits_interface) wiki page.
