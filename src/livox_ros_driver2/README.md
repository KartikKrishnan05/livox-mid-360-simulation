# livox_ros_driver2 (messages only)

A stand-in that provides only `livox_ros_driver2/msg/CustomMsg` and
`livox_ros_driver2/msg/CustomPoint`, so the simulation can publish exactly what the
real driver publishes without building Livox-SDK2.

* `msg/CustomMsg.msg` and `msg/CustomPoint.msg` are unmodified copies from
  <https://github.com/Livox-SDK/livox_ros_driver2> (MIT, see `LICENSE.txt`).
  Identical definitions give identical ROS 2 type hashes, so a bag recorded from the
  simulation plays into the real driver's consumers and vice versa.

**Do not copy this package into a workspace that already contains the real
`livox_ros_driver2`.** Both packages have the same name; colcon will refuse to build
(or one will silently shadow the other). In such a workspace, use the real driver for
the message definitions and copy only `livox_mid360_sim`.
