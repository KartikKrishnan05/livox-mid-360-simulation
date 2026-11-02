cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)

# Messages-only stand-in for https://github.com/Livox-SDK/livox_ros_driver2.
# msg/*.msg are identical copies of the upstream files (MIT, see LICENSE.txt).
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
