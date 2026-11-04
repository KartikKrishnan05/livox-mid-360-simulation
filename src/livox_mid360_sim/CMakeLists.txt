cmake_minimum_required(VERSION 3.19)
project(livox_mid360_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic through the ROS 2 Jazzy vendor packages. These resolve to the
# same gz-sim8 / gz-plugin2 libraries whether they come from apt (Linux) or
# RoboStack (macOS / pixi).
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_library(LivoxMid360System SHARED
  src/livox_mid360_system.cpp
  src/raycaster.cpp
  src/scan_pattern.cpp
)
target_include_directories(LivoxMid360System PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(LivoxMid360System PRIVATE
  gz-sim::gz-sim
  gz-plugin::register
)

install(TARGETS LivoxMid360System
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(livox_driver_emulator src/livox_driver_emulator.cpp)
target_link_libraries(livox_driver_emulator
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
)
if(APPLE)
  # The message *_TARGETS (also rclcpp's) include the *_generator_py libraries,
  # whose Python symbols only resolve inside a Python process. Drop unused
  # dylibs like conda-forge does, otherwise dyld aborts at startup.
  target_link_options(livox_driver_emulator PRIVATE "LINKER:-dead_strip_dylibs")
endif()
install(TARGETS livox_driver_emulator DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the check script with an absolute interpreter path: on macOS a
# `#!/usr/bin/env` shebang drops DYLD_LIBRARY_PATH, which rclpy needs to load
# workspace message libraries.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Re-run CMake when the script changes (file(READ) alone does not track it).
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS scripts/check_livox_cloud.py)
file(READ scripts/check_livox_cloud.py _check_script)
string(REGEX REPLACE "^#![^\n]*" "#!${Python3_EXECUTABLE}" _check_script "${_check_script}")
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/check_livox_cloud.py CONTENT "${_check_script}"
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/check_livox_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

# Puts lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH and the models on GZ_SIM_RESOURCE_PATH
# when install/setup.{bash,zsh,sh} is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
