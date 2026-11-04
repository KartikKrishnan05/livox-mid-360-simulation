cmake_minimum_required(VERSION 3.16)
project(livox_mid360_sim)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Gazebo Harmonic through the ROS 2 Jazzy vendor packages.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# --- gz-sim system plugins: LivoxMid360Gpu (approach 1), LivoxMid360Raycast (approach 2) ---
add_library(livox_mid360_sim_system SHARED
  src/livox_mid360_gpu_system.cpp
  src/livox_mid360_raycast_system.cpp
  src/raycaster.cpp
  src/scan_pattern.cpp
)
target_include_directories(livox_mid360_sim_system PRIVATE src)
target_link_libraries(livox_mid360_sim_system
  gz-sim::gz-sim
  gz-plugin::register
)
install(TARGETS livox_mid360_sim_system LIBRARY DESTINATION lib)

# --- gz-sim system plugin: LivoxMid360Modes (approach 3: snapshot | sliced | raycast) -----
# Own library and namespace (livox_mid360_sim::modes): its ScanPattern / ray caster are a
# separate implementation from approach 2's.
add_library(livox_mid360_modes_system SHARED
  src/modes/livox_mid360_modes_system.cpp
  src/modes/raycaster.cpp
  src/modes/scan_pattern.cpp
)
target_include_directories(livox_mid360_modes_system PRIVATE src/modes)
target_link_libraries(livox_mid360_modes_system
  gz-sim::gz-sim
  gz-plugin::register
)
install(TARGETS livox_mid360_modes_system LIBRARY DESTINATION lib)

# --- ROS 2 node: gz output -> livox_ros_driver2 topics --------------------------------
add_executable(livox_driver_emulator src/livox_driver_emulator.cpp)
target_link_libraries(livox_driver_emulator
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
)
install(TARGETS livox_driver_emulator DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/check_livox_cloud.py
  scripts/smoke_test.sh
  scripts/eval_accuracy.py
  scripts/benchmark.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch models urdf worlds DESTINATION share/${PROJECT_NAME})

# GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH when the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
