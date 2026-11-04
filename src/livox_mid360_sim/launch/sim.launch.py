# Copyright 2026 Kartik Krishnan
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Harmonic + Livox Mid-360 test rig + ros_gz_bridge + driver emulator (+ RViz).

The Gazebo server and GUI run as separate processes, which is required on
macOS and works the same on Linux.
"""

import os
import platform

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_mid360_sim')
    pkg_prefix = get_package_prefix('livox_mid360_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_robot = LaunchConfiguration('spawn_robot')
    verbosity = LaunchConfiguration('verbosity')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'test_rig.urdf.xacro']),
                 ' lidar_mode:=', LaunchConfiguration('lidar_mode'),
                 ' slices:=', LaunchConfiguration('slices')]),
        value_type=str)

    actions = []
    if platform.system() == 'Darwin' and 'GZ_IP' not in os.environ:
        # gz-transport multicast discovery fails on macOS; keep it on loopback.
        actions.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))

    return LaunchDescription(actions + [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds', 'livox_test.sdf'),
            description='SDF world to load'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'spawn_robot', default_value='true',
            description='Spawn the diff-drive test rig carrying the Mid-360'),
        DeclareLaunchArgument(
            'xfer_format', default_value='1',
            description='/livox/lidar type like livox_ros_driver2: 1 CustomMsg, 0 PointCloud2'),
        DeclareLaunchArgument(
            'lidar_mode', default_value='sliced',
            choices=['snapshot', 'sliced', 'raycast'],
            description='How ranges are measured: 1 GPU image per frame (snapshot), '
                        '`slices` GPU images per frame (sliced) or CPU ray casting (raycast)'),
        DeclareLaunchArgument(
            'slices', default_value='5', description='GPU images per frame in sliced mode'),
        DeclareLaunchArgument('verbosity', default_value='3', description='Gazebo log level 0-4'),

        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(pkg_prefix, 'lib')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models')),

        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '-v', verbosity, world],
            output='screen', name='gz_server'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v', verbosity],
            output='screen', name='gz_gui', condition=IfCondition(gui)),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen', condition=IfCondition(spawn_robot)),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', 'robot_description', '-name', 'livox_test_rig', '-z', '0.02'],
            output='screen', condition=IfCondition(spawn_robot)),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='livox_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
                'use_sim_time': True,
            }],
            output='screen'),
        Node(
            package='livox_mid360_sim', executable='livox_driver_emulator',
            parameters=[{
                'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'), value_type=int),
                'use_sim_time': True,
            }],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'livox.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz)),
    ])
