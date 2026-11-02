"""Livox Mid-360 simulation: Gazebo + bridge + livox_driver_emulator (+ robot, RViz).

ros2 launch livox_mid360_sim sim.launch.py [gui:=true] [rviz:=true] [spawn_robot:=true]
    [xfer_format:=1] [world:=<path or name in share/livox_mid360_sim/worlds>]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The bare sensor (spawn_robot:=false): model.sdf on a 0.5 m pole fixed to the world.
# This is the exact path ignc-kit uses: <include><uri>model://livox_mid360</uri> plus a
# joint to livox_mid360::lidar_link.
STAND_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="livox_stand">
  <link name="pole">
    <pose>0 0 0.25 0 0 0</pose>
    <inertial><mass>1.0</mass></inertial>
    <collision name="pole_collision"><geometry><cylinder><radius>0.02</radius>
      <length>0.5</length></cylinder></geometry></collision>
    <visual name="pole_visual"><geometry><cylinder><radius>0.02</radius>
      <length>0.5</length></cylinder></geometry></visual>
  </link>
  <joint name="pole_to_world" type="fixed"><parent>world</parent><child>pole</child></joint>
  <include><uri>model://livox_mid360</uri><pose>0 0 0.5 0 0 0</pose></include>
  <joint name="livox_mount" type="fixed">
    <parent>pole</parent><child>livox_mid360::lidar_link</child>
  </joint>
</model></sdf>"""


def _truthy(s):
    return s.strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    pkg = get_package_share_directory('livox_mid360_sim')
    gui = _truthy(context.launch_configurations['gui'])
    rviz = _truthy(context.launch_configurations['rviz'])
    spawn_robot = _truthy(context.launch_configurations['spawn_robot'])
    world = context.launch_configurations['world']
    if not os.path.isabs(world) and not os.path.exists(world):
        world = os.path.join(pkg, 'worlds', world)
    dense_rate = context.launch_configurations['dense_rate']

    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='livox_gz_bridge',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                          'use_sim_time': True}],
             output='screen'),
        Node(package='livox_mid360_sim', executable='livox_driver_emulator',
             parameters=[{
                 'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'),
                                               value_type=int),
                 'frame_id': 'livox_frame',
                 'use_sim_time': True}],
             output='screen'),
    ]

    if spawn_robot:
        import xacro
        robot_description = xacro.process_file(
            os.path.join(pkg, 'urdf', 'test_rig.urdf.xacro'),
            mappings={'dense_rate': dense_rate,
                      'range_noise': context.launch_configurations['range_noise']}).toxml()
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'test_rig',
                            '-z', '0.1'],
                 output='screen'),
        ]
    else:
        actions += [
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-string', STAND_SDF, '-name', 'livox_stand'], output='screen'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--frame-id', 'odom', '--child-frame-id', 'livox_frame',
                            '--z', '0.547'],
                 parameters=[{'use_sim_time': True}]),
        ]

    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', os.path.join(pkg, 'config', 'livox.rviz')],
                            parameters=[{'use_sim_time': True}], output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('spawn_robot', default_value='true',
                              description='true: diff-drive test rig; false: bare '
                                          'model://livox_mid360 on a static stand'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='1 = CustomMsg (default), 0 = PointCloud2'),
        DeclareLaunchArgument('world', default_value='livox_test.sdf',
                              description='World file (absolute or in share/worlds)'),
        DeclareLaunchArgument('dense_rate', default_value='10',
                              description='Render rate [Hz] of the dense gpu_lidar'),
        DeclareLaunchArgument('range_noise', default_value='0.02',
                              description='Range noise stddev [m] (test rig only)'),
        OpaqueFunction(function=_setup),
    ])
