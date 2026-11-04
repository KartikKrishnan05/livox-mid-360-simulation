"""Livox Mid-360 simulation: Gazebo + the sensor on a static stand + bridge + livox_driver_emulator (+ RViz).

ros2 launch livox_mid360_gazebo sim.launch.py [sensor:=raycast|gpu] [description:=xacro|sdf]
    [gui:=true] [rviz:=true] [xfer_format:=1] [world:=<path or name in share/livox_mid360_gazebo/worlds>]

sensor:=raycast  exact CPU ray per pattern direction (approach 2, default, recommended)
sensor:=gpu      dense gpu_lidar rendered at dense_rate Hz, sampled along the pattern (approach 1)

description:=xacro  urdf/livox_stand.urdf.xacro: the livox_mid360 xacro macro on a 0.5 m pole
                    (the URDF path; all sensor options apply)
description:=sdf    model://livox_mid360[_gpu] included on a 0.5 m pole and jointed to
                    livox_mid360::lidar_link (the SDF path; the model's fixed defaults apply)

Both put livox_frame at z = 0.547 m. To use the sensor on a robot, add the xacro macro or the
SDF include to its description (see the README).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Sensor implementations (launch argument "sensor"): description, SDF model.
SENSORS = {
    'raycast': ('raycast (approach 2) - exact CPU ray per pattern direction',
                'model://livox_mid360'),
    'gpu': ('gpu (approach 1) - dense GPU render, nearest-render sampling',
            'model://livox_mid360_gpu'),
}
DESCRIPTIONS = ('xacro', 'sdf')

# description:=sdf: model.sdf on a 0.5 m pole fixed to the world, the way a world or robot
# SDF adds the sensor: <include><uri>model://livox_mid360</uri> plus a joint to
# livox_mid360::lidar_link.
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
  <include><uri>{model}</uri><name>livox_mid360</name><pose>0 0 0.5 0 0 0</pose></include>
  <joint name="livox_mount" type="fixed">
    <parent>pole</parent><child>livox_mid360::lidar_link</child>
  </joint>
</model></sdf>"""


def _truthy(s):
    return s.strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    pkg = get_package_share_directory('livox_mid360_gazebo')
    cfg = context.launch_configurations
    gui = _truthy(cfg['gui'])
    rviz = _truthy(cfg['rviz'])
    world = cfg['world']
    if not os.path.isabs(world) and not os.path.exists(world):
        world = os.path.join(pkg, 'worlds', world)
    dense_rate = cfg['dense_rate']
    sensor = cfg['sensor'].strip().lower()
    if sensor not in SENSORS:
        raise RuntimeError(f"sensor:={sensor} is not one of {', '.join(SENSORS)}")
    description, sdf_model = SENSORS[sensor]
    desc = cfg['description'].strip().lower()
    if desc not in DESCRIPTIONS:
        raise RuntimeError(f"description:={desc} is not one of {', '.join(DESCRIPTIONS)}")

    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'
    extra = ''
    if sensor == 'gpu':
        extra = f' at {dense_rate} Hz' if desc == 'xacro' else ' at 50 Hz (model.sdf)'
    actions = [
        LogInfo(msg=f'Livox Mid-360 sensor: {description}{extra}, {desc} description'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='livox_gz_bridge',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                          'use_sim_time': True}],
             output='screen'),
        Node(package='livox_mid360_gazebo', executable='livox_driver_emulator',
             parameters=[{
                 'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'),
                                               value_type=int),
                 'frame_id': 'livox_frame',
                 'use_sim_time': True}],
             output='screen'),
    ]

    if desc == 'xacro':
        import xacro
        robot_description = xacro.process_file(
            os.path.join(pkg, 'urdf', 'livox_stand.urdf.xacro'),
            mappings={'sensor': sensor,
                      'geometry_source': cfg['geometry_source'],
                      'dense_rate': dense_rate,
                      'range_noise': cfg['range_noise'],
                      'angular_noise': cfg['angular_noise'],
                      'imu_bias': cfg['imu_bias'],
                      'dense_h': cfg['dense_h'],
                      'dense_v': cfg['dense_v']}).toxml()
        actions += [
            # Publishes world -> ... -> livox_frame on /tf_static from the fixed joints.
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'livox_stand'],
                 output='screen'),
        ]
    else:
        actions += [
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-string', STAND_SDF.replace('{model}', sdf_model),
                            '-name', 'livox_stand'], output='screen'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--frame-id', 'world', '--child-frame-id', 'livox_frame',
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
        DeclareLaunchArgument('sensor', default_value='raycast',
                              description='raycast (exact CPU rays, approach 2) | '
                                          'gpu (dense GPU render, approach 1)'),
        DeclareLaunchArgument('description', default_value='xacro',
                              description='xacro (livox_mid360 macro on a stand) | '
                                          'sdf (model://livox_mid360[_gpu] on a stand)'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='1 = CustomMsg (default), 0 = PointCloud2'),
        DeclareLaunchArgument('world', default_value='livox_test.sdf',
                              description='World file (absolute or in share/worlds)'),
        DeclareLaunchArgument('dense_rate', default_value='50',
                              description='gpu, xacro: render rate [Hz] of the dense gpu_lidar'),
        DeclareLaunchArgument('dense_h', default_value='1440',
                              description='gpu, xacro: dense gpu_lidar horizontal samples'),
        DeclareLaunchArgument('dense_v', default_value='160',
                              description='gpu, xacro: dense gpu_lidar vertical samples'),
        DeclareLaunchArgument('geometry_source', default_value='collision',
                              description='raycast, xacro: collision | visual geometry'),
        DeclareLaunchArgument('range_noise', default_value='0.02',
                              description='xacro: range noise stddev [m] @ 10 m (0 = off)'),
        DeclareLaunchArgument('angular_noise', default_value='0.15',
                              description='xacro: beam direction noise stddev [deg] (0 = off)'),
        DeclareLaunchArgument('imu_bias', default_value='true',
                              description='xacro: IMU turn-on bias and bias drift'),
        OpaqueFunction(function=_setup),
    ])
