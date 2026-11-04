"""Livox Mid-360 simulation: Gazebo + bridge + livox_driver_emulator (+ robot, RViz).

ros2 launch livox_mid360_sim sim.launch.py [sensor:=raycast|gpu|modes] [robot:=test_rig|scout_mini]
    [lidar_mode:=snapshot|sliced|raycast] [slices:=5]
    [gui:=true] [rviz:=true] [spawn_robot:=true] [xfer_format:=1] [world:=<path or name in share/livox_mid360_sim/worlds>]

sensor:=raycast  exact CPU ray per pattern direction (approach 2, default, recommended)
sensor:=gpu      dense gpu_lidar rendered at dense_rate Hz, sampled along the pattern (approach 1)
sensor:=modes    one plugin, three lidar modes (approach 3), chosen with lidar_mode:=
                   snapshot  1 dense GPU image per frame
                   sliced    `slices` GPU images per frame, each stamped with its image time
                   raycast   CPU ray casting against the visual geometry, per-ray timing
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

# Sensor implementations (launch argument "sensor"): description, model for spawn_robot:=false.
SENSORS = {
    'raycast': ('raycast (approach 2) - exact CPU ray per pattern direction',
                'model://livox_mid360'),
    'gpu': ('gpu (approach 1) - dense GPU render, nearest-render sampling',
            'model://livox_mid360_gpu'),
    'modes': ('modes (approach 3) - one plugin with snapshot / sliced / raycast lidar modes',
              'model://livox_mid360_modes'),
}
LIDAR_MODES = ('snapshot', 'sliced', 'raycast')

# Robots (launch argument "robot"): xacro file, spawn height of base_link [m].
ROBOTS = {
    'test_rig': ('test_rig.urdf.xacro', '0.1'),      # diff-drive rig, Mid-360 on a 0.45 m mast
    'scout_mini': ('scout_mini.urdf.xacro', '0.180'),  # AgileX Scout Mini (rests at 0.1805)
}

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
  <include><uri>{model}</uri><name>livox_mid360</name><pose>0 0 0.5 0 0 0</pose></include>
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
    sensor = context.launch_configurations['sensor'].strip().lower()
    if sensor not in SENSORS:
        raise RuntimeError(f"sensor:={sensor} is not one of {', '.join(SENSORS)}")
    description, stand_model = SENSORS[sensor]
    robot = context.launch_configurations['robot'].strip()
    if robot not in ROBOTS:
        raise RuntimeError(f"robot:={robot} is not one of {', '.join(ROBOTS)}")
    robot_xacro, spawn_z = ROBOTS[robot]

    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'
    lidar_mode = context.launch_configurations['lidar_mode'].strip().lower()
    slices = context.launch_configurations['slices'].strip()
    if sensor == 'modes':
        if lidar_mode not in LIDAR_MODES:
            raise RuntimeError(f"lidar_mode:={lidar_mode} is not one of {', '.join(LIDAR_MODES)}")
        if not slices.isdigit() or int(slices) < 1:
            raise RuntimeError(f'slices:={slices} must be a positive integer')
    extra = ''
    if sensor == 'gpu':
        extra = f" at {dense_rate} Hz" if spawn_robot else ' at 50 Hz (model.sdf)'
    elif sensor == 'modes':
        if not spawn_robot:
            extra = ', lidar_mode sliced, 5 slices (model.sdf; lidar_mode/slices need spawn_robot)'
        elif lidar_mode == 'sliced':
            extra = f', lidar_mode sliced, {slices} slices'
        else:
            extra = f', lidar_mode {lidar_mode}'
    actions = [
        LogInfo(msg=f'Livox Mid-360 sensor: {description}{extra}'),
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
            os.path.join(pkg, 'urdf', robot_xacro),
            mappings={'sensor': sensor,
                      'geometry_source': context.launch_configurations['geometry_source'],
                      'dense_rate': dense_rate,
                      **({'mesh_collision': context.launch_configurations['mesh_collision']}
                         if robot == 'scout_mini' else {}),
                      'range_noise': context.launch_configurations['range_noise'],
                      'dense_h': context.launch_configurations['dense_h'],
                      'dense_v': context.launch_configurations['dense_v'],
                      'lidar_mode': lidar_mode,
                      'slices': slices}).toxml()
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', robot,
                            '-z', spawn_z],
                 output='screen'),
        ]
    else:
        actions += [
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-string', STAND_SDF.replace('{model}', stand_model),
                            '-name', 'livox_stand'], output='screen'),
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
        DeclareLaunchArgument('sensor', default_value='raycast',
                              description='raycast (exact CPU rays, approach 2) | '
                                          'gpu (dense GPU render, approach 1) | '
                                          'modes (approach 3, see lidar_mode)'),
        DeclareLaunchArgument('lidar_mode', default_value='sliced',
                              description='modes: snapshot (1 GPU image/frame) | sliced '
                                          '(`slices` GPU images/frame) | raycast (CPU rays)'),
        DeclareLaunchArgument('slices', default_value='5',
                              description='modes, lidar_mode:=sliced: GPU images per frame'),
        DeclareLaunchArgument('robot', default_value='test_rig',
                              description='test_rig (Mid-360 on a 0.45 m mast) | '
                                          'scout_mini (AgileX Scout Mini, Mid-360 on the deck)'),
        DeclareLaunchArgument('mesh_collision', default_value='false',
                              description='scout_mini: original mesh collisions instead of '
                                          'fitted primitives'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('spawn_robot', default_value='true',
                              description='true: diff-drive test rig; false: bare '
                                          'model://livox_mid360 on a static stand'),
        DeclareLaunchArgument('xfer_format', default_value='1',
                              description='1 = CustomMsg (default), 0 = PointCloud2'),
        DeclareLaunchArgument('world', default_value='livox_test.sdf',
                              description='World file (absolute or in share/worlds)'),
        DeclareLaunchArgument('dense_rate', default_value='50',
                              description='gpu: render rate [Hz] of the dense gpu_lidar'),
        DeclareLaunchArgument('dense_h', default_value='1440',
                              description='Dense gpu_lidar horizontal samples (test rig)'),
        DeclareLaunchArgument('dense_v', default_value='160',
                              description='Dense gpu_lidar vertical samples (test rig)'),
        DeclareLaunchArgument('geometry_source', default_value='collision',
                              description='raycast: collision | visual geometry (test rig)'),
        DeclareLaunchArgument('range_noise', default_value='0.02',
                              description='Range noise stddev [m] (test rig only)'),
        OpaqueFunction(function=_setup),
    ])
