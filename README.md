# Livox Mid-360 simulation (ROS 2 Jazzy + Gazebo Harmonic)

A Livox Mid-360 lidar + IMU for gz-sim 8 whose ROS output matches what
`livox_ros_driver2` publishes on a real Mid-360. It uses the recorded,
non-repetitive Mid-360 scan pattern, gives every point its own timestamp and laser
line, and reports IMU acceleration in g.

```
src/
  livox_ros_driver2/   messages only (CustomMsg, CustomPoint), identical to upstream
  livox_mid360_sim/    gz-sim system plugin, model, xacro, world, launch, emulator, checks
```

## Install (Ubuntu 24.04)

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

`ros-jazzy-ros-gz` pulls in Gazebo Harmonic through the vendor packages
(`gz_sim_vendor`, `gz_plugin_vendor`, …). No separate Gazebo install or Livox-SDK2 is needed.

## Build

```bash
cd livox-mid-360-simulation
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash     # also sets GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH
```

## Run

```bash
ros2 launch livox_mid360_sim sim.launch.py                 # Gazebo GUI + RViz + test rig
ros2 run teleop_twist_keyboard teleop_twist_keyboard       # drive it (second terminal)
```

Launch arguments:

| arg | default | |
|---|---|---|
| `gui` | `true` | Gazebo GUI (`false` = headless server with EGL rendering) |
| `rviz` | `true` | RViz with `config/livox.rviz` |
| `spawn_robot` | `true` | `true`: diff-drive test rig (Mid-360 on a 0.45 m mast). `false`: the bare `model://livox_mid360` on a pole, jointed to `livox_mid360::lidar_link`, which is the ignc-kit integration path |
| `xfer_format` | `1` | `1` = CustomMsg, `0` = PointCloud2 (driver layout) on `/livox/lidar` |
| `world` | `livox_test.sdf` | world file (absolute path or a name in `share/livox_mid360_sim/worlds`) |
| `geometry_source` | `collision` | geometry the rays hit: `collision` or `visual` (test rig) |
| `range_noise` | `0.02` | range noise σ [m] (test rig; `0` for accuracy tests) |

## Check

```bash
ros2 run livox_mid360_sim smoke_test.sh                    # headless: launch, check, report, tear down
ros2 run livox_mid360_sim smoke_test.sh xfer_format:=0
SMOKE_GROUND_Z=-0.547 ros2 run livox_mid360_sim smoke_test.sh spawn_robot:=false
ros2 run livox_mid360_sim check_livox_cloud.py --ground-z -0.722   # against a running sim
```

`check_livox_cloud.py` exits non-zero on failure. It checks:

* CustomMsg rate of about 10 Hz in sim time
* points per frame
* `frame_id`
* `header.stamp == timebase == t(first point)`
* no (0,0,0) points
* elevation within −7.5..52.5°
* azimuth coverage of at least 30/36 sectors
* `line == {0,1,2,3}`
* `offset_time` within 0..100 ms, spanning more than 90 ms, and non-decreasing
* non-repetitive coverage: 5 frames cover more than 1.5× the 0.5° cells of one frame
* PointCloud2 fields and `point_step == 26`
* IMU at 200 Hz, |a| ≈ 1 g, IMU frame `livox_frame`
* optionally, that points below −5.5° lie on the ground plane (`--ground-z`)

`smoke_test.sh` also prints the gz server's CPU%, the real-time factor and the GPU load.

## Topics and frames

| topic | type | |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` (xfer_format 1) or `sensor_msgs/msg/PointCloud2` (0) | 10 Hz, ≤ 20 000 points, frame `livox_frame` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 200 Hz, angular velocity in rad/s, **linear acceleration in g**, frame `livox_frame`, no orientation |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` | internal: gz output (26-byte Livox layout), bridged |
| `/livox/imu_raw` | `sensor_msgs/msg/Imu` | internal: gz IMU in m/s², bridged |
| `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/clock` | | test rig and sim time |

Per-point contract, matching `livox_ros_driver2` on a Mid-360:

* CustomMsg: `header.stamp == timebase ==` time of the first point. `offset_time` is in ns
  relative to `timebase`, spans 0–100 ms and never decreases. `line` is 0–3, `tag` is 0, and
  `reflectivity` is 0–255. Misses are dropped and (0,0,0) is never published.
* PointCloud2 (xfer_format 0): packed 26-byte points `x,y,z,intensity` float32 @0,4,8,12,
  `tag` uint8 @16, `line` uint8 @17, `timestamp` float64 @18 (absolute ns).
* 200 000 pts/s, i.e. 20 000 rays per 100 ms frame. FOV 360° × −7°..+52°. Range 0.1–40 m at
  10 % reflectivity, at most 70 m (detection range ∝ √reflectivity). Range noise σ = 2 cm,
  applied along the ray.

Frames:

* `livox_frame` is the optical centre, 47 mm above the mounting face.
* The IMU sits at (0.011, 0.02329, −0.04412) m in `livox_frame`, so FAST-LIO's `extrinsic_T`
  is `[-0.011, -0.02329, 0.04412]` with identity `extrinsic_R`.

Data path:

```
gz: LivoxMid360 system ──/livox/points (PointCloudPacked, 26 B)──┐
    gz Imu sensor ───────/livox/imu_raw (IMU, m/s²)───────────────┤ ros_gz_bridge
                                                                  ▼
            livox_driver_emulator ──► /livox/lidar (CustomMsg | PointCloud2)
                                  └─► /livox/imu (g, livox_frame)
```

The conversion is lossless because every point carries its own line and absolute timestamp.

## Scan pattern

`models/livox_mid360/scan_mode/mid360.csv` is the recorded Mid-360 pattern
(800 000 rows = 4 s at 200 k pts/s). The system plugin loads it from
`model://livox_mid360/scan_mode/mid360.csv`, which it resolves through `GZ_SIM_RESOURCE_PATH`.
You can override it with `<pattern_file>` (a `model://`, `file://` or plain path). If the CSV
can't be found, the plugin logs a warning and falls back to a procedural pattern: 4 stacked
lasers 3° apart at the same azimuth, rotating non-repetitively.

How the CSV is read:

* The first column is an index, so point *i* is emitted at *i*/200 000 s.
* Elevation = 90° − zenith. Azimuth is counter-clockwise from +x.
* Rows come in groups of 4 lasers, so `line = row % 4`. The 4-laser group is a short line
  that rotates over the pattern: within a group the lasers are 1.5–3° apart in elevation and
  spread 0–8° in azimuth.
* Successive frames never repeat. 5 frames cover about 3× the cells of one.

**Attribution:** `mid360.csv` comes unmodified from
[ggory15/livox_laser_simulation_ros2](https://github.com/ggory15/livox_laser_simulation_ros2)
(a fork of [LCAS/livox_laser_simulation_ros2](https://github.com/LCAS/livox_laser_simulation_ros2)),
MIT License, Copyright (c) 2023 Ricardo Casimiro. See `scan_mode/LICENSE` and
`scan_mode/README.md`. Only the data is reused. That plugin's issues (wall-clock
`offset_time`, line/tag always 0, (0,0,0) for misses, 400 k pts/s, no IMU, noise never
applied) are not.

The `livox_ros_driver2` message definitions are copied from
[Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (MIT, see
`src/livox_ros_driver2/LICENSE.txt`).

## How the sensor works (branch `approach-2-exact-raycast`)

There is no `gpu_lidar` on this branch. The `livox_mid360_sim::LivoxMid360` system casts
**one exact ray per pattern direction** on the CPU:

* **Timing:** every physics step it casts exactly the rays whose timestamps
  `t0 + i · 5 µs` fall into that step (200 rays per 1 ms step). Each ray starts from the
  sensor pose **interpolated to its own time** (lerp + slerp between the previous and
  current step), so motion skew is exact, not tied to a render rate. A frame of 20 000 rays
  is published the moment its last ray is cast.
* **Geometry:** a custom ray caster over the shapes read from the ECM. It hits **collision**
  geometry by default; `<geometry_source>visual</geometry_source>` (launch
  `geometry_source:=visual`) uses visuals. Box, sphere, cylinder, capsule, ellipsoid and plane
  are intersected analytically. Meshes go through a per-mesh triangle BVH, loaded with
  gz-common's MeshManager and cached by uri + scale. A top-level BVH over the world AABBs is
  rebuilt only when a non-static shape moves. New and removed entities (e.g. a robot spawned
  later) are picked up automatically. Heightmaps, polylines and cones are skipped with a
  warning.
* Rays start at `min_range` from the optical centre, like gz lidars, so the housing is never
  seen from inside. The robot's own body is visible, as on a real robot.
* The output contract, return model (detection range, noise, reflectivity) and IMU are
  identical to approach 1.

Plugin parameters (`model.sdf`): `link_name` (`lidar_link`), `frame_offset`
(`0 0 0.047 0 0 0`: livox_frame relative to the link), `geometry_source`, `frame_id`,
`points_topic`, `pattern_file`, `point_rate`, `frame_rate`, `min_range`, `max_range`,
`nominal_range`, `range_noise_stddev`, `default_reflectivity`, `seed`.

### Which ray-casting back-end, and why

| option (gz-sim 8 / gz-physics 7 / gz-rendering 8) | result |
|---|---|
| `components::RaycastData` + Physics system (`GetRayIntersectionFromLastStep`), dartsim with its default ODE collision detector | **does not work**: `Raycast is not supported by 'ode'`, no hits |
| same, with `<dart><collision_detector>bullet</collision_detector></dart>` | works on collision geometry, but costs **2.6 µs/ray**, serial inside the physics step. 200 k rays/s adds about 0.52 s of CPU per simulated second, and the *whole world's* contact model has to switch to Bullet |
| gz-rendering `RayQuery` (Ogre2) | one ray per call. The GPU path renders a selection buffer per query (mouse picking); the CPU path does per-object Ogre intersection. It also needs the render thread's scene, which a system plugin doesn't own. Not viable at 200 k rays/s |
| **custom BVH over ECM geometry** (chosen) | **0.33 µs/ray** single-threaded, exact primitives, meshes supported, no side effects on physics or rendering, collision or visual geometry |

### Measured on this machine (AMD Ryzen 5 2600 Six-Core Processor, GTX 1050 Ti 4 GB)

* **Throughput:** 0.33 µs/ray steady-state. At 200 k rays/s the sensor costs about 133 µs
  per 1 ms step (about 13 % of one core), including the per-step scene update. Headless at
  unlimited speed, the world runs 20 s of sim time in 3.6 s without the lidar, 7.7 s with
  it at 200 k rays/s, and 17.7 s at **2 M rays/s**. That is 10× the Mid-360 rate and still
  faster than real time, single-threaded.
* **Full launch** (`benchmark.sh`, test rig): **RTF 1.00, gz server 119 % CPU, no GPU
  memory** beyond the desktop's. Approach 1 at 50 Hz: RTF 1.00, 212–219 % CPU, about
  170 MiB GPU.
* **Accuracy** (`eval_accuracy.py`, `range_noise:=0`): static, 10 frames, 78 594 points:
  median, p99 and max error are **0.00 mm** on ground, walls, overhang, primitives and the
  OBJ mesh, with **0 missed and 0 spurious returns**. The evaluator is an independent numpy
  ray caster over the same SDF, so this shows the gz side (poses, collision extraction,
  mesh loading, interpolation) adds no error. It is not an independent physical
  measurement.
* **Motion skew** (rig spinning at 1 rad/s, truth from the 1 kHz ground-truth pose at each
  ray's time): **0.00 mm** median and no missed or spurious returns in every quarter of the
  frame. Truth with one pose per frame is off by 9 / 134 / 197 / 324 mm per quarter, so the
  skew is fully present and exactly where it should be.

Because it hits **collision** geometry, the cloud shows what physics sees. Where visual
meshes are detailed and collisions are simplified boxes, the cloud follows the boxes; use
`geometry_source:=visual` to follow the visual meshes instead, which costs more for big
meshes. Approach 1 always sees visuals.

## Accuracy against analytic geometry

`eval_accuracy.py` ray-casts the world's collision shapes exactly from the true sensor pose,
which it queries from gz. It finds the pattern block of each frame from the timestamps and
compares all 20 000 rays per frame with ground truth:

```bash
ros2 launch livox_mid360_sim sim.launch.py gui:=false rviz:=false range_noise:=0.0
ros2 run livox_mid360_sim eval_accuracy.py --frames 20
```

It reports the range error per surface (ground, walls, overhang, objects), missed returns
(truth hits within 40 m but no point) and spurious returns (a point where truth hits nothing).

For motion skew, drive the rig (e.g.
`ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"`) and run
`eval_accuracy.py --moving`. Truth is computed from the 1 kHz ground-truth pose
(`/ground_truth/pose`) at each ray's timestamp. `benchmark.sh "" "geometry_source:=visual"`
measures RTF, gz CPU% and GPU load per launch configuration.

## Integration into ignc-kit

1. Copy **only** `src/livox_mid360_sim` into the ignc-kit workspace. **Do not copy
   `src/livox_ros_driver2`**: ignc-kit uses the real `livox_ros_driver2`, which has the same
   package name. Two packages with one name break the build, and the definitions are
   identical (same type hashes), so the real one is a drop-in.
2. In the world or robot SDF:
   ```xml
   <include><uri>model://livox_mid360</uri><pose>…</pose></include>
   <joint name="livox_mount" type="fixed">
     <parent>…your link…</parent><child>livox_mid360::lidar_link</child>
   </joint>
   ```
   The model origin (`lidar_link`) is the centre of the mounting face. On this branch the world needs only the `Imu`
   system for the sensor: no rendering and no `Sensors` system. In URDF, use the
   `urdf/livox_mid360.urdf.xacro` macro instead
   (`<xacro:livox_mid360 parent="…" xyz="…" rpy="…"/>`).
3. Bridge `/livox/points` and `/livox/imu_raw` (see `config/bridge.yaml`) and run
   `livox_driver_emulator` (`xfer_format:=1`).
4. FAST-LIO: `lidar_type: 1` (Livox), `lid_topic: /livox/lidar`, `imu_topic: /livox/imu`,
   `extrinsic_T: [-0.011, -0.02329, 0.04412]`, `extrinsic_R: identity`. The old
   `pointcloud2_to_livox` adapter is no longer needed, because `/livox/lidar` is already a
   CustomMsg with real per-point times and lines.
