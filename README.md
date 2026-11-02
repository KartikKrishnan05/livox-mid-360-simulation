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
| `robot` | `test_rig` | `test_rig` (diff-drive rig, Mid-360 on a 0.45 m mast) or `scout_mini` (AgileX Scout Mini, Mid-360 on the deck) |
| `mesh_collision` | `false` | `scout_mini`: original mesh collisions instead of fitted primitives (see below) |
| `sensor` | `raycast` | `raycast` (exact CPU rays, approach 2) or `gpu` (dense GPU render, approach 1) |
| `geometry_source` | `collision` | `raycast`: geometry the rays hit, `collision` or `visual` (test rig) |
| `dense_rate` | `50` | `gpu`: render rate of the dense gpu_lidar (test rig) |
| `dense_h`, `dense_v` | `1440`, `160` | `gpu`: dense gpu_lidar resolution (test rig) |
| `range_noise` | `0.02` | range noise σ [m] (test rig; `0` for accuracy tests) |

## Robots

```bash
ros2 launch livox_mid360_sim sim.launch.py robot:=scout_mini                # Scout Mini
ros2 launch livox_mid360_sim sim.launch.py robot:=scout_mini sensor:=gpu
```

**`scout_mini`** (`urdf/scout_mini.urdf.xacro`) is the AgileX Scout Mini from ignc-kit's
`gazebo_sim` package. It has the same links, joints, masses, DiffDrive (4 wheels, separation
0.49 m, radius 0.08 m), joint-state and odometry plugins (`/cmd_vel`, `/odom`, `/tf`
odom→base_link, `/joint_states`), and the same meshes (`models/scout_mini`, Apache-2.0). The
Mid-360 is centred on a 10 × 10 cm, 8 mm mounting plate on the top deck. The deck has an open
recess in the middle, which the plate spans.

| frame | height above `base_link` | above ground |
|---|---|---|
| `base_link` | 0 | 0.180 m (wheel centres −0.10, radius 0.08) |
| deck / plate top (= Mid-360 mounting face) | 0.057 / 0.065 m | |
| `livox_frame` (optical centre) | 0.112 m | **0.292 m** |

At −7° the lowest rays clear the deck edges, so the robot doesn't occlude itself. Changes
from the ignc-kit original:

* **Collisions are primitives fitted to the meshes:** a box for the body, cylinders of
  r = 0.08 m and width 0.074 m for the wheel treads. The visuals stay meshes. With the
  original mesh collisions (`mesh_collision:=true`), RTF drops to about 0.9 because of the
  106k-vertex body mesh.
* **The spawn height is 0.180 m**, i.e. resting height. ignc-kit spawns at 0.25 m. A drop
  of even 2 cm onto the 132 kg body shakes the IMU at several g for about 0.5 s, which
  would spoil FAST-LIO's IMU initialisation. At rest the IMU then shows only its configured
  noise (|a| std 0.006 g).
* Odometry frame names are set explicitly (`odom`, `base_link`), and a PosePublisher feeds
  `/ground_truth/pose`.

Checks on the Scout: `SMOKE_GROUND_Z=-0.292 ros2 run livox_mid360_sim smoke_test.sh
robot:=scout_mini [sensor:=gpu]` passes with both sensors at RTF 1.0. The ground points land
at −0.292 m. For accuracy and skew, use `eval_accuracy.py --model scout_mini`.

The meshes are resolved as `package://livox_mid360_sim/models/scout_mini/meshes/…`. Gazebo
finds them because the package's env hook also adds `share/` to `GZ_SIM_RESOURCE_PATH`.

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

## Choosing the sensor

Both implementations are built into one plugin library and produce the same output contract,
return model and IMU. Pick one at launch:

```bash
ros2 launch livox_mid360_sim sim.launch.py                  # sensor:=raycast (default)
ros2 launch livox_mid360_sim sim.launch.py sensor:=gpu      # dense GPU render
```

The launch file prints `Livox Mid-360 sensor: …`, and the plugin prints
`[livox_mid360] sensor: …` when Gazebo loads it. If the two disagree, rebuild and re-source.

| | `sensor:=raycast` (approach 2) | `sensor:=gpu` (approach 1) |
|---|---|---|
| SDF model | `model://livox_mid360` | `model://livox_mid360_gpu` (include it with `<name>livox_mid360</name>`) |
| system plugin | `livox_mid360_sim::LivoxMid360Raycast` (alias `livox_mid360_sim::LivoxMid360`) | `livox_mid360_sim::LivoxMid360Gpu` |
| xacro | `<xacro:livox_mid360 … sensor="raycast"/>` | `<xacro:livox_mid360 … sensor="gpu"/>` |
| options | `geometry_source:=collision\|visual` | `dense_rate:=50`, `dense_h:=1440`, `dense_v:=160` |
| world needs | `Imu` system | `Sensors` (ogre2) + `Imu` systems, a GPU |

The old single-render `main` sensor is the same as `sensor:=gpu dense_rate:=10`.

## Sensor `raycast` (approach 2, default): one exact ray per pattern direction

`model://livox_mid360` (and the xacro with `sensor:=raycast`, the default) has no
`gpu_lidar`. The `livox_mid360_sim::LivoxMid360Raycast` system casts
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

### Measured on this machine (`sensor:=raycast`) (AMD Ryzen 5 2600 Six-Core Processor, GTX 1050 Ti 4 GB)

* **Throughput:** 0.33 µs/ray steady-state. At 200 k rays/s the sensor costs about 133 µs
  per 1 ms step (about 13 % of one core), including the per-step scene update. Headless at
  unlimited speed, the world runs 20 s of sim time in 3.6 s without the lidar, 7.7 s with
  it at 200 k rays/s, and 17.7 s at **2 M rays/s**. That is 10× the Mid-360 rate and still
  faster than real time, single-threaded.
* **Full launch** (`benchmark.sh`, test rig): **RTF 1.00, gz server 119 % CPU, no GPU
  memory** beyond the desktop's. `sensor:=gpu` at 50 Hz: RTF 1.00, 212–219 % CPU, about
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

## Sensor `gpu` (approach 1): dense GPU render sampled along the pattern

`model://livox_mid360_gpu` (or the xacro with `sensor:=gpu`) carries a dense `gpu_lidar`: 1440 × 160 rays over 360° × −7.5..52.5°, i.e. 0.25° ×
0.377°, rendered by ogre2 on the GPU at **50 Hz** (`dense_rate`). The
`livox_mid360_sim::LivoxMid360Gpu` system buffers those renders (`gz.msgs.LaserScan`) and
publishes one frame of 20 000 pattern rays every 100 ms:

* point *i* of the recorded pattern is emitted at `timebase + i · 5 µs`;
* it is sampled from the render **nearest its own timestamp**. A render is expressed in the
  sensor frame at its render time, so motion skew inside a frame is real, to within half a
  render period (±10 ms at 50 Hz);
* a frame is built as soon as a render at or after its last point exists, i.e. at most
  one render period after the frame ends;
* sampling uses **bilinear** interpolation in 1/r when the four neighbouring pixels lie on
  one continuous surface (spread ≤ 10 %), and **nearest neighbour** at depth edges and
  hit/miss borders, so no "ghost" points float between foreground and background;
* the return model then applies the reflectivity-dependent detection range, the range noise
  and `min_range`.

**LaserScan layout (verified):** `ranges[v * count + h]`, with `h = 0` at `angle_min`
(counter-clockwise) and `v = 0` at `vertical_angle_min`. The proof uses the ground plane:
with this indexing, points below −5.5° land at median z = −0.546 m for a sensor at 0.547 m
(bare model on its pole) and at −0.716 m for the test rig, whose sensor gz reports at exactly
0.722 m. A flipped `v` would put those points high above the sensor (the `ground` check in
`check_livox_cloud.py`). The residual few mm come from depth-render quantization at grazing
incidence (see below).

**The pattern sweeps elevation within each frame.** In every 100 ms block of `mid360.csv`,
the mean elevation runs from +44° (0–10 ms) down to −2° (40–60 ms) and back to +44°
(90–100 ms). Motion skew therefore shows mostly in the middle of a frame, where the lasers
see walls.

### Measured on this machine (`sensor:=gpu`) (AMD Ryzen 5 2600 Six-Core Processor, GTX 1050 Ti 4 GB, headless EGL)

Performance (`benchmark.sh`, test world, rig standing still). CPU% is the gz server process
(100 % = one core). GPU memory is the whole GPU, including about 300 MiB used by the desktop.

| dense render | rate | RTF | gz CPU | GPU util | GPU mem |
|---|---|---|---|---|---|
| 1440 × 160 | 10 Hz | 1.00 | 136 % | 10 % | 471 MiB |
| 1440 × 160 | **50 Hz (default)** | **1.00** | 212–219 % | 17–28 % | 461–471 MiB |
| 1440 × 160 | 100 Hz | 0.92 | 218 % | 31 % | 471 MiB |
| 2880 × 160 | 10 Hz | 1.00 | 153 % | 17 % | 531 MiB |
| 2880 × 160 | 50 Hz | 0.96 | 225 % | 27 % | 531 MiB |
| 2880 × 160 | 100 Hz | 0.91 | 208 % | 11 % | 531 MiB |

The sensor publishes 200 k rays/s. The GPU renders 11.5 M (1440 × 160 × 50) rays/s for them.
The limit is the CPU side of the render pipeline: sensor update, GPU→CPU readback, and the
large `LaserScan` message. The GPU itself is not saturated.

Motion skew. The rig spins in place at 1 rad/s (ground truth ≈ 1.1 rad/s) with
`range_noise:=0`. The table gives the median |range error| on walls and vertical objects
against analytic truth computed with the sensor pose **at each ray's own time**, against
truth computed with **one pose per frame** (at `timebase`):

| dense rate | truth | 0–25 ms | 25–50 ms | 50–75 ms | 75–100 ms |
|---|---|---|---|---|---|
| 10 Hz | per-ray pose | 12 | 147 | 128 | 8 |
| 50 Hz | per-ray pose | 5 | **11** | **10** | 3 |
| 100 Hz | per-ray pose | 3 | 5 | 7 | 2 |
| any | one pose per frame | 2–9 | 7–135 | 191–321 | 153–342 |

So at 50 Hz the cloud follows the true per-point pose to about 1 cm (the ±10 ms render
granularity). Ignoring skew would be wrong by 13–34 cm. 10 Hz is effectively unskewed.

Geometric accuracy (static, `range_noise:=0`, 10 frames, `eval_accuracy.py`):

| render | surface | bias | median \|e\| | p95 | p99 | > 20 cm |
|---|---|---|---|---|---|---|
| 1440 × 160 | walls | −19 mm* | 1.4 mm | 6.4 mm | 23 mm | 0.43 % |
| 1440 × 160 | ground (grazing, −5..−7°) | −33 mm | 51 mm | 189 mm | 460 mm | 4.3 % |
| 1440 × 160 | overhang (from below) | −0.2 mm | 1.1 mm | 6.5 mm | 11 mm | 0 % |
| 1440 × 160 | objects (pillars, boxes, sphere, posts, OBJ mesh) | | 1.4 mm | 14 mm | 3.7 m | 2.3 % |
| 2880 × 160 | walls | | 0.7 mm | 3.5 mm | 6.6 mm | 0.13 % |
| 2880 × 160 | ground | | 23 mm | 112 mm | 319 mm | 2.0 % |

\* The bias and the large p99 come from a few edge points that took the wrong surface (a
pillar edge sampled from the wall behind it, or the reverse). Missed returns: 0.55 %.
Spurious returns: 0.33 %. Both sit at silhouette edges where the render resolution can't
resolve the edge.

Where the error comes from: ogre2 renders a depth cube map whose texel size follows the
**horizontal** sample count. Each value is the depth of the nearest texel's direction, not of
the exact pattern direction. At grazing incidence (ground at −6°: dr/dθ ≈ 65 m/rad) a few
hundredths of a degree become centimetres. Doubling `dense_h` halves the error; doubling
`dense_v` does not help. Use `dense_h:=2880` when accuracy matters more than real time.

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
(`/ground_truth/pose`) at each ray's timestamp. `benchmark.sh "" "range_noise:=0" …`
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
   This is the ray-casting sensor; the world only needs the `Imu` system. For the GPU sensor,
   use `<include><uri>model://livox_mid360_gpu</uri><name>livox_mid360</name>…</include>`
   (same `livox_mid360::lidar_link`); the world then also needs the `Sensors` system (ogre2).
   The model origin (`lidar_link`) is the centre of the mounting face. In URDF, use the
   `urdf/livox_mid360.urdf.xacro` macro instead
   (`<xacro:livox_mid360 parent="…" xyz="…" rpy="…" sensor="raycast|gpu"/>`).
3. Bridge `/livox/points` and `/livox/imu_raw` (see `config/bridge.yaml`) and run
   `livox_driver_emulator` (`xfer_format:=1`).
4. FAST-LIO: `lidar_type: 1` (Livox), `lid_topic: /livox/lidar`, `imu_topic: /livox/imu`,
   `extrinsic_T: [-0.011, -0.02329, 0.04412]`, `extrinsic_R: identity`. The old
   `pointcloud2_to_livox` adapter is no longer needed, because `/livox/lidar` is already a
   CustomMsg with real per-point times and lines.

## Approach 1 vs approach 2

Both sensors pass `check_livox_cloud.py` headless in all three modes (default,
`xfer_format:=0`, bare model on a pole), and run with the Gazebo GUI and RViz under
`teleop_twist_keyboard`. Both produce the same output contract, return model and IMU.
The development history of each is on the branches `approach-1-gpu-pattern` and
`approach-2-exact-raycast`.

Measured on this machine: AMD Ryzen 5 2600 (6C/12T), GTX 1050 Ti 4 GB, Ubuntu 24.04, test
world, test rig, `range_noise:=0` for accuracy.

| | **1: dense GPU render + pattern sampling** (`sensor:=gpu`) | **2: exact ray per pattern direction** (`sensor:=raycast`) |
|---|---|---|
| Scan pattern | real CSV, 20 000 rays/frame | real CSV, 20 000 rays/frame |
| Timing / skew | each point from the render nearest its time (50 Hz → ±10 ms). Spinning at 1 rad/s: **~10 mm** median residual mid-frame (5–7 mm at 100 Hz, 130–150 mm at 10 Hz) | ray cast from the pose interpolated to its exact time: **0.00 mm** residual |
| Geometry hit | **visual** (whatever ogre2 renders: meshes, heightmaps, everything) | **collision** by default, or visual (`geometry_source`). Box/sphere/cylinder/capsule/ellipsoid/plane/mesh; heightmaps, polylines and cones are skipped |
| Range accuracy (static) | walls median 1.4 mm, p99 21 mm. Grazing ground (−5..−7°) median **51 mm**, p95 190 mm (depth cube-map quantization; `dense_h:=2880` halves it but RTF drops to 0.96). Edges: 2.3 % of object points > 20 cm off, 0.55 % missed and 0.33 % spurious returns | exact: 0.00 mm everywhere, 0 missed / 0 spurious (vs an independent analytic reference) |
| Noise | σ = 2 cm along the ray (same model) | same |
| Rays/s | 200 k used, out of 11.5 M rendered | 200 k cast. **0.33 µs/ray** single-threaded; 2 M rays/s still faster than real time |
| RTF (full launch) | 1.00 at 50 Hz, 0.92 at 100 Hz | 1.00 |
| CPU (gz server, 100 % = 1 core) | 212–219 % | 119 % |
| GPU | 17–28 % util, about +170 MiB; needs ogre2 + EGL/GPU even headless | none |
| Code (non-blank, non-comment lines, sensor only) | ~225 (system + dense sampler) | ~710 (system + ray caster) |
| Moving into ignc-kit | copy the package; the world needs `Sensors` (ogre2) + `Imu`; GPU required | copy the package; the world needs only `Imu`; no GPU, so it also runs on CI / in containers without EGL |

**Recommendation: approach 2 (exact ray casting), hence the default `sensor:=raycast`.** It is exact in geometry and in per-point
timing, so FAST-LIO sees ideal Mid-360 data with true motion skew. It needs no GPU, uses
about half the CPU of approach 1, and has headroom for several sensors or a 1 kHz world on
one core. It has no side effects on physics, and integration is the same `<include>` plus
joint. Its limits are what it hits:

* **Collision geometry**, not visuals. If ignc-kit's collisions are crude boxes, the cloud
  will be too. Switch to `geometry_source:=visual` where visual meshes matter.
* **No heightmaps.** If an ignc-kit world uses heightmap terrain, the terrain is invisible to
  approach 2. Either add a mesh collision for it or use approach 1 for that world.

Keep approach 1 for worlds that rely on rendered-only geometry. Run it at 50 Hz, or at 100 Hz
when sub-centimetre skew matters more than real time.
