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
| `dense_rate` | `10` | render rate of the dense gpu_lidar (approach 1) |

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

## How the sensor works (this branch: `main`)

The model carries a dense `gpu_lidar`: 1440 × 160 rays over 360° × −7.5..52.5°, i.e. 0.25° ×
0.377°, rendered by ogre2 on the GPU. The `livox_mid360_sim::LivoxMid360` system subscribes to
its `gz.msgs.LaserScan`. For every 100 ms frame it samples the next 20 000 pattern directions
from the render:

* **bilinear** in 1/r when the four neighbouring pixels lie on one continuous surface
  (spread ≤ 10 %), which is exact to first order for planes;
* **nearest neighbour** at depth edges and hit/miss borders, so no "ghost" points float
  between foreground and background.

The return model then applies the reflectivity-dependent detection range, the range noise and
`min_range`.

**LaserScan layout (verified):** `ranges[v * count + h]`, with `h = 0` at `angle_min`
(counter-clockwise) and `v = 0` at `vertical_angle_min`. The proof uses the ground plane:
with this indexing, points below −5.5° land at median z = −0.546 m for a sensor at 0.547 m
(bare model on its pole) and at −0.716 m for the test rig, whose sensor gz reports at exactly
0.722 m. A flipped `v` would put those points high above the sensor (the `ground` check in
`check_livox_cloud.py`). The residual few mm come from depth-render quantization at grazing
incidence (see Accuracy).

On `main`, each frame comes from **one** render at the end of the frame, so all 20 000 points
see the scene as it was at one instant. Timestamps are still spread over 100 ms, but motion
skew is not modelled. The `approach-1-gpu-pattern` branch renders at 50–100 Hz and gives each
point the render nearest its own timestamp. The `approach-2-exact-raycast` branch casts one
exact ray per pattern direction. See the comparison at the end.

## Accuracy against analytic geometry

`eval_accuracy.py` ray-casts the world's collision shapes exactly from the true sensor pose,
which it queries from gz. It finds the pattern block of each frame from the timestamps and
compares all 20 000 rays per frame with ground truth:

```bash
ros2 launch livox_mid360_sim sim.launch.py gui:=false rviz:=false range_noise:=0.0
ros2 run livox_mid360_sim eval_accuracy.py --frames 20
```

It reports the range error per surface (ground, walls, objects), missed returns (truth hits
within 40 m but no point) and spurious returns (a point where truth hits nothing).

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
   The model origin (`lidar_link`) is the centre of the mounting face. The world needs the
   `Sensors` system (ogre2) and the `Imu` system. In URDF, use the
   `urdf/livox_mid360.urdf.xacro` macro instead
   (`<xacro:livox_mid360 parent="…" xyz="…" rpy="…"/>`).
3. Bridge `/livox/points` and `/livox/imu_raw` (see `config/bridge.yaml`) and run
   `livox_driver_emulator` (`xfer_format:=1`).
4. FAST-LIO: `lidar_type: 1` (Livox), `lid_topic: /livox/lidar`, `imu_topic: /livox/imu`,
   `extrinsic_T: [-0.011, -0.02329, 0.04412]`, `extrinsic_R: identity`. The old
   `pointcloud2_to_livox` adapter is no longer needed, because `/livox/lidar` is already a
   CustomMsg with real per-point times and lines.
