# Mid-360 scan pattern

`mid360.csv` is the recorded Livox Mid-360 scan pattern from
<https://github.com/ggory15/livox_laser_simulation_ros2/blob/main/scan_mode/mid360.csv>
(a fork of <https://github.com/LCAS/livox_laser_simulation_ros2>), MIT licensed,
Copyright (c) 2023 Ricardo Casimiro, see `LICENSE`. The file is unmodified
(sha256 `aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`).

Format, as used by `livox_mid360_gazebo`:

* header `Time/s,Azimuth/deg,Zenith/deg`; 800 000 rows = 4 s at 200 000 pts/s.
  Despite its name, the first column is a running index; point *i* is emitted at
  `i / 200000 s`.
* Azimuth in degrees, counter-clockwise from +x; zenith measured from +z,
  so elevation = 90 - zenith. Elevation range -7.21 .. 52.16 deg.
* Rows come in groups of 4 lasers (stacked ~1.5-3 deg apart in elevation; the group
  rotates over the pattern so its azimuth spread is 0-8 deg), so `line = row % 4`.
* Successive 100 ms frames do not repeat (5 frames cover ~3.1x the 0.5 deg cells of one).
