# Mid-360 scan pattern

`mid360.csv` is the Livox Mid-360 non-repetitive scan pattern from
[ggory15/livox_laser_simulation_ros2](https://github.com/ggory15/livox_laser_simulation_ros2)
(`scan_mode/mid360.csv`, fork of LCAS/livox_laser_simulation_ros2), MIT licensed, see `LICENSE`.

Format: header `Time/s,Azimuth/deg,Zenith/deg`, then 800 000 rows (4 s at 200 000 pts/s).
The first column is just a row index. Zenith is measured from +z, so elevation = 90 - zenith.
Rows come in groups of four stacked lasers about 3 deg apart; `line = row % 4`.
