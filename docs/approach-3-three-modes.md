# Approach 3: one plugin with three lidar modes (not on `main`)

Approach 3 was a single gz-sim system, `LivoxMid360System`, that followed the same recorded
Mid-360 scan pattern and measured the ranges in one of three ways (`<mode>`):

| mode | how ranges are measured | timing |
|---|---|---|
| `snapshot` | one dense `gpu_lidar` depth image per frame (10 Hz) | points stamped across the frame, but all measured at the image time (no real skew) |
| `sliced` (default) | `10 × slices` depth images per frame; each supplies the next `1/slices` of the rays | each slice stamped with its image time, so skew in steps of `100/slices` ms |
| `raycast` | CPU ray casting, no rendering: binned-SAH BVH over the tessellated **visual** geometry (box, sphere, cylinder, capsule, ellipsoid, cone, plane, mesh) | rays due each physics step cast from that step's sensor pose, each with its own time |

It was merged into `main` as `sensor:=modes lidar_mode:=… slices:=N` and passed
`check_livox_cloud.py` in every mode (test rig, Scout Mini, bare model, `xfer_format:=0`,
`slices:=10`) at RTF 1.00. It was then removed again because it overlaps the two sensors
that remain:

* `snapshot` is the same as `sensor:=gpu dense_rate:=10`.
* `sliced` is a coarser version of `sensor:=gpu` at 50 Hz, which picks the render nearest each
  point's own time and keeps per-point timestamps.
* `raycast` is a second CPU ray caster beside `sensor:=raycast` (approach 2). Approach 2 is
  exact on analytic primitives instead of tessellating them, interpolates the sensor pose to
  each ray's time, can hit collision or visual geometry, models reflectivity-dependent range,
  and has measured accuracy. Approach 3's one extra was hitting cones.

Keeping one GPU sensor (`sensor:=gpu`) and one ray-casting sensor (`sensor:=raycast`) leaves a
single plugin library and no duplicate ray caster or scan-pattern loader to maintain.

## Where the code is

* Commit `9906e28` (the former branch `approach-3-three-modes`, since deleted; the commit
  stays in `main`'s history as the second parent of `f676431`): the standalone version, with
  its own launch file, checker and pixi setup. `git checkout -b approach-3 9906e28` brings
  the branch back.
* Merge commit `f676431` on `main`: the version integrated as `sensor:=modes` (plugin in
  `src/livox_mid360_sim/src/modes/`, namespace `livox_mid360_sim::modes`,
  `model://livox_mid360_modes`). To look at it: `git show f676431 --stat`, or
  `git checkout f676431` to build and run it.
