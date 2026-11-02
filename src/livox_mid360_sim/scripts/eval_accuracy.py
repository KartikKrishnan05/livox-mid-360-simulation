#!/usr/bin/env python3
"""Compare simulated Mid-360 points against the world's analytic geometry.

The world SDF's static collision shapes (plane, box, cylinder, sphere) are ray-cast
exactly in numpy from the sensor's true pose (queried from gz). For every frame the
pattern block is identified from the point timestamps, so each of the 20 000 rays is
compared with ground truth:

  * range error of every returned point, grouped by the surface the true ray hits,
  * missed returns (truth hits within 0.1..40 m, sensor reported nothing),
  * spurious returns (sensor reported a point, truth hits nothing).

Run against a *static* sensor, ideally with range_noise:=0.0, e.g.
  ros2 launch livox_mid360_sim sim.launch.py gui:=false rviz:=false range_noise:=0.0
  ros2 run livox_mid360_sim eval_accuracy.py --frames 20

Motion skew (--moving): with the test rig moving (e.g. spinning in place), the truth for
each ray is computed from the sensor pose *at that ray's timestamp* (ground-truth pose from
gz, /ground_truth/pose, interpolated) and, for contrast, from one pose for the whole frame
(at timebase). Wheel odometry is not used: it drifts (slip) when spinning in place.
A sensor that models skew matches the per-ray truth; one that doesn't matches the
single-pose truth.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

LIVOX_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                        'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})


# ----------------------------------------------------------------------------- poses
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), \
        math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def parse_pose(el):
    if el is None or el.text is None:
        return np.eye(3), np.zeros(3)
    v = [float(x) for x in el.text.split()]
    return rpy_to_R(*v[3:6]), np.array(v[:3])


def compose(a, b):
    Ra, ta = a
    Rb, tb = b
    return Ra @ Rb, Ra @ tb + ta


# ----------------------------------------------------------------------------- shapes
class Shape:
    def __init__(self, name, kind, pose, size):
        self.name, self.kind, self.size = name, kind, size
        self.R, self.t = pose

    def intersect(self, o, d):
        """o: (3,) or (N,3), d: (N,3) unit. Returns t (N,) with inf for no hit (t > 0)."""
        ol = np.broadcast_to(o, d.shape) - self.t
        ol = ol @ self.R          # rows: R^T (o - t)
        dl = d @ self.R           # rows: R^T d
        n = len(d)
        inf = np.full(n, np.inf)
        with np.errstate(divide='ignore', invalid='ignore'):
            if self.kind == 'plane':
                nz = self.size  # local normal
                denom = dl @ nz
                t = -(ol @ nz) / denom
                return np.where((t > 1e-9) & np.isfinite(t), t, inf)
            if self.kind == 'box':
                h = self.size / 2
                t1 = (-h - ol) / dl
                t2 = (h - ol) / dl
                tmin = np.nanmax(np.minimum(t1, t2), axis=1)
                tmax = np.nanmin(np.maximum(t1, t2), axis=1)
                hit = (tmax >= tmin) & (tmax > 1e-9)
                t = np.where(tmin > 1e-9, tmin, tmax)
                return np.where(hit, t, inf)
            if self.kind == 'sphere':
                r = self.size
                b = np.sum(dl * ol, axis=1)
                c = np.sum(ol * ol, axis=1) - r * r
                disc = b * b - c
                sq = np.sqrt(np.maximum(disc, 0))
                t0, t1 = -b - sq, -b + sq
                t = np.where(t0 > 1e-9, t0, t1)
                return np.where((disc >= 0) & (t > 1e-9), t, inf)
            if self.kind == 'cylinder':
                r, L = self.size
                a = dl[:, 0] ** 2 + dl[:, 1] ** 2
                b = dl[:, 0] * ol[:, 0] + dl[:, 1] * ol[:, 1]
                c = ol[:, 0] ** 2 + ol[:, 1] ** 2 - r * r
                disc = b * b - a * c
                sq = np.sqrt(np.maximum(disc, 0))
                best = inf.copy()
                for tt in ((-b - sq) / a, (-b + sq) / a):
                    z = ol[:, 2] + tt * dl[:, 2]
                    ok = (disc >= 0) & (tt > 1e-9) & (np.abs(z) <= L / 2)
                    best = np.where(ok & (tt < best), tt, best)
                for zc in (-L / 2, L / 2):
                    tt = (zc - ol[:, 2]) / dl[:, 2]
                    x = ol[:, 0] + tt * dl[:, 0]
                    y = ol[:, 1] + tt * dl[:, 1]
                    ok = (tt > 1e-9) & (x * x + y * y <= r * r)
                    best = np.where(ok & (tt < best), tt, best)
                return best
        raise ValueError(self.kind)


def load_world(path):
    root = ET.parse(path).getroot()
    world = root.find('world')
    shapes = []
    for model in world.findall('model'):
        mp = parse_pose(model.find('pose'))
        for link in model.findall('link'):
            lp = compose(mp, parse_pose(link.find('pose')))
            for col in link.findall('collision'):
                cp = compose(lp, parse_pose(col.find('pose')))
                g = col.find('geometry')
                name = f"{model.get('name')}/{col.get('name')}"
                if g.find('plane') is not None:
                    nrm = np.array([float(x) for x in g.find('plane/normal').text.split()])
                    shapes.append(Shape(name, 'plane', cp, nrm / np.linalg.norm(nrm)))
                elif g.find('box') is not None:
                    sz = np.array([float(x) for x in g.find('box/size').text.split()])
                    shapes.append(Shape(name, 'box', cp, sz))
                elif g.find('sphere') is not None:
                    shapes.append(Shape(name, 'sphere', cp, float(g.find('sphere/radius').text)))
                elif g.find('cylinder') is not None:
                    shapes.append(Shape(name, 'cylinder', cp,
                                        (float(g.find('cylinder/radius').text),
                                         float(g.find('cylinder/length').text))))
    return shapes


def raycast(shapes, o, d):
    best = np.full(len(d), np.inf)
    idx = np.full(len(d), -1)
    for i, s in enumerate(shapes):
        t = s.intersect(o, d)
        m = t < best
        best[m] = t[m]
        idx[m] = i
    return best, idx


def group_of(name):
    if name.startswith('ground'):
        return 'ground'
    if name.startswith('wall'):
        return 'walls'
    if name.startswith('overhang/slab'):
        return 'overhang'
    return 'objects'


# ----------------------------------------------------------------------------- gz pose
def gz_pose(args_list):
    out = subprocess.run(['gz', 'model'] + args_list, capture_output=True, text=True,
                         timeout=20).stdout
    # "- Pose [...]" (not "- Inertial Pose [...]")
    m = re.findall(r'^\s*-?\s*Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]*)\]\s*'
                   r'\[([^\]]*)\]', out, re.M)
    if not m:
        raise RuntimeError(f'no pose in: gz model {" ".join(args_list)}\n{out}')
    xyz = [float(v) for v in m[0][0].split()]
    rpy = [float(v) for v in m[0][1].split()]
    return rpy_to_R(*rpy), np.array(xyz)


def load_pattern(path):
    d = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(1, 2))
    az = np.radians(d[:, 0])
    el = np.radians(90.0 - d[:, 1])
    dirs = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
    return dirs[: len(dirs) // 4 * 4]


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--frames', type=int, default=20)
    ap.add_argument('--model', default='test_rig')
    ap.add_argument('--link', default='livox_frame')
    ap.add_argument('--offset-z', type=float, default=0.0,
                    help='livox_frame above --link (0.047 for lidar_link)')
    ap.add_argument('--world', default=None, help='world SDF (default: livox_test.sdf)')
    ap.add_argument('--pattern', default=None, help='mid360.csv')
    ap.add_argument('--topic', default='/livox/points')
    ap.add_argument('--max-range', type=float, default=40.0,
                    help='truth hits beyond this are not counted as missed')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--moving', action='store_true',
                    help='moving rig: per-ray sensor pose from /ground_truth/pose')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import PointCloud2
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    share = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('livox_mid360_sim')
    except Exception:
        pass
    world = args.world or os.path.join(share, 'worlds', 'livox_test.sdf')
    pattern = args.pattern or os.path.join(share, 'models', 'livox_mid360', 'scan_mode',
                                           'mid360.csv')
    shapes = load_world(world)
    dirs_all = load_pattern(pattern)
    n_frames_pat = len(dirs_all) // 20000

    # Sensor pose in the world: model pose * link pose * (0,0,offset_z)
    Pm = gz_pose(['-m', args.model, '-p'])
    Pl = gz_pose(['-m', args.model, '-l', args.link])
    R, t = compose(Pm, Pl)
    t = t + R @ np.array([0, 0, args.offset_z])
    print(f'sensor at {np.round(t, 4)} (world), {len(shapes)} analytic shapes, '
          f'pattern {len(dirs_all)} rays')

    rclpy.init()
    node = rclpy.create_node('eval_accuracy')
    msgs = []
    gt = []
    node.create_subscription(PointCloud2, args.topic, lambda m: msgs.append(m),
                             QoSProfile(depth=50))
    if args.moving:
        def on_gt(m):
            p, q = m.pose.position, m.pose.orientation
            gt.append((m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                       p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        node.create_subscription(PoseStamped, '/ground_truth/pose', on_gt,
                                 QoSProfile(depth=2000))
    t0 = time.monotonic()
    while len(msgs) < args.frames + 1 and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    t1 = time.monotonic()
    while args.moving and time.monotonic() - t1 < 0.5:   # poses past the last frame
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    msgs = msgs[1:]  # skip a possibly partial first frame
    if not msgs:
        print('no frames received')
        return 1

    if args.moving:
        g = np.array(gt)
        g = g[np.argsort(g[:, 0])]
        qs = g[:, 4:8].copy()
        for i in range(1, len(qs)):             # consistent hemisphere for interpolation
            if np.dot(qs[i], qs[i - 1]) < 0:
                qs[i] = -qs[i]
        Pl_R, Pl_t = Pl

        def poses_at(ts):
            """World pose of livox_frame at times ts (ns) -> (N,3,3), (N,3)."""
            pos = np.stack([np.interp(ts, g[:, 0], g[:, j]) for j in (1, 2, 3)], axis=1)
            q = np.stack([np.interp(ts, g[:, 0], qs[:, j]) for j in range(4)], axis=1)
            q /= np.linalg.norm(q, axis=1, keepdims=True)
            w, x, y, z = q.T
            Rb = np.stack([
                np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], 1),
                np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], 1),
                np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], 1),
            ], axis=1)
            Rs = Rb @ Pl_R
            return Rs, pos + Rb @ Pl_t + Rs @ np.array([0, 0, args.offset_z])
        yaw = np.unwrap(np.arctan2(2 * (qs[:, 0] * qs[:, 3] + qs[:, 1] * qs[:, 2]),
                                   1 - 2 * (qs[:, 2] ** 2 + qs[:, 3] ** 2)))
        dt = np.diff(g[:, 0]) * 1e-9
        print(f'ground truth: {len(g)} poses, yaw rate median {np.median(np.diff(yaw) / dt):.3f}'
              f' rad/s, speed {np.median(np.hypot(np.diff(g[:, 1]), np.diff(g[:, 2])) / dt):.3f}'
              ' m/s')
        quart_ray = [[] for _ in range(4)]
        quart_frame = [[] for _ in range(4)]

    rows = {g: [] for g in ('ground', 'walls', 'overhang', 'objects')}
    vertical = np.array([group_of(sh.name) in ('walls', 'objects') for sh in shapes])
    missed = spurious = truth_hits = n_points = 0
    dir_err = []
    for m in msgs:
        a = np.frombuffer(bytes(m.data), dtype=LIVOX_DTYPE, count=m.width * m.height)
        xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        tb = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        k = np.rint((a['timestamp'] - tb) / 5000.0).astype(np.int64)  # index within frame
        rng = np.linalg.norm(xyz, axis=1)
        u = xyz / rng[:, None]
        # Which 20 000-ray block of the pattern is this frame?
        best_blk, best_err = None, 1e9
        for blk in range(n_frames_pat):
            e = np.median(np.linalg.norm(dirs_all[blk * 20000 + k[:200]] - u[:200], axis=1))
            if e < best_err:
                best_blk, best_err = blk, e
        D = dirs_all[best_blk * 20000: (best_blk + 1) * 20000]
        if args.moving:
            ts = tb + np.arange(20000) * 5000.0
            Rs, os_ = poses_at(ts)
            Dw = np.einsum('nij,nj->ni', Rs, D)
            tr, idx = raycast(shapes, os_, Dw)          # truth with per-ray pose
            tr0, _ = raycast(shapes, os_[0], D @ Rs[0].T)   # one pose for the frame
            # Skew is only visible on surfaces whose range changes with yaw: use walls and
            # vertical objects (the pattern points steeply up at the start/end of a frame).
            vert = vertical[np.maximum(idx[k], 0)] & (idx[k] >= 0)
            for q in range(4):
                sel = (k >= q * 5000) & (k < (q + 1) * 5000) & vert
                for store, truth in ((quart_ray, tr), (quart_frame, tr0)):
                    ok_ = np.isfinite(truth[k[sel]])
                    store[q].append(np.abs(rng[sel][ok_] - truth[k[sel]][ok_]))
        else:
            tr, idx = raycast(shapes, t, D @ R.T)      # truth for all 20k rays
        hit_truth = np.isfinite(tr) & (tr >= 0.1) & (tr <= args.max_range)
        got = np.zeros(20000, bool)
        got[k] = True
        truth_hits += int(hit_truth.sum())
        missed += int((hit_truth & ~got).sum())
        spurious += int((got & ~np.isfinite(tr)).sum())
        n_points += len(k)
        dir_err.append(np.degrees(np.arccos(np.clip(np.sum(D[k] * u, axis=1), -1, 1))))
        ok = np.isfinite(tr[k])
        err = rng[ok] - tr[k][ok]
        for e, i in zip(err, idx[k][ok]):
            rows[group_of(shapes[i].name)].append(e)

    dir_err = np.concatenate(dir_err)
    print(f'{len(msgs)} frames, {n_points} points, direction error vs pattern: '
          f'max {dir_err.max():.4f} deg')
    print(f'{"surface":<9} {"n":>7} {"bias mm":>8} {"std mm":>7} {"|e| p50":>8} '
          f'{"p95":>7} {"p99":>7} {">5cm %":>7} {">20cm %":>8}')
    allerr = []
    for g, v in rows.items():
        if not v:
            continue
        e = np.array(v)
        allerr.append(e)
        ae = np.abs(e)
        print(f'{g:<9} {len(e):>7} {1e3 * e.mean():>8.2f} {1e3 * e.std():>7.2f} '
              f'{1e3 * np.median(ae):>8.2f} {1e3 * np.percentile(ae, 95):>7.2f} '
              f'{1e3 * np.percentile(ae, 99):>7.2f} {100 * np.mean(ae > 0.05):>7.3f} '
              f'{100 * np.mean(ae > 0.2):>8.3f}')
    e = np.concatenate(allerr)
    ae = np.abs(e)
    print(f'{"all":<9} {len(e):>7} {1e3 * e.mean():>8.2f} {1e3 * e.std():>7.2f} '
          f'{1e3 * np.median(ae):>8.2f} {1e3 * np.percentile(ae, 95):>7.2f} '
          f'{1e3 * np.percentile(ae, 99):>7.2f} {100 * np.mean(ae > 0.05):>7.3f} '
          f'{100 * np.mean(ae > 0.2):>8.3f}')
    print(f'missed returns: {missed}/{truth_hits} ({100 * missed / max(1, truth_hits):.3f}%), '
          f'spurious returns: {spurious}/{n_points} ({100 * spurious / max(1, n_points):.3f}%)')
    if args.moving:
        print('motion skew: median |range error| [mm] on walls/vertical objects, by offset_time')
        print(f'{"truth model":<28} {"0-25ms":>8} {"25-50":>8} {"50-75":>8} {"75-100":>8}')
        for name, store in (('per-ray pose (skew model)', quart_ray),
                            ('one pose @ timebase', quart_frame)):
            vals = [1e3 * np.median(np.concatenate(store[q])) for q in range(4)]
            print(f'{name:<28} ' + ' '.join(f'{v:>8.2f}' for v in vals))
    return 0


if __name__ == '__main__':
    sys.exit(main())
