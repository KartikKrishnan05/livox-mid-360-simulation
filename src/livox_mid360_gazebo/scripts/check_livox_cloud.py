#!/usr/bin/env python3
"""Check that the simulated Mid-360 behaves like livox_ros_driver2 on a real sensor.

ros2 run livox_mid360_gazebo check_livox_cloud.py [--frames 20] [--timeout 90]
    [--ground-z -0.547] [--min-points 2000]

Exits non-zero if any check fails. The sensor should be standing still (IMU |a| ~ 1 g).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # pragma: no cover
    CustomMsg = None

LIVOX_FIELDS = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                ('tag', 16, 2), ('line', 17, 2), ('timestamp', 18, 8)]
LIVOX_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                        'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})


def stamp_ns(st):
    return st.sec * 1_000_000_000 + st.nanosec


class Frame:
    """One lidar frame as numpy arrays (x, y, z, refl, tag, line, t_ns absolute)."""

    def __init__(self, header_ns, frame_id, timebase, xyz, refl, tag, line, t_abs):
        self.header_ns = header_ns
        self.frame_id = frame_id
        self.timebase = timebase
        self.xyz = xyz
        self.refl = refl
        self.tag = tag
        self.line = line
        self.t_abs = t_abs


def frame_from_custom(m):
    n = len(m.points)
    xyz = np.empty((n, 3), np.float64)
    off = np.empty(n, np.int64)
    refl = np.empty(n, np.int64)
    tag = np.empty(n, np.int64)
    line = np.empty(n, np.int64)
    for i, p in enumerate(m.points):
        xyz[i, 0] = p.x
        xyz[i, 1] = p.y
        xyz[i, 2] = p.z
        off[i] = p.offset_time
        refl[i] = p.reflectivity
        tag[i] = p.tag
        line[i] = p.line
    f = Frame(stamp_ns(m.header.stamp), m.header.frame_id, m.timebase, xyz, refl, tag, line,
              m.timebase + off)
    f.point_num = m.point_num
    return f


def frame_from_pc2(m):
    a = np.frombuffer(bytes(m.data), dtype=LIVOX_DTYPE, count=m.width * m.height)
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
    t = a['timestamp'].astype(np.int64)
    hdr = stamp_ns(m.header.stamp)
    f = Frame(hdr, m.header.frame_id, hdr, xyz, a['intensity'].astype(np.int64),
              a['tag'].astype(np.int64), a['line'].astype(np.int64), t)
    f.point_num = len(a)
    return f


def pc2_layout_ok(m):
    got = sorted((f.name, f.offset, f.datatype) for f in m.fields)
    return m.point_step == 26 and got == sorted(LIVOX_FIELDS), \
        f'point_step={m.point_step} fields={[(f.name, f.offset, f.datatype) for f in m.fields]}'


class Checker(Node):
    def __init__(self, args):
        super().__init__('check_livox_cloud')
        self.args = args
        self.frames = []
        self.frame_recv_clock = []
        self.points_msgs = []
        self.imu = []
        self.clock_ns = None
        self.lidar_type = None
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        # Deep reliable queues: converting a 20k-point CustomMsg in Python takes a while.
        self.deep = QoSProfile(depth=2000)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, self.deep)
        self.create_subscription(PointCloud2, '/livox/points', self.on_points,
                                 qos_profile_sensor_data)

    def detect_lidar_type(self):
        for name, types in self.get_topic_names_and_types():
            if name == '/livox/lidar':
                if 'livox_ros_driver2/msg/CustomMsg' in types:
                    if CustomMsg is None:
                        raise RuntimeError('livox_ros_driver2 python messages not found')
                    self.lidar_type = 'CustomMsg'
                    self.create_subscription(CustomMsg, '/livox/lidar', self.on_custom,
                                             QoSProfile(depth=50))
                elif 'sensor_msgs/msg/PointCloud2' in types:
                    self.lidar_type = 'PointCloud2'
                    self.create_subscription(PointCloud2, '/livox/lidar', self.on_lidar_pc2,
                                             QoSProfile(depth=50))
                return True
        return False

    def on_clock(self, m):
        self.clock_ns = stamp_ns(m.clock)

    def on_imu(self, m):
        a = m.linear_acceleration
        self.imu.append((stamp_ns(m.header.stamp), m.header.frame_id,
                         math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))

    def on_points(self, m):
        if len(self.points_msgs) < 3:
            self.points_msgs.append(m)

    def _add(self, f, raw=None):
        if len(self.frames) < self.args.frames:
            self.frames.append(f)
            self.frame_recv_clock.append(self.clock_ns)
            if raw is not None:
                self.lidar_pc2_raw = raw

    def on_custom(self, m):
        self._add(frame_from_custom(m))

    def on_lidar_pc2(self, m):
        self._add(frame_from_pc2(m), m)

    def done(self):
        if not self.imu:
            return False
        imu_span = (self.imu[-1][0] - self.imu[0][0]) * 1e-9
        return len(self.frames) >= self.args.frames and imu_span >= 2.0


class Report:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail):
        self.rows.append((name, bool(ok), detail))

    def print(self):
        w = max(len(r[0]) for r in self.rows)
        for name, ok, detail in self.rows:
            print(f'  [{"PASS" if ok else "FAIL"}] {name:<{w}}  {detail}')
        return all(r[1] for r in self.rows)


def cells(f, res_deg=0.5):
    x, y, z = f.xyz[:, 0], f.xyz[:, 1], f.xyz[:, 2]
    az = np.degrees(np.arctan2(y, x)) % 360.0
    el = np.degrees(np.arctan2(z, np.hypot(x, y)))
    return set(zip((az // res_deg).astype(int).tolist(), (el // res_deg).astype(int).tolist()))


def evaluate(node, args):
    rep = Report()
    fr = node.frames
    # --- lidar rate (sim time) ---
    hdr = np.array([f.header_ns for f in fr], np.int64)
    rate = (len(hdr) - 1) / ((hdr[-1] - hdr[0]) * 1e-9) if len(hdr) > 1 else 0.0
    clk = [c for c in node.frame_recv_clock if c is not None]
    rx_rate = (len(clk) - 1) / ((clk[-1] - clk[0]) * 1e-9) if len(clk) > 1 and clk[-1] > clk[0] \
        else float('nan')
    rep.add('lidar rate (sim time)', 9.0 <= rate <= 11.0 and not (rx_rate < 8.0),
            f'{rate:.2f} Hz from stamps, {rx_rate:.2f} Hz received per sim second')
    rep.add('lidar type', node.lidar_type is not None, f'/livox/lidar is {node.lidar_type}')

    npts = np.array([len(f.xyz) for f in fr])
    pn_ok = all(f.point_num == len(f.xyz) for f in fr)
    rep.add('points/frame', npts.min() >= args.min_points and npts.max() <= 20000 and pn_ok,
            f'min {npts.min()} mean {npts.mean():.0f} max {npts.max()} (<= 20000 rays/frame),'
            f' point_num {"ok" if pn_ok else "MISMATCH"}')
    fids = {f.frame_id for f in fr}
    rep.add('frame_id', fids == {'livox_frame'}, f'{sorted(fids)}')

    hdr_tb = all(f.header_ns == f.timebase for f in fr)
    first = all(len(f.t_abs) == 0 or f.t_abs[0] == f.timebase for f in fr)
    rep.add('header.stamp == timebase == t(first point)', hdr_tb and first,
            f'header==timebase: {hdr_tb}, first point at timebase: {first}')

    allxyz = np.concatenate([f.xyz for f in fr])
    norms = np.linalg.norm(allxyz, axis=1)
    rep.add('no (0,0,0) / non-finite points',
            np.isfinite(allxyz).all() and (norms > 1e-6).all(),
            f'min range {norms.min():.3f} m, max {norms.max():.2f} m')

    el = np.degrees(np.arctan2(allxyz[:, 2], np.hypot(allxyz[:, 0], allxyz[:, 1])))
    rep.add('elevation within -7.5..52.5', el.min() >= -7.5 and el.max() <= 52.5,
            f'{el.min():.2f} .. {el.max():.2f} deg')

    sectors = []
    for f in fr:
        az = np.degrees(np.arctan2(f.xyz[:, 1], f.xyz[:, 0])) % 360.0
        sectors.append(len(np.unique((az // 10).astype(int))))
    rep.add('azimuth coverage >= 30/36 sectors per frame', min(sectors) >= 30,
            f'min {min(sectors)}/36')

    lines = sorted(set(np.concatenate([f.line for f in fr]).tolist()))
    rep.add('lines == [0,1,2,3]', lines == [0, 1, 2, 3], f'{lines}')
    tags = sorted(set(np.concatenate([f.tag for f in fr]).tolist()))
    rep.add('tag == 0', tags == [0], f'{tags}')
    refl = np.concatenate([f.refl for f in fr])
    rep.add('reflectivity 0..255', refl.min() >= 0 and refl.max() <= 255,
            f'{refl.min()} .. {refl.max()}')

    off_ok, span_min, mono = True, 1e9, True
    for f in fr:
        off = f.t_abs - f.timebase
        if len(off) == 0:
            continue
        off_ok &= off.min() >= 0 and off.max() <= 100_000_000
        span_min = min(span_min, (off.max() - off.min()) * 1e-6)
        mono &= bool((np.diff(off) >= 0).all())
    rep.add('offset_time in 0..100 ms, span > 90 ms, non-decreasing',
            off_ok and span_min > 90.0 and mono,
            f'in range: {off_ok}, min span {span_min:.2f} ms, non-decreasing: {mono}')

    if len(fr) >= 6:
        c1 = cells(fr[1])
        c5 = set().union(*[cells(f) for f in fr[1:6]])
        ratio = len(c5) / max(1, len(c1))
        rep.add('non-repetitive (5 frames > 1.5x cells of 1)', ratio > 1.5,
                f'{len(c1)} -> {len(c5)} cells of 0.5 deg ({ratio:.2f}x)')

    # --- internal gz cloud layout ---
    if node.points_msgs:
        ok, detail = pc2_layout_ok(node.points_msgs[-1])
        rep.add('/livox/points PointCloud2 layout (26 B)', ok, detail)
    else:
        rep.add('/livox/points PointCloud2 layout (26 B)', False, 'no message')
    if node.lidar_type == 'PointCloud2':
        ok, detail = pc2_layout_ok(node.lidar_pc2_raw)
        rep.add('/livox/lidar PointCloud2 layout (26 B)', ok, detail)

    # --- ground plane: proves the LaserScan layout (v=0 at vertical_angle_min) ---
    if args.ground_z is not None:
        f = fr[-1]
        e = np.degrees(np.arctan2(f.xyz[:, 2], np.hypot(f.xyz[:, 0], f.xyz[:, 1])))
        below = f.xyz[e < -5.5]
        if len(below):
            dz = below[:, 2] - args.ground_z
            frac = float(np.mean(np.abs(dz) < 0.08))
            rep.add('ground points (el < -5.5) at z ~ ground_z',
                    frac > 0.7 and abs(np.median(dz)) < 0.03,
                    f'{len(below)} pts, median z {np.median(below[:, 2]):+.3f} '
                    f'(expected {args.ground_z:+.3f}), {100 * frac:.0f}% within 8 cm')
        else:
            rep.add('ground points (el < -5.5) at z ~ ground_z', False, 'no points below -5.5 deg')

    # --- IMU ---
    imu = node.imu
    t = np.array([s for s, _, _ in imu], np.int64)
    span = (t[-1] - t[0]) * 1e-9 if len(t) > 1 else 0.0
    dt = np.diff(t) * 1e-9
    imu_rate = 1.0 / np.median(dt) if len(dt) else 0.0
    received = (len(t) - 1) / (span * imu_rate) if span > 0 and imu_rate > 0 else 0.0
    rep.add('IMU rate (sim time)', 190 <= imu_rate <= 210 and received > 0.9,
            f'{imu_rate:.1f} Hz (median stamp spacing), {100 * received:.0f}% of msgs '
            f'received over {span:.2f} s')
    acc = np.array([a for _, _, a in imu])
    rep.add('IMU |a| ~ 1 g (at rest)', 0.95 <= acc.mean() <= 1.05,
            f'mean {acc.mean():.4f} g, std {acc.std():.4f}')
    ifr = sorted({fid for _, fid, _ in imu})
    rep.add('IMU frame_id', ifr == ['livox_frame'], f'{ifr}')
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--frames', type=int, default=20)
    ap.add_argument('--timeout', type=float, default=90.0, help='wall-clock seconds')
    ap.add_argument('--min-points', type=int, default=2000)
    ap.add_argument('--ground-z', type=float, default=None,
                    help='expected ground z in livox_frame (e.g. -0.547 on the stand)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Checker(args)
    t0 = time.monotonic()
    while rclpy.ok() and node.lidar_type is None and time.monotonic() - t0 < args.timeout:
        node.detect_lidar_type()
        rclpy.spin_once(node, timeout_sec=0.2)
    while rclpy.ok() and not node.done() and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'check_livox_cloud: {len(node.frames)} lidar frames, {len(node.imu)} IMU msgs')
    if len(node.frames) < 2 or len(node.imu) < 2:
        print('  [FAIL] not enough data (is the simulation running?)')
        rclpy.shutdown()
        return 1
    ok = evaluate(node, args).print()
    print('RESULT:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
