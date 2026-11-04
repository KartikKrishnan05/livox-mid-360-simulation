#!/usr/bin/env python3
# Copyright 2026 Kartik Krishnan
# SPDX-License-Identifier: Apache-2.0
"""Sanity-check the simulated Livox Mid-360 against the livox_ros_driver2 contract.

Listens for a few seconds and verifies:
  /livox/lidar   livox_ros_driver2/CustomMsg: ~10 Hz, ~20k points/frame, FOV
                 360 x (-7..52) deg, lines 0-3, offset_time spanning the frame,
                 header stamp == timebase, non-repetitive coverage
  /livox/points  PointCloud2 in the driver's 26-byte layout
  /livox/imu     sensor_msgs/Imu, ~200 Hz, acceleration in g, frame livox_frame
Exits non-zero if a check fails.
"""

import argparse
import sys
import time

from livox_ros_driver2.msg import CustomMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable like the driver / bridge publishers, so rates are not skewed by
# best-effort drops when the machine is busy.
QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
from sensor_msgs.msg import Imu, PointCloud2

EXPECTED_FIELDS = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp']


class Checker(Node):

    def __init__(self, args):
        super().__init__('livox_mid360_check')
        self.custom, self.clouds, self.imu = [], [], []
        self.create_subscription(
            CustomMsg, args.lidar_topic, self.custom.append, QOS)
        self.create_subscription(
            PointCloud2, args.points_topic, self.clouds.append, QOS)
        self.create_subscription(Imu, args.imu_topic, self.imu.append, QOS)


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def custom_arrays(msg):
    pts = msg.points
    xyz = np.array([(p.x, p.y, p.z) for p in pts], dtype=np.float64).reshape(-1, 3)
    line = np.array([p.line for p in pts], dtype=np.int64)
    offset = np.array([p.offset_time for p in pts], dtype=np.int64)
    return xyz, line, offset


def cells(xyz, res_deg=0.5):
    r = np.linalg.norm(xyz, axis=1)
    a = np.floor(np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) / res_deg)
    e = np.floor(np.degrees(np.arcsin(xyz[:, 2] / np.maximum(r, 1e-9))) / res_deg)
    return set(zip(a.astype(int), e.astype(int)))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--points-topic', default='/livox/points')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--duration', type=float, default=6.0, help='wall-clock seconds')
    args = parser.parse_args()

    rclpy.init()
    node = Checker(args)
    wall_start = time.time()
    while time.time() < wall_start + args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.time() - wall_start

    ok = True

    def check(cond, text):
        nonlocal ok
        print(('  [ OK ] ' if cond else '  [FAIL] ') + text)
        ok &= bool(cond)

    print(f'CustomMsg {args.lidar_topic}: {len(node.custom)} frames')
    check(len(node.custom) >= 5, 'received at least 5 CustomMsg frames')
    if len(node.custom) >= 2:
        sim_span = stamp(node.custom[-1]) - stamp(node.custom[0])
        rate = (len(node.custom) - 1) / sim_span if sim_span > 0 else 0.0
        check(9.0 < rate < 11.0, f'frame rate {rate:.2f} Hz (sim time)')
        print(f'  real-time factor seen by ROS: {sim_span / wall:.2f} '
              f'({len(node.custom) / wall:.1f} frames per wall second)')
    if node.custom:
        msg = node.custom[-1]
        xyz, line, offset = custom_arrays(msg)
        counts = [m.point_num for m in node.custom]
        print(f'  points/frame: min {min(counts)}, median {int(np.median(counts))}, '
              f'max {max(counts)} (20000 rays/frame minus no-returns)')
        check(msg.point_num == len(msg.points), 'point_num == len(points)')
        check(len(xyz) > 5000, f'{len(xyz)} points in last frame')
        check(msg.header.frame_id == 'livox_frame', f'frame_id "{msg.header.frame_id}"')
        header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        check(header_ns == msg.timebase, 'header stamp == timebase')
        rng = np.linalg.norm(xyz, axis=1)
        check(np.all(rng > 0.05), 'no (0,0,0) points for missed rays')
        el = np.degrees(np.arcsin(np.clip(xyz[:, 2] / np.maximum(rng, 1e-9), -1, 1)))
        az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        check(el.min() >= -7.5 and el.max() <= 52.5,
              f'elevation {el.min():.1f} .. {el.max():.1f} deg (spec -7 .. 52)')
        sectors = (np.histogram(az, bins=36, range=(-180, 180))[0] > 0).sum()
        check(sectors >= 30, f'azimuth coverage {sectors}/36 sectors of 10 deg')
        check(rng.max() <= 70.0, f'range {rng.min():.2f} .. {rng.max():.2f} m')
        lines = sorted(set(line.tolist()))
        check(lines == [0, 1, 2, 3], f'lines {lines}')
        check(offset.min() >= 0 and offset.max() < 100_000_000,
              f'offset_time {offset.min() / 1e6:.2f} .. {offset.max() / 1e6:.2f} ms')
        # snapshot / raycast: up to ~100 ms; sliced: the last slice's start (80 ms for 5)
        check(offset.max() >= 50_000_000, 'offset_time spans most of the 100 ms frame')
        print(f'  distinct point times in the frame: {len(np.unique(offset))} '
              '(snapshot / raycast: one per point, sliced: one per slice)')
        check(np.all(np.diff(offset) >= 0), 'offset_time non-decreasing')
        if len(node.custom) >= 5:
            single = len(cells(xyz))
            union = len(set().union(*(cells(custom_arrays(m)[0]) for m in node.custom[-5:])))
            check(union > 1.5 * single,
                  f'non-repetitive: 5 frames hit {union} cells of 0.5 deg vs {single} for one')

    print(f'PointCloud2 {args.points_topic}: {len(node.clouds)} frames')
    if node.clouds:
        msg = node.clouds[-1]
        names = [f.name for f in msg.fields]
        check(names == EXPECTED_FIELDS, f'fields {names}')
        check(msg.point_step == 26, f'point_step {msg.point_step} (livox_ros_driver2: 26)')

    print(f'IMU {args.imu_topic}: {len(node.imu)} messages')
    check(len(node.imu) > 20, 'received IMU messages')
    if len(node.imu) > 1:
        sim_span = stamp(node.imu[-1]) - stamp(node.imu[0])
        rate = (len(node.imu) - 1) / sim_span if sim_span > 0 else 0.0
        check(180 < rate < 220, f'IMU rate {rate:.0f} Hz (sim time, Mid-360: 200 Hz)')
        a = node.imu[-1].linear_acceleration
        g = float(np.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2))
        check(0.9 < g < 1.1, f'|accel| {g:.3f} g at rest (driver reports g, not m/s^2)')
        check(node.imu[-1].header.frame_id == 'livox_frame',
              f'IMU frame_id "{node.imu[-1].header.frame_id}"')

    node.destroy_node()
    rclpy.shutdown()
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
