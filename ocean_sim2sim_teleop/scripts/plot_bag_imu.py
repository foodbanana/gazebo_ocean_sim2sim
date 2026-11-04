#!/usr/bin/env python3
# =============================================================
# plot_bag_imu.py - look at the boat IMU (/base/imu) and robot base motion (/base/odom) of a recorded bag (docs/teleop.md)
#
#   ros2 run ocean_sim2sim_teleop plot_bag_imu.py <bag dir> [--out file.png] [--show] [--csv]
#   Figure (time = message header stamp, sim s from the first IMU sample):
#     1. linear acceleration x / y / z [m/s^2] (specific force, includes gravity on z)
#     2. angular velocity x / y / z [deg/s]
#     3. roll / pitch from the IMU orientation [deg]
#     4. robot base position from /base/odom x / y / z [m] (odom frame = robot base at the episode start)
#     drone motors off times of episode.json drawn as vertical lines.
#   Printed: rate, mean / std / min / max per axis. --csv writes base_imu.csv (+ base_odom.csv) next to the figure.
#   Output default: <bag dir>/base_imu.png
# =============================================================
import argparse
import csv
import json
import math
import os
import sys

import numpy as np


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    meta = os.path.join(path, 'metadata.yaml')
    if not os.path.isfile(meta):
        sys.exit(f'[ERROR] not a finished bag (no metadata.yaml): {path}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    cls = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        name, data, _ = reader.read_next()
        out[name].append(deserialize_message(data, cls[name]))
    return out, missing


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def roll_pitch(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def stats(name, unit, a):
    for i, ax in enumerate('xyz'[:a.shape[1]]):
        v = a[:, i]
        print(f'  {name} {ax}: mean {v.mean():+.4f}  std {v.std():.4f}  min {v.min():+.4f}  max {v.max():+.4f} {unit}')


def main():
    ap = argparse.ArgumentParser(description='plot /base/imu and /base/odom of a bag')
    ap.add_argument('bag')
    ap.add_argument('--out', default=None, help='figure path (default <bag>/base_imu.png)')
    ap.add_argument('--show', action='store_true', help='open a window as well')
    ap.add_argument('--csv', action='store_true', help='also write base_imu.csv / base_odom.csv next to the figure')
    a = ap.parse_args()
    bag = os.path.abspath(os.path.expanduser(a.bag))
    msgs, missing = read_bag(bag, ['/base/imu', '/base/odom'])
    if missing:
        print(f'[WARN] not in the bag: {missing}')
    imu, odom = msgs['/base/imu'], msgs['/base/odom']
    if not imu:
        sys.exit('[ERROR] no /base/imu messages in the bag')
    t_imu = np.array([stamp(m) for m in imu])
    t0 = t_imu[0]
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    gyr = np.degrees(np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu]))
    rp = np.array([roll_pitch((m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)) for m in imu])
    dt = np.diff(t_imu)
    # rate = samples / time (Gazebo sends sensors on physics steps only: at 240 Hz physics a 100 Hz IMU comes every
    # 8.33 or 12.5 ms, so the median interval is not the rate)
    ivals, counts = np.unique(np.round(dt * 1000, 2), return_counts=True)
    top = ', '.join(f'{v:g} ms x{c}' for v, c in sorted(zip(ivals, counts), key=lambda x: -x[1])[:3])
    print(f'{bag}\n/base/imu: {len(imu)} messages, {t_imu[-1] - t0:.2f} s, rate {(len(imu) - 1) / (t_imu[-1] - t0):.1f} Hz '
          f'(intervals {top}; max {dt.max() * 1000:.2f} ms), frame {imu[0].header.frame_id}')
    stats('accel', 'm/s^2', acc)
    stats('gyro ', 'deg/s', gyr)
    stats('roll/pitch', 'deg', rp)
    pos = t_odom = None
    if odom:
        t_odom = np.array([stamp(m) for m in odom]) - t0
        pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in odom])
        print(f'/base/odom: {len(odom)} messages, rate {(len(odom) - 1) / (t_odom[-1] - t_odom[0]):.1f} Hz, '
              f'frame {odom[0].header.frame_id}')
        stats('base position', 'm', pos)
    kills = []
    ep = os.path.join(bag, 'episode.json')
    if os.path.isfile(ep):
        with open(ep) as f:
            kills = [k.get('sim_time') for k in json.load(f).get('drone_kill', []) if k.get('sim_time') is not None]

    import matplotlib
    if not a.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = t_imu - t0
    rows = 4 if pos is not None else 3
    fig, ax = plt.subplots(rows, 1, sharex=True, figsize=(11, 2.6 * rows))
    for k, lab in enumerate('xyz'):
        ax[0].plot(t, acc[:, k], lw=0.7, label=lab)
        ax[1].plot(t, gyr[:, k], lw=0.7, label=lab)
        if pos is not None:
            ax[3].plot(t_odom, pos[:, k], lw=0.9, label=lab)
    ax[2].plot(t, rp[:, 0], lw=0.8, label='roll')
    ax[2].plot(t, rp[:, 1], lw=0.8, label='pitch')
    for axis, yl in zip(ax, ['linear acc [m/s^2]', 'angular vel [deg/s]', 'roll / pitch [deg]', 'base pos (odom) [m]']):
        axis.set_ylabel(yl)
        axis.grid(alpha=0.3)
        axis.legend(loc='upper right', fontsize=8, ncol=3)
        for tk in kills:
            axis.axvline(tk - t0, color='k', ls='--', lw=0.8)
    ax[-1].set_xlabel('time [s sim]' + ('   (dashed = drone motors off)' if kills else ''))
    fig.suptitle(os.path.basename(bag) + '  /base/imu' + (' + /base/odom' if pos is not None else ''))
    fig.tight_layout()
    out = os.path.abspath(os.path.expanduser(a.out)) if a.out else os.path.join(bag, 'base_imu.png')
    fig.savefig(out, dpi=110)
    print(f'figure: {out}')
    if a.csv:
        d = os.path.dirname(out)
        with open(os.path.join(d, 'base_imu.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'ax', 'ay', 'az', 'gx_dps', 'gy_dps', 'gz_dps', 'roll_deg', 'pitch_deg'])
            w.writerows(np.column_stack([t, acc, gyr, rp]).round(6).tolist())
        if pos is not None:
            with open(os.path.join(d, 'base_odom.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'z'])
                w.writerows(np.column_stack([t_odom, pos]).round(6).tolist())
        print(f'csv: {d}/base_imu.csv' + (', base_odom.csv' if pos is not None else ''))
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()
