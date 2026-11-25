"""Bag (mcap) of one recording run -> 100 Hz table on an exact sim time grid (+ E1 timing check).

Grid: t = k * 0.01 s sim time (integer nanoseconds, no drift), from the first to the last time covered by both the
ground truth and the IMU. Per column:
  ground truth (A2 - A5, 240 Hz, every physics step): position / velocities linear interpolation, orientation slerp
  IMU (A6 - A8, 100 Hz on the 240 Hz physics grid), joints (B5), commands (B8, B9), gripper (B10): the latest sample at
    or before t (zero-order hold, the spec's rule for slower streams); its age is kept (imu_age_s, joint_cmd_age_s)
  valid = the ground truth has no gap around t (a gap = more than 1.5 physics steps between samples) and the IMU
    sample is at most 2 IMU periods old. Rows are kept either way; the receiving side cuts invalid stretches.
Files: data_100hz.npz (arrays), data_100hz.csv (same, one header line with the spec item numbers), e1_timing.json.
"""
import csv
import json
import os

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp
import rosbag2_py

GRID_NS = 10_000_000                      # 100 Hz


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in types.items()}
    out = {n: [] for n in types}
    while reader.has_next():
        topic, data, t_bag = reader.read_next()
        out[topic].append((t_bag, deserialize_message(data, cls[topic])))
    return out, types


def _arr(rows, fn):
    return np.array([fn(m) for _, m in rows], dtype=float) if rows else np.zeros((0,))


def intervals(ns):
    """Interval statistics of a stamp series [ns]."""
    if len(ns) < 3:
        return {'n': int(len(ns))}
    d = np.diff(ns)
    med = float(np.median(d))
    vals, counts = np.unique(np.round(d / 1e6, 3), return_counts=True)
    order = np.argsort(-counts)[:6]
    return {'n': int(len(ns)), 'span_s': round((ns[-1] - ns[0]) * 1e-9, 4),
            'rate_hz': round((len(ns) - 1) / ((ns[-1] - ns[0]) * 1e-9), 4),
            'interval_ms_median': round(med / 1e6, 4), 'interval_ms_min': round(float(d.min()) / 1e6, 4),
            'interval_ms_max': round(float(d.max()) / 1e6, 4),
            'interval_ms_counts': {f'{vals[i]:g}': int(counts[i]) for i in order},
            'gaps_over_1.5x_median': int(np.sum(d > 1.5 * med)),
            'non_increasing': int(np.sum(d <= 0))}


def zoh(stamps, values, grid):
    """Latest sample at or before each grid time: (values, age_s); NaN before the first sample."""
    idx = np.searchsorted(stamps, grid, side='right') - 1
    ok = idx >= 0
    shape = (len(grid),) + values.shape[1:]
    v = np.full(shape, np.nan)
    v[ok] = values[idx[ok]]
    age = np.full(len(grid), np.nan)
    age[ok] = (grid[ok] - stamps[idx[ok]]) * 1e-9
    return v, age


def convert(run_dir, joint_names=None, log=print):
    bag_dir = os.path.join(run_dir, 'bag')
    msgs, types = read_bag(bag_dir)
    gt = msgs.get('/sim/boat_ground_truth', [])
    imu = msgs.get('/base/imu', [])
    if len(gt) < 10 or len(imu) < 10:
        raise RuntimeError(f'not enough ground truth ({len(gt)}) or IMU ({len(imu)}) messages in {bag_dir}')

    # ---------------- ground truth (header stamp = sim time of the physics step)
    gt_ns = np.array([stamp_ns(m.header) for _, m in gt], dtype=np.int64)
    keep = np.concatenate([[True], np.diff(gt_ns) > 0])
    gt = [g for g, k in zip(gt, keep) if k]
    gt_ns = gt_ns[keep]
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in gt])
    quat = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                      m.pose.pose.orientation.w] for _, m in gt])
    v_b = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for _, m in gt])
    w_b = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for _, m in gt])

    # ---------------- IMU
    imu_ns = np.array([stamp_ns(m.header) for _, m in imu], dtype=np.int64)
    imu_q = np.array([[m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w] for _, m in imu])
    imu_w = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
    imu_a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])

    # ---------------- grid
    k0 = -(-max(gt_ns[0], imu_ns[0]) // GRID_NS)
    k1 = min(gt_ns[-1], imu_ns[-1]) // GRID_NS
    grid = np.arange(k0, k1 + 1, dtype=np.int64) * GRID_NS
    t = grid * 1e-9

    g_pos = np.stack([np.interp(grid, gt_ns, pos[:, i]) for i in range(3)], axis=1)
    g_v = np.stack([np.interp(grid, gt_ns, v_b[:, i]) for i in range(3)], axis=1)
    g_w = np.stack([np.interp(grid, gt_ns, w_b[:, i]) for i in range(3)], axis=1)
    g_q = Slerp((gt_ns - gt_ns[0]) * 1e-9, Rotation.from_quat(quat))((grid - gt_ns[0]) * 1e-9).as_quat()
    g_q *= np.where(g_q[:, 3:4] < 0, -1.0, 1.0)          # w >= 0 (q and -q are the same rotation)

    step = float(np.median(np.diff(gt_ns)))
    j = np.clip(np.searchsorted(gt_ns, grid, side='right'), 1, len(gt_ns) - 1)
    gt_gap = (gt_ns[j] - gt_ns[j - 1]) > 1.5 * step

    i_q, imu_age = zoh(imu_ns, imu_q, grid)
    i_w, _ = zoh(imu_ns, imu_w, grid)
    i_a, _ = zoh(imu_ns, imu_a, grid)
    imu_period = float(np.median(np.diff(imu_ns))) * 1e-9
    valid = (~gt_gap) & (imu_age <= 2 * imu_period)

    cols = {'t': t, 'gt_pos': g_pos, 'gt_quat_xyzw': g_q, 'gt_lin_vel_body': g_v, 'gt_ang_vel_body': g_w,
            'imu_quat_xyzw': i_q, 'imu_ang_vel': i_w, 'imu_lin_acc': i_a, 'imu_age_s': imu_age,
            'valid': valid.astype(np.int8)}

    # ---------------- arm (B5, B8), gripper (B9, B10)
    js = msgs.get('/joint_states', [])
    names = list(joint_names or (js[0][1].name if js else []))
    if js and names:
        ns_ = np.array([stamp_ns(m.header) for _, m in js], dtype=np.int64)
        val = np.array([[dict(zip(m.name, m.position)).get(n, np.nan) for n in names] for _, m in js])
        cols['joint_pos'], _ = zoh(ns_, val, grid)
    jc = msgs.get('/joint_command', [])
    if jc and names:
        ns_ = np.array([stamp_ns(m.header) for _, m in jc], dtype=np.int64)
        val = np.array([[dict(zip(m.name, m.position)).get(n, np.nan) for n in names] for _, m in jc])
        cols['joint_cmd'], cols['joint_cmd_age_s'] = zoh(ns_, val, grid)
    gc = msgs.get('/gripper/command', [])
    if gc:      # std_msgs/Float64: no header -> bag receive time (sim time, recorded with --use-sim-time)
        cols['gripper_cmd_raw'], _ = zoh(np.array([tb for tb, _ in gc], dtype=np.int64),
                                         np.array([m.data for _, m in gc]), grid)
    gj = msgs.get('/gripper/joint_states', [])
    if gj:
        cols['gripper_pos_raw'], _ = zoh(np.array([stamp_ns(m.header) for _, m in gj], dtype=np.int64),
                                         np.array([m.position[0] for _, m in gj]), grid)

    # ---------------- files
    np.savez_compressed(os.path.join(run_dir, 'data_100hz.npz'), joint_names=np.array(names), **cols)
    header, items = [], {'t': 'A1', 'gt_pos': 'A2', 'gt_quat_xyzw': 'A3', 'gt_lin_vel_body': 'A4',
                         'gt_ang_vel_body': 'A5', 'imu_quat_xyzw': 'A6', 'imu_ang_vel': 'A7', 'imu_lin_acc': 'A8',
                         'joint_pos': 'B5', 'joint_cmd': 'B8', 'gripper_cmd_raw': 'B9', 'gripper_pos_raw': 'B10'}
    sub = {'gt_pos': 'xyz', 'gt_lin_vel_body': 'xyz', 'gt_ang_vel_body': 'xyz', 'imu_ang_vel': 'xyz',
           'imu_lin_acc': 'xyz', 'gt_quat_xyzw': 'xyzw', 'imu_quat_xyzw': 'xyzw'}
    mats = []
    for k, v in cols.items():
        v2 = v.reshape(len(t), -1)
        if v2.shape[1] == 1:
            header.append(f'{items[k]}_{k}' if k in items else k)
        else:
            labels = sub.get(k) or names
            header += [f'{items.get(k, "")}_{k}_{lab}'.lstrip('_') for lab in labels]
        mats.append(v2)
    table = np.hstack(mats)
    with open(os.path.join(run_dir, 'data_100hz.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for row in table:
            w.writerow(['' if np.isnan(x) else f'{x:.9g}' for x in row])

    # ---------------- E1
    e1 = {'grid': {'rate_hz': 100.0, 'rows': int(len(t)), 't_first_s': float(t[0]), 't_last_s': float(t[-1]),
                   'valid_rows': int(valid.sum()), 'invalid_rows': int((~valid).sum()),
                   'gt_gap_rows': int(gt_gap.sum())},
          'sources': {'/sim/boat_ground_truth': intervals(gt_ns), '/base/imu': intervals(imu_ns)},
          'imu_age_ms': {'max': round(float(np.nanmax(imu_age)) * 1e3, 3), 'mean': round(float(np.nanmean(imu_age)) * 1e3, 3)}}
    for topic in ('/joint_states', '/joint_command', '/gripper/joint_states'):
        rows = msgs.get(topic, [])
        if rows:
            e1['sources'][topic] = intervals(np.array([stamp_ns(m.header) for _, m in rows], dtype=np.int64))
    ps = msgs.get('/protective_stop', [])
    e1['protective_stop_true_msgs'] = int(sum(1 for _, m in ps if m.data))
    rtf_path = os.path.join(run_dir, 'rtf.jsonl')
    if os.path.exists(rtf_path):
        rtf = [json.loads(line)['rtf'] for line in open(rtf_path) if line.strip()]
        if rtf:
            e1['rtf'] = {'samples_1s_wall': len(rtf), 'mean': round(float(np.mean(rtf)), 4),
                         'min': round(float(np.min(rtf)), 4), 'max': round(float(np.max(rtf)), 4),
                         'below_0.9': int(np.sum(np.array(rtf) < 0.9))}
    e1['note'] = ('lockstep simulation: a real time factor below 1 only slows the wall clock, the sim time stamps and '
                  'the physics are the same')
    with open(os.path.join(run_dir, 'e1_timing.json'), 'w') as f:
        json.dump(e1, f, indent=1)
        f.write('\n')
    log(f'table: {len(t)} rows ({t[-1] - t[0]:.1f} s), valid {int(valid.sum())}, columns {len(header)} -> '
        f'{run_dir}/data_100hz.npz / .csv')
    return e1, header
