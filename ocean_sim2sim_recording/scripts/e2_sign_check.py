#!/usr/bin/env python3
# =============================================================
# e2_sign_check.py - spec E2: frames and signs, measured in calm water (docs/recording.md)
#
#   Starts its own simulation (world empty_ocean_calm = no waves + ApplyLinkWrench, drone_mode none, ground truth) and:
#     1. rest 10 s       : IMU specific force (gravity direction), ground truth / IMU orientation
#     2. bow up          : persistent torque -T about the world y axis on wamv/base_link (= bow up, boat heading 0),
#                          held --torque-time s: sign of the pitch change (ground truth, IMU orientation), of the IMU
#                          gyro y and of the ground truth body angular velocity y while the bow rises
#     3. release         : wrench cleared, 5 s
#   Every angle is given in the recorded convention (world ENU, body FLU, Z-Y-X yaw-pitch-roll) and converted to
#   NED / FRD (the predictor's convention, AURA data: "Z-Y-X of body FRD in world NED"), so the conversion is checked
#   on numbers, not only on paper.
#   Output: <out>/<stamp>_e2_sign_check/e2_sign_check.json (+ launch.log)
#
#       ros2 run ocean_sim2sim_recording e2_sign_check.py [--torque 3000] [--torque-time 8]
# =============================================================

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity, EntityWrench
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu

from ocean_sim2sim_recording import meta
from ocean_sim2sim_recording.proc import gazebo_servers, kill_group, sim_processes, sweep

ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
LINK = 'wamv::wamv/base_link'


def ypr_deg(R):
    """Z-Y-X (yaw, pitch, roll) [deg] of a rotation matrix (body -> world)."""
    return [round(float(v), 4) for v in np.degrees(Rotation.from_matrix(R).as_euler('ZYX'))]


def both_conventions(q_xyzw):
    R = Rotation.from_quat(q_xyzw).as_matrix()
    R_ned_frd = ENU_TO_NED @ R @ FLU_TO_FRD.T
    return {'enu_flu_yaw_pitch_roll_deg': ypr_deg(R), 'ned_frd_yaw_pitch_roll_deg': ypr_deg(R_ned_frd)}


class N(Node):
    def __init__(self, world):
        super().__init__('e2_sign_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt, self.imu = [], []
        self.create_subscription(Odometry, '/sim/boat_ground_truth', lambda m: self.gt.append(m), 1000)
        self.create_subscription(Imu, '/base/imu', lambda m: self.imu.append(m), 1000)
        self.pub_w = self.create_publisher(EntityWrench, f'/world/{world}/wrench/persistent', 10)
        self.pub_c = self.create_publisher(Entity, f'/world/{world}/wrench/clear', 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, sec):
        t0 = self.now()
        while self.now() - t0 < sec:
            time.sleep(0.01)


def t_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def window(msgs, t0, t1):
    return [m for m in msgs if t0 <= t_of(m) < t1]


def main():
    p = argparse.ArgumentParser(description='Spec E2: frames and signs in calm water (gravity, bow up).')
    p.add_argument('--torque', type=float, default=500.0, help='bow up torque [N m] (3000 N m: about 12 deg)')
    p.add_argument('--torque-time', type=float, default=8.0, help='[s sim]')
    p.add_argument('--out-dir', default='')
    a = p.parse_args()
    root = meta.repo_root()
    out = a.out_dir or (os.path.join(root, 'ocean_gazebo_recordings') if root else os.path.expanduser('~/ocean_gazebo_recordings'))
    d = os.path.join(out, datetime.now().strftime('%Y%m%d_%H%M%S') + '_e2_sign_check')
    os.makedirs(d)
    if gazebo_servers() or sim_processes():
        sys.exit('a simulation is already running: stop it first')
    world = 'empty_ocean_calm'       # Gazebo names the world after the file (topics /world/empty_ocean_calm/...)
    f = open(os.path.join(d, 'launch.log'), 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'ocean_sim2sim_bringup', 'wamv_arm.launch.py', 'world:=empty_ocean_calm',
                               'headless:=true', 'drone_mode:=none', 'ground_truth:=true'],
                              stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                               f'/world/{world}/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
                               f'/world/{world}/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    rclpy.init()
    n = N(world)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    res = {'torque_nm': a.torque, 'torque_time_s': a.torque_time}
    try:
        t0 = time.time()
        while 'scene ready without a drone' not in open(os.path.join(d, 'launch.log'), errors='replace').read():
            if launch.poll() is not None or time.time() - t0 > 240:
                raise RuntimeError('simulation did not get ready, see launch.log')
            time.sleep(1.0)
        while n.pub_w.get_subscription_count() == 0:
            time.sleep(0.1)
        n.wait(3.0)
        # 1. rest
        r0 = n.now()
        n.wait(10.0)
        r1 = n.now()
        gt_r, imu_r = window(n.gt, r0, r1), window(n.imu, r0, r1)
        acc = np.mean([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu_r], axis=0)
        q_gt = [gt_r[-1].pose.pose.orientation.x, gt_r[-1].pose.pose.orientation.y, gt_r[-1].pose.pose.orientation.z,
                gt_r[-1].pose.pose.orientation.w]
        q_imu = [imu_r[-1].orientation.x, imu_r[-1].orientation.y, imu_r[-1].orientation.z, imu_r[-1].orientation.w]
        res['rest'] = {'imu_specific_force_mean_mps2': [round(float(v), 4) for v in acc],
                       'gt_orientation': both_conventions(q_gt), 'imu_orientation': both_conventions(q_imu),
                       'gt_position_m': [round(gt_r[-1].pose.pose.position.x, 4), round(gt_r[-1].pose.pose.position.y, 4),
                                         round(gt_r[-1].pose.pose.position.z, 4)]}
        # 2. bow up: torque about world -y (heading 0: body y = world y; +y rotation turns the bow down in FLU)
        w = EntityWrench()
        w.entity.name, w.entity.type = LINK, Entity.LINK
        w.wrench.torque.y = -a.torque
        b0 = n.now()
        n.pub_w.publish(w)
        n.wait(a.torque_time)
        b1 = n.now()
        c = Entity()
        c.name, c.type = LINK, Entity.LINK
        n.pub_c.publish(c)
        n.wait(5.0)
        r2 = n.now()
        gt_b, imu_b = window(n.gt, b0, b1), window(n.imu, b0, b1)
        gt_rise = window(n.gt, b0, b0 + 0.5)
        imu_rise = window(n.imu, b0, b0 + 0.5)
        q_gt_b = [gt_b[-1].pose.pose.orientation.x, gt_b[-1].pose.pose.orientation.y, gt_b[-1].pose.pose.orientation.z,
                  gt_b[-1].pose.pose.orientation.w]
        q_imu_b = [imu_b[-1].orientation.x, imu_b[-1].orientation.y, imu_b[-1].orientation.z, imu_b[-1].orientation.w]
        pitch_series = [both_conventions([m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                                          m.pose.pose.orientation.w])['enu_flu_yaw_pitch_roll_deg'][1] for m in gt_b]
        res['bow_up'] = {
            'gt_orientation_end': both_conventions(q_gt_b), 'imu_orientation_end': both_conventions(q_imu_b),
            'gt_pitch_enu_flu_deg_min_max': [round(min(pitch_series), 4), round(max(pitch_series), 4)],
            'first_0.5s_imu_gyro_mean_radps': [round(float(v), 5) for v in np.mean(
                [[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu_rise], axis=0)],
            'first_0.5s_gt_ang_vel_body_mean_radps': [round(float(v), 5) for v in np.mean(
                [[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in gt_rise], axis=0)],
            'bow_tip_height_change_m': None}
        # bow tip (x = +2.5 m in the body) height change, ground truth
        def tip_z(m):
            q = [m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w]
            return m.pose.pose.position.z + Rotation.from_quat(q).apply([2.5, 0.0, 0.0])[2]
        res['bow_up']['bow_tip_height_change_m'] = round(tip_z(gt_b[-1]) - tip_z(gt_r[-1]), 4)
        rel = window(n.gt, r2 - 1.0, r2)
        res['release_end_gt_orientation'] = both_conventions(
            [rel[-1].pose.pose.orientation.x, rel[-1].pose.pose.orientation.y, rel[-1].pose.pose.orientation.z,
             rel[-1].pose.pose.orientation.w])
        bu = res['bow_up']
        pitch_flu = bu['gt_orientation_end']['enu_flu_yaw_pitch_roll_deg'][1] - res['rest']['gt_orientation']['enu_flu_yaw_pitch_roll_deg'][1]
        pitch_frd = bu['gt_orientation_end']['ned_frd_yaw_pitch_roll_deg'][1] - res['rest']['gt_orientation']['ned_frd_yaw_pitch_roll_deg'][1]
        res['conclusion'] = {
            'gravity': 'IMU specific force at rest is +g on the IMU z axis (z up, FLU): ' + str(res['rest']['imu_specific_force_mean_mps2']),
            'bow_up_pitch_change_deg': {'recorded_enu_flu': round(pitch_flu, 4), 'converted_ned_frd': round(pitch_frd, 4)},
            'bow_up_gyro_y_sign': 'negative' if bu['first_0.5s_imu_gyro_mean_radps'][1] < 0 else 'positive',
            'rule': 'bow up = negative pitch and negative gyro y in the recorded ENU / FLU convention; positive pitch '
                    'in NED / FRD. Conversion: R_ned_frd = M R_enu_flu F^T with M = [[0,1,0],[1,0,0],[0,0,-1]] '
                    '(ENU -> NED), F = diag(1,-1,-1) (FLU -> FRD); vectors in the body frame: FRD = F FLU '
                    '(gyro / acc: x same, y and z negated); roll same, pitch negated, yaw_NED = 90 deg - yaw_ENU'}
        meta.write_json(os.path.join(d, 'e2_sign_check.json'), res)
        print(json.dumps(res['conclusion'], indent=1))
        print(f'-> {d}/e2_sign_check.json')
    finally:
        kill_group(bridge, signal.SIGINT, 10.0)
        kill_group(launch, signal.SIGINT, 30.0)
        sweep(print)
        ex.shutdown()                 # stop the spin thread before rclpy goes away (else: C++ terminate)
        spin.join(timeout=5.0)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
