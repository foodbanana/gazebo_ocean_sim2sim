#!/usr/bin/env python3
# =============================================================
# arm_motion_node.py - arm motion for the boat motion recordings (docs/recording.md)
#
#   --mode fixed    : the arm holds the init pose (capture ready pose, gripper up under the drone area), gripper open.
#                     /joint_command keeps being sent (the init pose) so B8 exists in every run.
#   --mode scripted : capture-like motion, repeated: random grasp point in the drone target area (config grasp_box) ->
#                     straight line to the pregrasp point below it -> straight up to the grasp point (approach speed)
#                     -> close the gripper, hold -> open -> back down -> sometimes back to the init pose TCP -> pause.
#                     Speeds and times uniform in the config ranges, all from --seed (same seed = same motion).
#   Commands: /joint_command (sensor_msgs/JointState, 60 Hz sim time) through the Isaac Sim / real-robot interface as
#   a teleop or policy would (B8), /gripper/command (std_msgs/Float64 raw). Segment starts: /recording/arm_event
#   (std_msgs/String JSON). IK = damped least squares on the commanded joints (kinematic, as grasp_demo.py), joint
#   target rate limit max_joint_speed and lead limit max_lead over the measured joints (protective stop at 5 deg).
#   A protective stop ends the node with exit code 3 (the run is not valid).
#
#   Run (sim up, arm at the init pose after /sim/reset; record_run.py does this):
#       ros2 run ocean_sim2sim_recording arm_motion_node.py --mode scripted --seed 1 [--duration 600]
# =============================================================

import argparse
import json
import math
import os
import random
import signal
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from ocean_sim2sim_interface.kinematics import Chain, dls_step


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class ArmMotion(Node):
    def __init__(self, cfg, mode, seed):
        super().__init__('recording_arm_motion', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cfg, self.mode, self.seed = cfg, mode, seed
        self.rng = random.Random(seed)
        self.chain = None
        self.q_meas = None
        self.pstop = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(Bool, '/protective_stop', lambda m: setattr(self, 'pstop', self.pstop or m.data), 10)
        self.pub_cmd = self.create_publisher(JointState, '/joint_command', 10)
        self.pub_grip = self.create_publisher(Float64, '/gripper/command', 10)
        self.pub_event = self.create_publisher(String, '/recording/arm_event', 10)

    def _on_description(self, m):
        self.chain = Chain(m.data, self.cfg['base_link'], self.cfg['tcp_link'])

    def _on_js(self, m):
        if self.chain is None:
            return
        d = dict(zip(m.name, m.position))
        if all(j in d for j in self.chain.joints):
            self.q_meas = np.array([d[j] for j in self.chain.joints])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, seg, **kw):
        msg = dict(t=round(self.now(), 4), mode=self.mode, seg=seg, **kw)
        self.pub_event.publish(String(data=json.dumps(msg)))
        self.get_logger().info(json.dumps(msg))

    def gripper(self, closed):
        self.pub_grip.publish(Float64(data=float(self.cfg['gripper_raw_max'] if closed else 0.0)))

    # ------------------------------------------------------------------ scripted motion = list of segments
    def segments(self, p_home, R_home):
        """Endless generator of segments: ('cart', name, p_goal, R_goal, speed, info) | ('wait', name, T, gripper, info)."""
        c = self.cfg
        box, pre = c['grasp_box'], c['pregrasp_below']
        u = self.rng.uniform
        cycle = 0
        while True:
            p = np.array([u(*box['x']), u(*box['y']), u(*box['z'])])
            yaw = u(-c['yaw_range'], c['yaw_range'])
            R = rot_z(yaw) @ R_home
            v_move, v_app = u(*c['move_speed']), u(*c['approach_speed'])
            hold, pause = u(*c['hold_time']), u(*c['pause'])
            home = self.rng.random() < c['home_probability']
            info = dict(cycle=cycle, grasp_base=[round(v, 4) for v in p], yaw=round(yaw, 4))
            below = p - np.array([0.0, 0.0, pre])
            yield ('cart', 'to_pregrasp', below, R, v_move, dict(info, speed=round(v_move, 3)))
            yield ('cart', 'approach', p, R, v_app, dict(info, speed=round(v_app, 3)))
            yield ('wait', 'close_hold', c['gripper_time'] + hold, True, dict(info, hold=round(hold, 3)))
            yield ('wait', 'open', c['gripper_time'], False, info)
            yield ('cart', 'retreat', below, R, v_move, dict(info, speed=round(v_move, 3)))
            if home:
                yield ('cart', 'home', p_home, R_home, v_move, dict(info, speed=round(v_move, 3)))
            if pause > 0:
                yield ('wait', 'pause', pause, None, dict(info, pause=round(pause, 3)))
            cycle += 1

    # ------------------------------------------------------------------ main loop
    def run(self, duration, stop, wait_subscribers=0):
        c = self.cfg
        while rclpy.ok() and not stop['v'] and (self.chain is None or self.q_meas is None or self.now() <= 0.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        # a recorder must see the command topics BEFORE the motion starts (discovery of a new topic takes ~2 s;
        # without this the first commands are missing in the bag)
        pubs = (self.pub_cmd, self.pub_grip, self.pub_event)
        while (rclpy.ok() and not stop['v'] and wait_subscribers > 0
               and min(p.get_subscription_count() for p in pubs[:2]) < wait_subscribers):
            rclpy.spin_once(self, timeout_sec=0.1)
        if wait_subscribers > 0:
            self.get_logger().info('subscribers: ' + ', '.join(f'{p.topic_name} {p.get_subscription_count()}' for p in pubs))
            t_w = self.now()
            while rclpy.ok() and not stop['v'] and self.now() - t_w < 0.5:     # let the subscriptions settle
                rclpy.spin_once(self, timeout_sec=0.05)
        if stop['v'] or not rclpy.ok():
            return 0
        names = list(self.chain.joints)
        init = np.array(c['init_pose'], dtype=float)
        T_home = self.chain.fk(init)
        p_home, R_home = T_home[:3, 3].copy(), T_home[:3, :3].copy()
        dt = 1.0 / c['publish_hz']
        step_max = c['max_joint_speed'] * dt
        q_cmd = self.q_meas.copy()
        off = np.abs(q_cmd - init).max()
        if off > 0.05:
            self.get_logger().error(f'arm is {off:.3f} rad from the init pose: call /sim/reset first')
            return 2
        t0 = self.now()
        self.gripper(False)
        self.event('start', seed=self.seed, duration=duration, init_pose=list(c['init_pose']),
                   tcp_home_base=[round(v, 4) for v in p_home])
        gen = self.segments(p_home, R_home) if self.mode == 'scripted' else None
        seg, seg_t0, p_from, R_from, seg_T = None, None, p_home, R_home, 0.0
        next_tick = self.now()
        while rclpy.ok() and not stop['v']:
            rclpy.spin_once(self, timeout_sec=0.002)
            t = self.now()
            if t < next_tick:
                continue
            next_tick += dt
            if next_tick < t:                       # sim jumped (should not happen): no burst of catch-up ticks
                next_tick = t + dt
            if self.pstop:
                self.event('protective_stop')
                self.get_logger().error('protective stop: the run is not valid')
                return 3
            if duration > 0 and t - t0 >= duration:
                self.event('end', elapsed=round(t - t0, 3))
                return 0
            if self.mode == 'fixed':
                q_goal = init
            else:
                if seg is None or t - seg_t0 >= seg_T:
                    if seg is not None and seg[0] == 'cart':
                        p_from, R_from = seg[2], seg[3]
                    seg = next(gen)
                    seg_t0 = t
                    if seg[0] == 'cart':
                        dist = float(np.linalg.norm(seg[2] - p_from))
                        ang = float(np.linalg.norm(Rotation.from_matrix(seg[3] @ R_from.T).as_rotvec()))
                        # min-jerk peak speed = 1.875 x mean: mean speed = the drawn speed
                        seg_T = max(dist / seg[4], ang / 1.0, 0.3)
                        slerp = Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([R_from, seg[3]])))
                        self.event(seg[1], T=round(seg_T, 3), goal_base=[round(v, 4) for v in seg[2]], **seg[5])
                    else:
                        seg_T = seg[2]
                        if seg[3] is not None:
                            self.gripper(seg[3])
                        self.event(seg[1], T=round(seg_T, 3), **seg[4])
                if seg[0] == 'cart':
                    s = min_jerk((t - seg_t0) / seg_T)
                    p_t = p_from + s * (seg[2] - p_from)
                    R_t = slerp([s]).as_matrix()[0]
                    q_goal = q_cmd.copy()
                    for _ in range(c['ik_iters']):
                        dq, e = dls_step(self.chain, q_goal, p_t, R_t, c['ik_damping'], 0.2)
                        q_goal = q_goal + dq
                        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                            break
                else:
                    q_goal = q_cmd
            dq = np.clip(q_goal - q_cmd, -step_max, step_max)
            q_new = q_cmd + dq
            q_new = np.clip(q_new, self.q_meas - c['max_lead'], self.q_meas + c['max_lead'])
            q_cmd = q_new
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = names
            m.position = [float(v) for v in q_cmd]
            self.pub_cmd.publish(m)
        self.event('stopped', elapsed=round(self.now() - t0, 3))
        return 0


def load_cfg(path):
    if not path:
        path = os.path.join(get_package_share_directory('ocean_sim2sim_recording'), 'config', 'recording.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['arm_motion']


def main():
    a = argparse.ArgumentParser(description='Arm motion for boat motion recordings (fixed / scripted capture-like).')
    a.add_argument('--mode', choices=('fixed', 'scripted'), required=True)
    a.add_argument('--seed', type=int, default=0)
    a.add_argument('--duration', type=float, default=0.0, help='sim seconds, 0 = until SIGINT / SIGTERM')
    a.add_argument('--config', default='', help='recording.yaml (default: installed config)')
    a.add_argument('--wait-subscribers', type=int, default=0,
                   help='start only when /joint_command and /gripper/command have this many subscribers '
                        '(record_run.py: 2 = interface + bag)')
    args = a.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    cfg = load_cfg(args.config)
    stop = {'v': False}
    rclpy.init()
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(v=True))
    node = ArmMotion(cfg, args.mode, args.seed)
    try:
        code = node.run(args.duration, stop, args.wait_subscribers)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
