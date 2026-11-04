#!/usr/bin/env python3
"""Scripted drone grasp demo + judge in Gazebo, the same sequence and judge as Isaac Sim
(data_collection_ur5_gripper isaacsim/scripts/grasp_demo.py + grasp_judge.py, config/grasp_demo.yaml). docs/grasp_demo.md

The arm is driven only through the public interface (as a policy would): /joint_command (arm joint targets at
control_hz, IK done here), /gripper/command, /sim/drone_kill, /sim/reset. The judge also reads what a policy does not get:
/drone/odometry (drone body pose) and /drone/contacts (finger forces).

Run (calm water first):
    ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm
    ros2 run ocean_sim2sim_bringup grasp_demo.py --case all
Each case starts with /sim/reset with the drone at drone_pos_base (Isaac Sim default drone position; drone targets and
offset are switched off for the demo, --drone targets keeps the scene's shuffled targets).
Report: --report-dir (default ~/ocean_sim2sim_reports)/grasp_demo_<time>/ report.txt, result_<case>.json, timeline_<case>.csv
"""
import argparse
import datetime
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from ocean_sim2sim_interface.grasp_judge import GraspJudge
from ocean_sim2sim_interface.kinematics import rot_log
from ocean_sim2sim_interface.observe import GraspObservation

CASES = ('success', 'offset_y', 'offset_x', 'empty', 'miss')
EXPECT = {'success': ('held', None), 'offset_y': ('held', None), 'offset_x': ('held', None),
          'empty': ('failed', 'no_grasp_empty'), 'miss': ('failed', 'no_grasp')}
ROW_KEYS = ('t', 'phase', 'state', 'tcp_x', 'tcp_y', 'tcp_z', 'drone_x', 'drone_y', 'drone_z', 'grip_deg', 'f_right', 'f_left',
            'f_other', 'armed', 'rel_x', 'rel_y', 'rel_z', 'rel_rx_deg', 'rel_ry_deg', 'rel_rz_deg')


class Abort(Exception):
    pass


class Demo(Node, GraspObservation):
    def __init__(self, args, dc):
        super().__init__('grasp_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args, self.dc = args, dc
        # judge observation sources (ocean_sim2sim_interface/observe.py, shared with ocean_sim2sim_policy policy_eval.py)
        self.init_grasp_observation(dc['gripper_upper'], dc['gripper_raw_max'])
        self.arm_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.grip_pub = self.create_publisher(Float64, '/gripper/command', 10)
        self.reset_cli = self.create_client(Trigger, '/sim/reset')
        self.kill_cli = self.create_client(Trigger, '/sim/drone_kill')
        self.param_cli = self.create_client(SetParameters, '/ocean_sim2sim_scene/set_parameters')

    # ------------------------------------------------------------------ setup
    def setup(self):
        a = self.args
        t0 = time.time()
        while rclpy.ok():                                       # scene spawns the first drone (~8 s sim)
            miss = dict(self.observation_missing(), interface=self.arm_pub.get_subscription_count() == 0,
                        scene=not self.reset_cli.service_is_ready())
            if not any(miss.values()):
                break
            if time.time() - t0 > 90:
                raise RuntimeError(f'not ready within 90 s: {[k for k, v in miss.items() if v]}')
            time.sleep(0.1)
        self.setup_kinematics(a.base_link, a.tcp_link, a.boat_link)
        x_g = np.asarray(self.dc['tool_x_in_drone'], dtype=float)
        z_g = np.array([0.0, 0.0, 1.0])
        self.R_dg = np.column_stack([x_g, np.cross(z_g, x_g), z_g])     # gripper axes in the drone frame

    def set_scene_params(self, targets, offset, pos_base=None):
        req = SetParameters.Request()
        req.parameters = [
            ParamMsg(name='use_drone_targets', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=targets)),
            ParamMsg(name='drone_offset', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=offset))]
        if pos_base is not None:
            req.parameters.append(ParamMsg(name='drone_pos_base', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(v) for v in pos_base])))
        res = self.call(self.param_cli, req, 10.0)
        if not all(r.successful for r in res.results):
            raise RuntimeError(f'scene parameters not set: {res.results}')

    def call(self, cli, req, timeout):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {cli.srv_name} not available')
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise RuntimeError(f'service {cli.srv_name} timed out')
            time.sleep(0.01)
        return fut.result()

    # ------------------------------------------------------------------ commands
    def send_arm(self, q):
        m = JointState()
        m.name, m.position = list(self.arm), [float(v) for v in q]
        self.arm_pub.publish(m)

    def send_gripper(self, raw):
        self.grip_raw = float(raw)
        self.grip_pub.publish(Float64(data=float(raw)))


class Episode:
    """One case: control loop (control_hz, sim time), IK, judge, log. Mirrors Isaac Sim grasp_demo.Demo."""

    def __init__(self, n, case, dc):
        self.n, self.case, self.dc = n, case, dc
        self.cc = dict(dc['cases'].get(case, {}))
        if n.args.tcp_offset is not None:
            self.cc['tcp_offset'] = list(n.args.tcp_offset)
        self.judge = GraspJudge(dc['judge'])
        self.dt = 1.0 / dc['control_hz']
        self.phase = 'start'
        self.rows, self.cmd_rows = [], []
        self.link_force_max, self.f_other_max = {}, {}
        self.q_cmd = n.q_arm()
        self.armed = True
        self.t_kill = None
        n.grip_raw = 0.0
        self.next_tick = n.now()
        self.p_close0, self.push_max_mm = None, 0.0
        p_d, R_d = n.drone_world()
        self.drone_start = (p_d.copy(), R_d.copy())

    # ---- geometry
    def grasp_target(self):
        """Grasp TCP pose (world). empty: the drone was spawned at default + drone_offset (Isaac Sim), the arm aims at
        the default position (drone - drone_offset, base frame = level, yaw of the drone)."""
        p, R = self.n.drone_world()
        if self.case == 'empty':
            p = p - R @ np.asarray(self.cc['drone_offset'], dtype=float)
        off = np.asarray(self.cc.get('tcp_offset', [0, 0, 0]), dtype=float)
        return p + R @ np.asarray(self.dc['tcp_in_drone']) + off, R @ self.n.R_dg

    # ---- one control tick
    def tick(self, control=None):
        n = self.n
        # poll every 2 ms (was 0.5 ms: a busy wait that took CPU and the GIL from the spin thread; same as policy_eval).
        # A tick starts at most 2 ms (wall) late in a 20 ms (50 Hz) period; commands carry the sim time of the tick.
        while rclpy.ok() and n.now() < self.next_tick:
            time.sleep(0.002)
        self.next_tick += self.dt
        self.armed = n.drone_armed(self.armed, self.t_kill is not None)
        o = self.observe()
        self.judge.update(o)
        if control is not None:
            control()
        n.send_arm(self.q_cmd)
        self.cmd_rows.append((o['t'], *self.q_cmd, n.grip_raw))
        rel_p = o['tcp_R'].T @ (o['drone_p'] - o['tcp_p'])
        rel_rv = Rotation.from_matrix(o['tcp_R'].T @ o['drone_R']).as_rotvec()
        self.rows.append((o['t'], self.phase, self.judge.state, *o['tcp_p'], *o['drone_p'], math.degrees(o['grip_q']),
                          o['f_right'], o['f_left'], o['f_other'], int(o['armed']), *rel_p, *np.degrees(rel_rv)))
        if self.phase == 'S3_close':
            if self.p_close0 is None:
                self.p_close0 = o['drone_p'].copy()
            self.push_max_mm = max(self.push_max_mm, float(np.linalg.norm(o['drone_p'] - self.p_close0)) * 1000)
        if o['f_other'] > 1.0:
            self.f_other_max[self.phase] = max(self.f_other_max.get(self.phase, 0.0), o['f_other'])
        if self.judge.state == 'failed':
            raise Abort(self.judge.reason)

    def observe(self):
        n = self.n
        self.js_age_max = max(getattr(self, 'js_age_max', 0.0), n.now() - n.js_t)
        o, f = n.judge_observation(n.grip_raw > 0.5 * self.dc['gripper_raw_max'], self.armed)
        for k, v in f.items():
            if v > 1.0:
                ph = self.link_force_max.setdefault(self.phase, {})
                ph[k] = max(ph.get(k, 0.0), v)
        return o

    def wait(self, sec, until=None):
        t_end = self.n.now() + sec
        while self.n.now() < t_end:
            self.tick()
            if until is not None and until():
                return True
        return False

    # ---- motions
    def joint_move(self, q_goal, speed):
        q0 = self.n.q_arm()
        q_goal = np.asarray(q_goal, dtype=float)
        T = max(float(np.abs(q_goal - q0).max()) / speed, self.dt)
        t0 = self.n.now()

        def ctrl():
            u = min((self.n.now() - t0) / T, 1.0)
            u = u * u * (3 - 2 * u)
            self.q_cmd = q0 + u * (q_goal - q0)

        while self.n.now() - t0 < T + 0.5:
            self.tick(ctrl)

    def ik_step(self, p_w, R_w):
        """DiffIK step added to the accumulated joint target. ik_feedback (config):
          measured: error and Jacobian at the MEASURED joints (Isaac Sim DiffIK: also makes up for gravity sag)
          command:  at the commanded joints (kinematic IK, the arm follows the command with its own lag)"""
        n, dc = self.n, self.dc
        Tb = n.base_world()
        Rb = Tb[:3, :3]
        p_b, R_b = Rb.T @ (p_w - Tb[:3, 3]), Rb.T @ R_w          # target in the robot base frame (boat moves)
        q_meas = n.q_arm()
        q_fb = q_meas if dc.get('ik_feedback', 'measured') == 'measured' else self.q_cmd
        T = n.chain.fk(q_fb)
        e = np.concatenate([p_b - T[:3, 3], rot_log(R_b @ T[:3, :3].T)])
        J = n.chain.jacobian(q_fb)
        dq = J.T @ np.linalg.solve(J @ J.T + dc['ik_damping'] ** 2 * np.eye(6), e)
        m = np.max(np.abs(dq)) / dc['max_dq_per_tick']
        if m > 1:
            dq /= m
        self.q_cmd = np.clip(self.q_cmd + dq, q_meas - dc['max_lead'], q_meas + dc['max_lead'])
        return float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))

    def cart_move(self, goal_fn, speed):
        """TCP straight to goal_fn() (world, called every tick: follows the drone), orientation slerp."""
        tol = self.dc['reach']
        p0, R0 = self.n.tcp_world()
        g0, _ = goal_fn()
        T = max(float(np.linalg.norm(g0 - p0)) / speed, 0.2)
        t0 = self.n.now()
        st = {'ep': 1.0, 'er': 1.0}

        def ctrl():
            g, Rg = goal_fn()
            u = min((self.n.now() - t0) / T, 1.0)
            u = u * u * (3 - 2 * u)
            R_t = Slerp([0, 1], Rotation.from_matrix([R0, Rg]))(u).as_matrix()
            st['ep'], st['er'] = self.ik_step(p0 + u * (g - p0), R_t)
            if u >= 1.0:
                tp, tR = self.n.tcp_world()
                st['ep'], st['er'] = float(np.linalg.norm(g - tp)), float(np.linalg.norm(rot_log(Rg @ tR.T)))

        while True:
            self.tick(ctrl)
            done = self.n.now() - t0 >= T
            if done and st['ep'] <= tol['pos_tol'] and math.degrees(st['er']) <= tol['rot_tol_deg']:
                return
            if self.n.now() - t0 > T + tol['timeout']:
                raise Abort(f"reach_timeout:{self.phase} (position error {st['ep'] * 1000:.1f} mm, "
                            f"orientation {math.degrees(st['er']):.2f} deg)")

    def sequence(self):
        n, dc = self.n, self.dc
        self.phase = 'S0_home'
        self.joint_move(dc['home_pose'], dc['joint_speed'])
        self.phase = 'S1_seed'
        self.joint_move(dc['seed_pose'], dc['joint_speed'])
        self.q_cmd = n.q_arm() if self.q_cmd is None else self.q_cmd
        self.phase = 'S1_pregrasp'
        self.cart_move(lambda: (self.grasp_target()[0] - np.array([0, 0, dc['pregrasp_below']]), self.grasp_target()[1]),
                       dc['move_speed'])
        self.phase = 'S2_approach'
        frozen = {}

        def approach_goal():
            if 'g' in frozen:
                return frozen['g']
            g = self.grasp_target()
            if g[0][2] - n.tcp_world()[0][2] <= dc['approach_track_until']:
                frozen['g'] = g
            return g

        self.cart_move(approach_goal, dc['approach_speed'])
        self.phase = 'S3_close'
        n.send_gripper(dc['gripper_raw_max'])
        self.wait(dc['grip_close_timeout'], until=lambda: self.judge.state != 'approach')
        if self.judge.state == 'approach':
            raise Abort('grasp_timeout')
        self.phase = 'S3_hold'
        self.wait(dc['kill_delay'])
        self.phase = 'S4_release'
        self.t_kill = n.now()
        fut = n.kill_cli.call_async(Trigger.Request())
        self.wait(dc['hold_time'] + 0.5, until=lambda: self.judge.state == 'held')
        if fut.done() and not fut.result().success:
            raise Abort(f'drone_kill failed: {fut.result().message}')
        self.phase = 'S5_end'
        self.wait(dc['end_wait'])


def hold_metrics(rows, phase='S3_hold'):
    r = [x for x in rows if x[1] == phase]
    if len(r) < 10:
        return None
    a = {k: np.array([x[i] for x in r], dtype=float) for i, k in enumerate(ROW_KEYS) if k not in ('phase', 'state')}
    rel = np.column_stack([a['rel_x'], a['rel_y'], a['rel_z']]) * 1000
    Rr = Rotation.from_rotvec(np.radians(np.column_stack([a['rel_rx_deg'], a['rel_ry_deg'], a['rel_rz_deg']])))
    rot = np.degrees((Rr[0].inv() * Rr).as_rotvec())
    out = {'duration_s': round(float(a['t'][-1] - a['t'][0]), 3), 'samples': len(r),
           'rel_pos_ptp_mm': np.round(np.ptp(rel, axis=0), 2).tolist(), 'rel_pos_std_mm': np.round(rel.std(axis=0), 3).tolist(),
           'rel_rot_ptp_deg': np.round(np.ptp(rot, axis=0), 3).tolist()}
    for k in ('f_right', 'f_left'):
        out[f'{k}_mean_N'], out[f'{k}_std_N'] = round(float(a[k].mean()), 2), round(float(a[k].std()), 3)
    return out


def run_case(n, case, dc, out_dir, log):
    if not n.args.no_reset:
        moved = 'drone_offset' in dc['cases'].get(case, {}) and n.args.drone == 'default'
        if moved:     # empty: drone at the default position + drone_offset for this reset (Isaac Sim)
            n.set_scene_params(False, 0.0, np.asarray(n.default_pos) + np.asarray(dc['cases'][case]['drone_offset']))
        try:
            # the scene accepts /sim/reset only after its first drone is up (PX4: after the air start, ~20 s sim after
            # the launch, longer in wall time with the GUI): wait and try again instead of failing
            t_wait = time.time()
            while True:
                res = n.call(n.reset_cli, Trigger.Request(), 120.0)
                if res.success or 'not ready' not in res.message or time.time() - t_wait > 300:
                    break
                if t_wait + 2.5 > time.time():
                    print('[grasp_demo] scene not ready yet (first drone still starting), waiting ...', flush=True)
                time.sleep(2.0)
        finally:
            if moved:
                n.set_scene_params(False, 0.0, n.default_pos)
        if not res.success:
            raise RuntimeError(f'/sim/reset failed: {res.message}')
        info = json.loads(res.message)
        log(f"\n===== case {case} =====  reset: drone base {info['drone_pos_base']} ({info['drone_target']})")
    else:
        log(f'\n===== case {case} =====')
    t0 = time.time()
    # fixed drone: hanging on its anchor; PX4 drone: flying (released after the air start, armed)
    while n.drone is None or (n.drone_state != 'attached' if n.px4 is None else not n.px4.get('armed')):
        if time.time() - t0 > 10:
            raise RuntimeError('no drone odometry / drone not attached')
        time.sleep(0.05)
    time.sleep(0.2)
    ep = Episode(n, case, dc)
    abort = None
    try:
        ep.sequence()
    except Abort as e:
        abort = str(e)
    n.send_gripper(0.0)
    res = ep.judge.finish(n.now())
    log(f'  joint state age at the control ticks: max {ep.js_age_max * 1000:.0f} ms')
    res.update(case=case, abort=abort, push_max_mm=round(ep.push_max_mm, 2), kill_delay=dc['kill_delay'],
               hold=hold_metrics(ep.rows), approach=hold_metrics(ep.rows, 'S2_approach'),
               f_other_max_N={k: round(v, 2) for k, v in ep.f_other_max.items()},
               link_force_max_N={ph: {k: round(v, 2) for k, v in m.items()} for ph, m in ep.link_force_max.items()},
               tcp_in_drone=dc['tcp_in_drone'], case_config=ep.cc)
    exp_state, exp_reason = EXPECT[case]
    ok = res['state'] == exp_state and (exp_reason is None or str(res['reason']).startswith(exp_reason))
    res['expected'] = {'state': exp_state, 'reason': exp_reason}
    res['as_expected'] = ok
    for t, st, info in res['events']:
        log(f'  t {t:7.2f} s  {st:10s} {info}')
    log(f"  result {res['state']} ({res['reason']}), abort {abort}, drone pushed while closing max {res['push_max_mm']} mm, "
        f"settle after motor stop {res['settle_mm']} mm, slip after grasp max {res['max_slip_mm']} mm / "
        f"rotation {res['max_rot_deg']} deg, robot-drone contact other than fingers (> 1 N) {res['f_other_max_N'] or 'none'}")
    for ph, m in res['link_force_max_N'].items():
        log(f'    {ph:12s} robot links touching the drone (max N): {m}')
    if res['hold'] is not None:
        log(f"  holding with motors on {res['hold']['duration_s']} s (S3_hold): {res['hold']}")
    if res['approach'] is not None:
        log(f"  approach (S2_approach) drone position range in the TCP frame {res['approach']['rel_pos_ptp_mm']} mm")
    log(f"  expected {exp_state}/{exp_reason} -> {'OK' if ok else 'WRONG'}")
    with open(os.path.join(out_dir, f'result_{case}.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    with open(os.path.join(out_dir, f'timeline_{case}.csv'), 'w', encoding='utf-8') as f:
        f.write(','.join(ROW_KEYS) + '\n')
        for r in ep.rows:
            f.write(','.join(f'{v:.5f}' if isinstance(v, float) else str(v) for v in r) + '\n')
    with open(os.path.join(out_dir, f'commands_{case}.csv'), 'w', encoding='utf-8') as f:
        f.write('t,' + ','.join(n.arm) + ',gripper_raw\n')
        for r in ep.cmd_rows:
            f.write(','.join(f'{float(v):.6f}' for v in r) + '\n')
    return ok


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--case', nargs='+', default=['success'], choices=CASES + ('all',))
    a.add_argument('--config', default=os.path.join(get_package_share_directory('ocean_sim2sim_bringup'), 'config',
                                                    'grasp_demo.yaml'))
    a.add_argument('--repeat', type=int, default=1, help='run the case list this many times')
    a.add_argument('--drone', choices=('default', 'targets'), default='default',
                   help='default: drone at drone_pos_base (no offset) as Isaac Sim; targets: scene drone targets + offset')
    a.add_argument('--no-reset', action='store_true', help='do not call /sim/reset before a case (use the drone as it is)')
    a.add_argument('--tcp-offset', type=float, nargs=3, default=None, metavar=('DX', 'DY', 'DZ'))
    a.add_argument('--reach-tol', type=float, nargs=2, default=None, metavar=('MM', 'DEG'),
                   help='override reach.pos_tol / rot_tol_deg (Isaac Sim fixed table: 1.5 mm, 0.5 deg; waves need more)')
    a.add_argument('--report-dir', default=os.path.expanduser('~/ocean_sim2sim_reports'))
    a.add_argument('--base-link', default='robot_mount')
    a.add_argument('--tcp-link', default='rh_p12_rn_tcp')
    a.add_argument('--boat-link', default='wamv/base_link')
    args = a.parse_args()
    with open(args.config) as f:
        dc = yaml.safe_load(f)
    cases = list(CASES) if 'all' in args.case else args.case
    if args.reach_tol is not None:
        dc['reach']['pos_tol'], dc['reach']['rot_tol_deg'] = args.reach_tol[0] / 1000.0, args.reach_tol[1]

    rclpy.init()
    n = Demo(args, dc)
    ex = EventsExecutor()
    ex.add_node(n)
    def spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  shutdown at the end
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    out_dir = os.path.join(args.report_dir, 'grasp_demo_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    report = open(os.path.join(out_dir, 'report.txt'), 'w', encoding='utf-8')

    def log(s):
        print(s, flush=True)
        report.write(s + '\n')
        report.flush()

    try:
        n.setup()
        n.default_pos = dc['drone_pos_base']
        if not args.no_reset:
            n.set_scene_params(args.drone == 'targets', 0.03 if args.drone == 'targets' else 0.0, n.default_pos)
        log(f'grasp_demo {datetime.datetime.now().isoformat(timespec="seconds")}  cases {cases} x {args.repeat}, '
            f'drone {args.drone}, config {args.config}, reach {dc["reach"]}')
        oks = []
        for _ in range(args.repeat):
            for c in cases:
                oks.append((c, run_case(n, c, dc, out_dir, log)))
        log(f"\nas expected: {sum(ok for _, ok in oks)} / {len(oks)}  {[c for c, ok in oks if not ok] or ''}")
        log(f'report: {out_dir}')
    finally:
        if not args.no_reset:
            try:
                n.set_scene_params(True, 0.03)
            except Exception:  # noqa: BLE001
                pass
        report.close()
        # stop the executor thread before rclpy.shutdown (a daemon thread still inside the executor when the context
        # goes away ended the process with "terminate called without an active exception" / core dump)
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
