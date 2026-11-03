#!/usr/bin/env python3
"""Policy evaluation in the Gazebo ocean sim: N episodes of reset -> openpi policy -> motors off -> grasp judge. docs/policy.md

Per episode:
  1. /sim/reset (the scene's own seed / drone targets / offset are used as they are; its JSON answer is recorded)
  2. wait for the drone (PX4 armed / fixed drone attached), 0.2 s sim
  3. the policy client (ocean_sim2sim_policy/client.py, the same code as policy_client_node.py) runs --duration sim s;
     at the same time the grasp judge (ocean_sim2sim_interface grasp_judge.GraspJudge, config = the judge section of
     ocean_sim2sim_bringup config/grasp_demo.yaml) is updated at its control_hz from the same observation sources as
     grasp_demo.py (ocean_sim2sim_interface/observe.py: /drone/odometry, /drone/contacts, TCP from /robot_description +
     /sim/boat_odometry, /drone/px4/state armed, /gripper/joint_states) and /gripper/command (close command)
  4. drone motors off: --auto-kill S (client rule: S s after the close command while the gripper is blocked) or the
     operator (ros2 run ocean_sim2sim_drone drone_cmd.py kill, or ros2 service call /sim/drone_kill ...). If the drone
     is grasped and still on when the policy ends: --auto-kill -> motors off at once, otherwise wait --operator-wait s
     for the operator (the arm holds the last target)
  5. after motors off the judge needs hold_time (2.5 s) -> wait until it decides (held / failed), then finish
Records protective stop events (/protective_stop), inference timing, gripper changes.

Report: --report-dir (default ~/ocean_sim2sim_reports)/policy_eval_<time>/
  report.txt (log + summary table), summary.json, summary.csv, ep<NN>_result.json (reset answer, client result, judge
  result, protective stop events), ep<NN>_timeline.csv (judge ticks), ep<NN>_commands.csv (sent arm commands)

Run (Gazebo stack + policy server first, venv with openpi-client active, docs/policy.md):
    ros2 run ocean_sim2sim_policy policy_eval.py --episodes 10 --duration 30 [--auto-kill 1.0]
"""
import argparse
import csv
import datetime
import json
import os
import sys
import threading
import time
from collections import Counter

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64

from ocean_sim2sim_interface.grasp_judge import GraspJudge
from ocean_sim2sim_interface.observe import GraspObservation
from ocean_sim2sim_policy.client import PolicyClient, load_config

ROW_KEYS = ('t', 'phase', 'state', 'tcp_x', 'tcp_y', 'tcp_z', 'drone_x', 'drone_y', 'drone_z', 'grip_deg', 'f_right',
            'f_left', 'f_other', 'armed', 'grip_cmd_raw', 'pstop', 'rel_x', 'rel_y', 'rel_z')
SUMMARY_KEYS = ('episode', 'drone_target', 'drone_pos_base', 'success', 'state', 'reason', 'policy_stop', 'policy_sim_s',
                't_grasped', 't_motor_off', 'chunks', 'infer_wall_median_s', 'infer_wall_max_s', 'chunk_arrive_step_median',
                'held_frac', 'limited_frac', 'gripper_changes', 'protective_stops', 'auto_killed', 'wall_s')


class EvalNode(Node, GraspObservation):
    """Judge observations (observe.py) + close command + protective stop events, sim time."""

    def __init__(self, dc, topics):
        super().__init__('policy_eval', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.init_grasp_observation(dc['gripper_upper'], dc['gripper_raw_max'])
        self.grip_cmd = 0.0
        self.pstop, self.pstop_events = None, []     # (sim t, stopped) at every change
        self.create_subscription(Float64, topics['gripper_command'], lambda m: setattr(self, 'grip_cmd', float(m.data)), 10)
        self.create_subscription(Bool, topics['protective_stop'], self._on_pstop, 10)

    def _on_pstop(self, m):
        v = bool(m.data)
        if v != self.pstop:
            if self.pstop is not None or v:
                self.pstop_events.append((round(self.now(), 3), v))
            self.pstop = v


class JudgeLoop(threading.Thread):
    """GraspJudge.update at control_hz (sim time) with the observe.py observation, as grasp_demo.py Episode.tick."""

    def __init__(self, ev, dc):
        super().__init__(daemon=True)
        self.ev, self.dc = ev, dc
        self.judge = GraspJudge(dc['judge'])
        self.dt = 1.0 / float(dc['control_hz'])
        self.phase = 'policy'
        self.armed, self.t_motor_off = True, None
        self.rows, self.err = [], None
        self.link_force_max = {}
        self._halt = threading.Event()

    def run(self):
        ev = self.ev
        try:
            next_t = ev.now()
            while not self._halt.is_set():
                while ev.now() < next_t and not self._halt.is_set():
                    time.sleep(0.002)          # same poll as the client loop (GIL: leave the spin threads room)
                next_t += self.dt
                self.armed = ev.drone_armed(self.armed, True)     # any kill (client or operator) counts
                if not self.armed and self.t_motor_off is None:
                    self.t_motor_off = ev.now()
                o, f = ev.judge_observation(ev.grip_cmd > 0.5 * self.dc['gripper_raw_max'], self.armed)
                for k, v in f.items():
                    if v > 1.0:
                        m = self.link_force_max.setdefault(self.phase, {})
                        m[k] = max(m.get(k, 0.0), v)
                self.judge.update(o)
                rel = o['tcp_R'].T @ (o['drone_p'] - o['tcp_p'])
                self.rows.append((o['t'], self.phase, self.judge.state, *o['tcp_p'], *o['drone_p'], np.degrees(o['grip_q']),
                                  o['f_right'], o['f_left'], o['f_other'], int(o['armed']), ev.grip_cmd, int(bool(ev.pstop)),
                                  *rel))
        except Exception as e:                       # noqa: BLE001  (reported by the main thread)
            self.err = e

    def stop(self):
        self._halt.set()
        self.join(timeout=5.0)


def wait_sim(ev, cond, max_s, poll=0.01):
    """Wait until cond() or max_s sim seconds. -> cond()."""
    t0 = ev.now()
    while not cond() and ev.now() - t0 < max_s:
        time.sleep(poll)
    return cond()


def write_csv(path, header, rows):
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f'{v:.5f}' if isinstance(v, (float, np.floating)) else v for v in r])


def policy_and_kill(args, pc, ev, cfg, dc, jl, stop_check, trace, log):
    """Policy run, then (if grasped with motors on) auto kill / operator wait, then the judge hold window."""
    res_c = pc.run_episode(args.duration, args.auto_kill, stop_check=stop_check, trace=trace)
    # policy over: no more commands, the arm holds its last target, the gripper its last command
    jl.phase = 'post'
    auto_killed = res_c['drone_killed']
    if jl.err is None and jl.judge.state == 'grasped' and jl.armed:
        if args.auto_kill is not None:
            r = pc.call(pc.cli_kill, cfg['topics']['kill_service'], 10.0)
            auto_killed = True
            log(f'  grasped and motors still on at the policy end -> motors off (--auto-kill): {r.success} {r.message[:80]}')
        elif args.operator_wait > 0:
            log(f'  grasped and motors still on at the policy end: waiting up to {args.operator_wait:g} s sim for the '
                f'operator (drone_cmd.py kill / ros2 service call /sim/drone_kill std_srvs/srv/Trigger)')
            wait_sim(ev, lambda: not jl.armed or jl.judge.done or jl.err is not None, args.operator_wait)
    if jl.err is None and jl.judge.state == 'grasped' and not jl.armed:
        wait_sim(ev, lambda: jl.judge.done or jl.err is not None, float(dc['judge']['hold_time']) + args.hold_margin)
    return res_c, auto_killed


def run_episode(k, args, pc, ev, cfg, dc, out_dir, log):
    w0 = time.monotonic()
    tag = f'ep{k:02d}'
    reset = None
    if not args.no_reset:
        reset = pc.reset()
        log(f"\n===== episode {k} =====  reset: drone base {reset.get('drone_pos_base')} ({reset.get('drone_target')}), "
            f"seed {reset.get('seed')}, reset count {reset.get('reset_count')}")
    else:
        log(f'\n===== episode {k} =====  (no reset)')
    # fixed drone: hanging on its anchor; PX4 drone: flying (released after the air start, armed)
    t0 = time.monotonic()
    while ev.drone is None or (ev.drone_state != 'attached' if ev.px4 is None else not ev.px4.get('armed')):
        if time.monotonic() - t0 > 30:
            raise RuntimeError('no drone odometry / drone not attached or armed 30 s after the reset')
        time.sleep(0.05)
    pc.wait_sim_until(pc.now() + 0.2)                 # fresh observations after the reset
    n_pstop0 = len(ev.pstop_events)
    jl = JudgeLoop(ev, dc)
    jl.start()

    def stop_check():
        if jl.err is not None:
            return 'judge_error'
        if args.stop_on_result and jl.judge.done:
            return f'judge_{jl.judge.state}'
        return None

    trace = []
    try:
        res_c, auto_killed = policy_and_kill(args, pc, ev, cfg, dc, jl, stop_check, trace, log)
    finally:
        jl.stop()
    if jl.err is not None:
        raise RuntimeError(f'judge loop failed: {jl.err!r}') from jl.err
    res_j = jl.judge.finish(ev.now())
    pstops = ev.pstop_events[n_pstop0:]
    t_start = res_c['detail']['t_start']
    ev_t = {s: round(t - t_start, 3) for t, s, _ in res_j['events']}
    row = {'episode': k, 'drone_target': (reset or {}).get('drone_target'), 'drone_pos_base': (reset or {}).get('drone_pos_base'),
           'success': res_j['success'], 'state': res_j['state'], 'reason': res_j['reason'], 'policy_stop': res_c['stop'],
           'policy_sim_s': res_c['sim_s'], 't_grasped': ev_t.get('grasped'),
           't_motor_off': None if jl.t_motor_off is None else round(jl.t_motor_off - t_start, 3),
           'chunks': res_c['chunks'], 'infer_wall_median_s': res_c['infer_wall_median_s'],
           'infer_wall_max_s': res_c['infer_wall_max_s'], 'chunk_arrive_step_median': res_c['chunk_arrive_step_median'],
           'held_frac': res_c['held_frac'], 'limited_frac': res_c['limited_frac'],
           'gripper_changes': len(res_c['detail']['gripper_events']), 'protective_stops': sum(1 for _, v in pstops if v),
           'auto_killed': auto_killed, 'wall_s': round(time.monotonic() - w0, 1)}
    for t, st, info in res_j['events']:
        log(f'  t {t - t_start:7.2f} s  {st:10s} {info}')
    log(f"  result {res_j['state']} ({res_j['reason']}), policy stop {res_c['stop']}, motors off at "
        f"{row['t_motor_off']} s, protective stops {pstops or 'none'}, slip max {res_j['max_slip_mm']} mm, "
        f"settle {res_j['settle_mm']} mm")
    full = {'summary': row, 'reset': reset, 'policy': res_c, 'judge': res_j, 'protective_stop_events': pstops,
            't_motor_off_sim': jl.t_motor_off,
            'link_force_max_N': {ph: {kk: round(v, 2) for kk, v in m.items()} for ph, m in jl.link_force_max.items()}}
    with open(os.path.join(out_dir, f'{tag}_result.json'), 'w', encoding='utf-8') as f:
        json.dump(full, f, ensure_ascii=False, indent=1, default=float)
    write_csv(os.path.join(out_dir, f'{tag}_timeline.csv'), ROW_KEYS, jl.rows)
    write_csv(os.path.join(out_dir, f'{tag}_commands.csv'), ('t', 'x_step', 'chunk', *pc.arm_joints, 'gripper_closed'), trace)
    return row


def summary_table(rows):
    cols = (('ep', 'episode', 3), ('target', 'drone_target', 14), ('result', 'state', 8), ('reason', 'reason', 18),
            ('policy stop', 'policy_stop', 16), ('grasp s', 't_grasped', 8), ('off s', 't_motor_off', 7),
            ('infer s', 'infer_wall_median_s', 7), ('held', 'held_frac', 5), ('limited', 'limited_frac', 7),
            ('grip chg', 'gripper_changes', 8), ('pstop', 'protective_stops', 5))
    lines = ['  '.join(f'{h:<{w}}' for h, _, w in cols)]
    for r in rows:
        lines.append('  '.join(f"{str('-' if r[k] is None else r[k])[:w]:<{w}}" for _, k, w in cols))
    n = len(rows)
    ok = sum(bool(r['success']) for r in rows)
    reasons = Counter('held' if r['success'] else str(r['reason']) for r in rows)
    lines.append(f'success {ok}/{n} ({100.0 * ok / max(n, 1):.0f} %), results {dict(reasons)}')
    return lines, {'episodes': n, 'success': ok, 'success_rate': round(ok / max(n, 1), 3), 'results': dict(reasons)}


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--episodes', type=int, default=1)
    a.add_argument('--duration', type=float, required=True, help='[sim s] policy run per episode')
    a.add_argument('--auto-kill', type=float, default=None,
                   help='[sim s] drone motors off this long after the close command while the gripper is blocked '
                        '(default: the operator does it)')
    a.add_argument('--operator-wait', type=float, default=15.0,
                   help='[sim s] without --auto-kill: wait this long after the policy end for the operator kill when '
                        'the drone is grasped and still on (0 = do not wait)')
    a.add_argument('--hold-margin', type=float, default=1.0, help='[sim s] waited beyond judge hold_time after motors off')
    a.add_argument('--stop-on-result', action='store_true', help='end the policy run as soon as the judge decides')
    a.add_argument('--no-reset', action='store_true', help='no /sim/reset (one episode from the current state)')
    a.add_argument('--config', default=None, help='policy_client.yaml (default: installed ocean_sim2sim_policy one)')
    a.add_argument('--judge-config', default=None,
                   help='grasp_demo.yaml with the judge section (default: installed ocean_sim2sim_bringup one)')
    a.add_argument('--host', default=None)
    a.add_argument('--port', type=int, default=None)
    a.add_argument('--prompt', default=None)
    a.add_argument('--report-dir', default='~/ocean_sim2sim_reports')
    a.add_argument('--base-link', default='robot_mount')
    a.add_argument('--tcp-link', default='rh_p12_rn_tcp')
    a.add_argument('--boat-link', default='wamv/base_link')
    args = a.parse_args()
    if args.no_reset and args.episodes > 1:
        a.error('--no-reset runs one episode')
    cfg_path = args.config or os.path.join(get_package_share_directory('ocean_sim2sim_policy'), 'config', 'policy_client.yaml')
    judge_path = args.judge_config or os.path.join(get_package_share_directory('ocean_sim2sim_bringup'), 'config',
                                                   'grasp_demo.yaml')
    cfg = load_config(cfg_path)
    if args.prompt is not None:
        cfg['prompt'] = args.prompt
    with open(judge_path) as f:
        gd = yaml.safe_load(f)
    dc = {k: gd[k] for k in ('judge', 'control_hz', 'gripper_upper', 'gripper_raw_max')}
    host, port = args.host or cfg['server']['host'], args.port or int(cfg['server']['port'])

    out_dir = os.path.join(os.path.expanduser(args.report_dir),
                           'policy_eval_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    report = open(os.path.join(out_dir, 'report.txt'), 'w', encoding='utf-8')

    def log(s):
        print(s, flush=True)
        report.write(s + '\n')
        report.flush()

    from openpi_client import websocket_client_policy
    log(f'[eval] connecting to the policy server {host}:{port} ...')
    policy = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
    meta = policy.get_server_metadata()
    log(f'[eval] server metadata: {meta}')
    rclpy.init()
    ev = EvalNode(dc, cfg['topics'])
    ex = EventsExecutor()
    ex.add_node(ev)

    def spin():
        try:
            ex.spin()
        except Exception:  # noqa: BLE001  shutdown at the end
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    pc = PolicyClient(cfg, policy, log=log)
    rows = []
    try:
        t0 = time.time()
        while any(ev.observation_missing().values()):
            if time.time() - t0 > 90:
                raise RuntimeError(f'judge inputs not ready within 90 s: {[k for k, v in ev.observation_missing().items() if v]}')
            time.sleep(0.1)
        ev.setup_kinematics(args.base_link, args.tcp_link, args.boat_link)
        pc.wait_ready()
        pc.warmup()
        with open(os.path.join(out_dir, 'config.json'), 'w', encoding='utf-8') as f:
            json.dump({'args': vars(args), 'argv': sys.argv, 'policy_config_file': cfg_path, 'policy_config': cfg,
                       'judge_config_file': judge_path, 'judge_config': dc, 'server': f'{host}:{port}',
                       'server_metadata': meta}, f, ensure_ascii=False, indent=1, default=str)
        log(f"policy_eval {datetime.datetime.now().isoformat(timespec='seconds')}  {args.episodes} episodes x "
            f"{args.duration:g} s, auto-kill {args.auto_kill}, cameras {cfg['cameras']}, blank {cfg['blank_images']}, "
            f"base_imu {cfg['base_imu']['enabled']}, prompt {cfg['prompt']!r}")
        for k in range(1, args.episodes + 1):
            rows.append(run_episode(k, args, pc, ev, cfg, dc, out_dir, log))
    except KeyboardInterrupt:
        log('[eval] interrupted (Ctrl+C)')
    finally:
        if rows:
            lines, tot = summary_table(rows)
            log('\n===== summary =====')
            for ln in lines:
                log(ln)
            with open(os.path.join(out_dir, 'summary.json'), 'w', encoding='utf-8') as f:
                json.dump({'total': tot, 'episodes': rows}, f, ensure_ascii=False, indent=1, default=float)
            write_csv(os.path.join(out_dir, 'summary.csv'), SUMMARY_KEYS, [[r[c] for c in SUMMARY_KEYS] for r in rows])
        log(f'report: {out_dir}')
        report.close()
        pc.shutdown()
        ex.shutdown()                                # stop the EventsExecutor thread before rclpy (else abort at exit)
        spin_thread.join(timeout=2.0)
        ev.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
