#!/usr/bin/env python3
"""One-command regression suite: each stage starts its own headless bringup, runs its checks and stops it. docs/regression.md

    ros2 run ocean_sim2sim_bringup regression.py                 # all stages (~10 min)
    ros2 run ocean_sim2sim_bringup regression.py --quick         # E + G3 x1 + pstop + demo success
    ros2 run ocean_sim2sim_bringup regression.py --only grip,pstop

Stages:
    grip   calm world, fixed drone: RTF idle, grasp_test.py E, G3 x2, G1, G4, H (0.8 kg)
    pstop  calm world, fixed drone: reset; shoulder_lift ramp into the boat -> stop by contact force; reset (escape pose)
           must succeed with the arm at init_pose; 1 rad wrist_1 jump -> stop; reset
    demo   calm world, PX4 drone (static): hover RTF, grasp_demo.py --case all, RTF samples, protective stop never trips
    waves  empty_ocean (gain 0.3), PX4 drone (hover): hover RTF only (the scripted demo cannot reach in waves)

Expected values / tolerances and their sources: config/regression.yaml. Results: --report-dir (default
~/ocean_sim2sim_reports)/regression_<time>/ report.txt, results.json, <stage>/ logs. Exit code 0 only if no check FAILs.
Needs the Python venv with pymavlink for the PX4 stages (README). Aborts if another Gazebo server is running.
"""
import argparse
import bisect
import csv
import datetime
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import yaml

STAGES = ('grip', 'pstop', 'demo', 'waves')
ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ANSI = re.compile(r'\x1b\[[0-9;]*m')


# ====================================================================================================== process helpers
def read_argv(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return []


def own_ancestors():
    """This process and its parents (never stopped by the sweep)."""
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def is_sim_process(argv):
    """Anchored match on argv (not a substring of the whole command line, so no shell running this script matches)."""
    if not argv:
        return False
    a0, a1 = argv[0], argv[1] if len(argv) > 1 else ''
    base = os.path.basename(a0)
    if ' '.join(argv).startswith('gz sim'):
        return True
    if base == 'ruby' and 'gz_tools_vendor' in a1:
        return True
    if a0.endswith('/ros_gz_bridge/parameter_bridge') or a0.endswith('/robot_state_publisher') or a0.endswith('/bin/px4'):
        return True
    if base.startswith('python3') and re.search(r'/(px4_bridge_node(\.py)?|interface_node|scene_node|spawner|spawn_controllers\.py)$', a1):
        return True
    return False


def gazebo_servers():
    return [int(p) for p in os.listdir('/proc') if p.isdigit() and ' '.join(read_argv(p)).startswith('gz sim')]


def sim_processes():
    skip = own_ancestors()
    return [int(p) for p in os.listdir('/proc') if p.isdigit() and int(p) not in skip and is_sim_process(read_argv(p))]


def kill_group(proc, first=signal.SIGINT, timeout=20.0):
    """Stop a process started with start_new_session=True: its whole process group, SIGINT -> SIGTERM -> SIGKILL."""
    if proc is None:
        return
    for sig, wait in ((first, timeout), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=wait)
            # the leader is gone; give the rest of the group a moment, then go on with the next signal if any remain
            time.sleep(0.5)
            if not group_alive(proc.pid):
                break
        except subprocess.TimeoutExpired:
            continue


def group_alive(pgid):
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                if os.getpgid(int(p)) == pgid:
                    return True
            except (ProcessLookupError, PermissionError):
                pass
    return False


def sweep(log):
    """Stop leftover simulation processes (anchored patterns): SIGINT, then SIGTERM, then SIGKILL."""
    for sig, wait in ((signal.SIGINT, 5.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        left = sim_processes()
        if not left:
            return []
        log(f'  sweep {sig.name}: {[(p, " ".join(read_argv(p))[:80]) for p in left]}')
        for p in left:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        t0 = time.time()
        while time.time() - t0 < wait and sim_processes():
            time.sleep(0.2)
    return sim_processes()


def run_logged(cmd, log_path, timeout, env=None):
    """Run cmd in its own session, output to log_path. Returns (returncode, timed_out)."""
    with open(log_path, 'a', encoding='utf-8') as f:
        f.write(f'$ {" ".join(cmd)}\n')
        f.flush()
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        try:
            return p.wait(timeout=timeout), False
        except subprocess.TimeoutExpired:
            kill_group(p, signal.SIGTERM, 10.0)
            return p.returncode, True
        except BaseException:
            kill_group(p, signal.SIGTERM, 10.0)
            raise


def worker_cmd(*args):
    return [sys.executable, os.path.abspath(__file__), '--worker', *args]


# ====================================================================================================== workers (rclpy)
def worker_rtf(out, duration, window):
    """Sample RTF = d(sim time on /clock) / d(wall time) every `window` wall seconds, and count /protective_stop true.
    Writes one JSON line per window to `out`; runs for `duration` s (0 = until SIGINT / SIGTERM)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool

    rclpy.init()
    n = rclpy.create_node('regression_rtf')
    st = {'sim': None, 'wall': None, 'ps': 0, 'ps_msgs': 0}

    def on_clock(m):
        st['sim'], st['wall'] = m.clock.sec + m.clock.nanosec * 1e-9, time.time()

    def on_ps(m):
        st['ps_msgs'] += 1
        st['ps'] += int(m.data)

    n.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    n.create_subscription(Bool, '/protective_stop', on_ps, 10)
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    t_end = time.time() + duration if duration > 0 else float('inf')
    last = None
    next_t = time.time() + window
    with open(out, 'a', encoding='utf-8') as f:
        while not stop['v'] and time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() < next_t:
                continue
            next_t += window
            if st['sim'] is None:
                continue
            cur = (st['wall'], st['sim'])
            if last is not None and cur[0] > last[0]:
                f.write(json.dumps({'wall0': last[0], 'wall1': cur[0], 'sim0': last[1], 'sim1': cur[1],
                                    'rtf': round((cur[1] - last[1]) / (cur[0] - last[0]), 4),
                                    'ps_true': st['ps'], 'ps_msgs': st['ps_msgs']}) + '\n')
                f.flush()
            last = cur
    n.destroy_node()
    rclpy.shutdown()


def worker_pstop(out, cfg):
    """Protective stop sequence through the public interface (/joint_command, /sim/reset, /protective_stop)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    rclpy.init()

    class W(Node):
        def __init__(self):
            super().__init__('regression_pstop', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.q, self.ps, self.ps_wall = {}, None, 0.0
            self.create_subscription(JointState, '/joint_state_broadcaster/joint_states',
                                     lambda m: self.q.update(zip(m.name, m.position)), 50)
            self.create_subscription(Bool, '/protective_stop', self._on_ps, 10)
            self.pub = self.create_publisher(JointState, '/joint_command', 10)
            self.reset_cli = self.create_client(Trigger, '/sim/reset')

        def _on_ps(self, m):
            self.ps, self.ps_wall = m.data, time.time()

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

    n = W()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    res = {}

    def log(s):
        print(s, flush=True)

    def wait_until(cond, wall_timeout):
        t0 = time.time()
        while time.time() - t0 < wall_timeout:
            if cond():
                return True
            time.sleep(0.02)
        return False

    def sim_sleep(sec, wall_timeout=120.0):
        t0 = n.now()
        wait_until(lambda: n.now() - t0 >= sec, wall_timeout)

    def reset():
        t0 = time.time()
        while True:
            if not n.reset_cli.wait_for_service(timeout_sec=30.0):
                return False, {'error': '/sim/reset not available'}
            fut = n.reset_cli.call_async(Trigger.Request())
            if not wait_until(fut.done, 180.0):
                return False, {'error': '/sim/reset timed out (180 s)'}
            r = fut.result()
            try:
                info = json.loads(r.message)
            except ValueError:
                info = {'message': r.message}
            if r.success or 'not ready' not in r.message or time.time() - t0 > 300:
                return r.success, info
            time.sleep(2.0)

    def arm_off(init_pose):
        return max(abs(n.q[j] - q) for j, q in zip(ARM, init_pose))

    def send(q):
        m = JointState()
        m.header.stamp = n.get_clock().now().to_msg()
        m.name, m.position = list(ARM), [float(v) for v in q]
        n.pub.publish(m)

    def after_reset(key):
        ok, info = reset()
        res[key] = ok
        res[key + '_info'] = info
        log(f'{key}: success {ok} {info}')
        if not ok:
            return False
        sim_sleep(0.5)
        init = info.get('init_pose') or [n.q[j] for j in ARM]
        res[key + '_pose_off_rad'] = round(arm_off(init), 4)
        res[key + '_pose'] = res[key + '_pose_off_rad'] <= cfg['pose_tol']
        # /protective_stop comes on every interface loop tick: wait for a fresh false
        t_ask = time.time()
        res[key + '_cleared'] = wait_until(lambda: n.ps_wall > t_ask and n.ps is False, 5.0)
        log(f'  arm off init_pose {res[key + "_pose_off_rad"]} rad, /protective_stop cleared {res[key + "_cleared"]}')
        return True

    def drive(target_fn, sim_limit, wall_limit=300.0):
        """Publish target_fn(elapsed sim s) at command_hz until /protective_stop true or sim_limit. -> (stopped, elapsed)."""
        t0, w0 = n.now(), time.time()
        period = 1.0 / cfg['command_hz']
        while True:
            el = n.now() - t0
            if n.ps:
                return True, el
            if el > sim_limit or time.time() - w0 > wall_limit:
                return False, el
            send(target_fn(el))
            time.sleep(period)

    try:
        ok = wait_until(lambda: len(n.q) >= 6 and n.ps is not None and n.pub.get_subscription_count() > 0, 120.0)
        if not ok:
            raise RuntimeError('no joint states / protective_stop / interface subscriber within 120 s')
        if not after_reset('reset_initial'):
            raise RuntimeError('initial reset failed')
        # 1. crash: ramp one joint from the start pose until the protective stop
        k = ARM.index(cfg['ramp_joint'])
        q0 = [n.q[j] for j in ARM]
        rate, qmax = cfg['ramp_rate'], cfg['ramp_max']

        def ramp(el):
            q = list(q0)
            q[k] += min(rate * el, qmax)
            return q

        stopped, el = drive(ramp, qmax / rate + 3.0)
        res['crash_stopped'], res['crash_elapsed_s'] = stopped, round(el, 3)
        res['crash_delta_rad'] = round(n.q[ARM[k]] - q0[k], 4)
        log(f'crash ramp {cfg["ramp_joint"]}: stopped {stopped} after {el:.2f} s sim, moved {res["crash_delta_rad"]} rad')
        sim_sleep(1.0)
        after_reset('crash_reset')
        # 2. joint jump
        k = ARM.index(cfg['jump_joint'])
        q0 = [n.q[j] for j in ARM]
        tgt = list(q0)
        tgt[k] += cfg['jump']
        stopped, el = drive(lambda _el: tgt, 3.0)
        res['jump_stopped'], res['jump_elapsed_s'] = stopped, round(el, 3)
        log(f'jump {cfg["jump_joint"]} {cfg["jump"]:+} rad: stopped {stopped} after {el:.3f} s sim')
        sim_sleep(1.0)
        after_reset('jump_reset')
    except Exception as e:  # noqa: BLE001  reported as a FAIL by the suite
        res['error'] = str(e)
        log(f'ERROR: {e}')
    finally:
        with open(out, 'w', encoding='utf-8') as f:
            json.dump(res, f, indent=1, default=str)
        ex.shutdown()
        th.join(timeout=2.0)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ====================================================================================================== suite
class Suite:
    def __init__(self, args, cfg, out_dir):
        self.args, self.cfg, self.out_dir = args, cfg, out_dir
        self.items = []                 # dicts: stage, name, status, value, expected, source, note
        self.stage_info = {}
        self.launch = None
        self.launch_log = None
        self.main_log = open(os.path.join(out_dir, 'suite.log'), 'a', encoding='utf-8')
        self.env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0',
                        RCUTILS_LOGGING_BUFFERED_STREAM='0')

    def log(self, s):
        line = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] {s}'
        print(line, flush=True)
        self.main_log.write(line + '\n')
        self.main_log.flush()

    # ---------------------------------------------------------------- checks
    def check(self, stage, key, value, note=''):
        c = self.cfg['checks'][f'{stage}.{key}']
        if c.get('info'):
            status = 'INFO'
        elif value is None:
            status = 'FAIL'
        elif 'equals' in c:
            status = 'PASS' if value == c['equals'] else 'FAIL'
        else:
            try:
                v = float(value)
                ok = not math.isnan(v) and ('min' not in c or v >= c['min']) and ('max' not in c or v <= c['max'])
            except (TypeError, ValueError):
                ok = False
            status = 'PASS' if ok else 'FAIL'
        lim = ''
        if 'min' in c or 'max' in c:
            lim = f" [{c.get('min', '')}..{c.get('max', '')}]"
        self.items.append({'stage': stage, 'name': key, 'status': status, 'value': value,
                           'expected': f"{c.get('expected', '')}{lim}", 'source': c.get('source', ''), 'note': note})
        self.log(f'  {status:4s} {stage}.{key} = {fmt(value)} {note}  (expected {c.get("expected", "")}{lim})')

    def fail(self, stage, name, msg):
        self.items.append({'stage': stage, 'name': name, 'status': 'FAIL', 'value': None, 'expected': 'runs',
                           'source': '', 'note': msg})
        self.log(f'  FAIL {stage}.{name}: {msg}')

    # ---------------------------------------------------------------- bringup
    def start_launch(self, stage, scfg):
        if gazebo_servers():
            raise Abort(f'another Gazebo server is running (pids {gazebo_servers()}): stop it first '
                        '(two servers corrupt the results)')
        left = sim_processes()
        if left:
            raise Abort(f'leftover simulation processes are running: '
                        f'{[(p, " ".join(read_argv(p))[:80]) for p in left]}')
        lc = self.cfg['launch']
        largs = dict(lc.get('common_args', {}), **scfg.get('launch_args', {}))
        cmd = ['ros2', 'launch', lc['package'], lc['file']] + [f'{k}:={v}' for k, v in largs.items()]
        self.launch_log = os.path.join(self.out_dir, stage, 'launch.log')
        self.log(f'[{stage}] launch: {" ".join(cmd)}')
        f = open(self.launch_log, 'w', encoding='utf-8')
        self.launch = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=self.env)
        f.close()
        t0 = time.time()
        found_log = active = False
        while time.time() - t0 < lc['ready_timeout_s']:
            time.sleep(3.0)
            if self.launch.poll() is not None:
                raise StageError(f'launch exited (code {self.launch.returncode}) before ready, see {self.launch_log}')
            if not found_log:
                found_log = scfg['ready_log'] in self.launch_text()
            if found_log and not active:
                active = self.controllers_active() >= lc['controllers_active']
            if found_log and active:
                self.log(f'[{stage}] ready after {time.time() - t0:.0f} s')
                self.stage_info.setdefault(stage, {})['ready_s'] = round(time.time() - t0, 1)
                return
        raise StageError(f'not ready within {lc["ready_timeout_s"]} s (log "{scfg["ready_log"]}" {found_log}, '
                         f'controllers active {active}), see {self.launch_log}')

    def controllers_active(self):
        try:
            r = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=20,
                               env=self.env)
        except subprocess.TimeoutExpired:
            return 0
        return sum(1 for line in ANSI.sub('', r.stdout).splitlines() if line.split() and line.split()[-1] == 'active')

    def launch_text(self):
        try:
            with open(self.launch_log, encoding='utf-8', errors='replace') as f:
                return ANSI.sub('', f.read())
        except OSError:
            return ''

    def stop_launch(self):
        if self.launch is None:
            return
        self.log('  stopping the bringup')
        kill_group(self.launch, signal.SIGINT, self.cfg['launch']['shutdown_timeout_s'])
        self.launch = None
        left = sweep(self.log)
        if left:
            self.log(f'  WARNING: processes still running after the sweep: {left}')
        t0 = time.time()
        while gazebo_servers() and time.time() - t0 < 20:
            time.sleep(0.5)

    def protective_stops(self):
        """[(t, reason)] from the interface log lines in the launch log."""
        return [(float(t), r) for t, r in
                re.findall(r'PROTECTIVE STOP at t ([0-9.]+) s: (.*?)\. Arm held', self.launch_text())]

    def rtf(self, stage, name, seconds):
        path = os.path.join(self.out_dir, stage, f'rtf_{name}.jsonl')
        run_logged(worker_cmd('rtf', '--out', path, '--duration', str(seconds), '--window', '1.0'),
                   os.path.join(self.out_dir, stage, 'workers.log'), seconds + 60, self.env)
        return load_jsonl(path)

    # ---------------------------------------------------------------- stages
    def stage_grip(self, s):
        samples = self.rtf('grip', 'idle', s['rtf_idle_s'])
        self.check('grip', 'rtf_idle', median([x['rtf'] for x in samples]), f'{len(samples)} samples')
        tests = s['grasp_tests_quick'] if self.args.quick else s['grasp_tests']
        log_path = os.path.join(self.out_dir, 'grip', 'grasp_test.log')
        cmd = ['ros2', 'run', 'ocean_sim2sim_bringup', 'grasp_test.py', '--world', s['world'], '--mass', str(s['mass']),
               '--tests', *tests]
        self.log(f'[grip] {" ".join(cmd)}')
        rc, to = run_logged(cmd, log_path, s['timeout_s'], self.env)
        if rc != 0 or to:
            self.fail('grip', 'grasp_test', f'exit code {rc}, timed out {to} (see {log_path})')
        with open(log_path, encoding='utf-8', errors='replace') as f:
            r = parse_grasp_test(f.read())
        self.stage_info['grip']['grasp_test'] = r
        if 'E' in tests:
            self.check('grip', 'E_r1_rad', r.get('E_r1'))
        for i in range(tests.count('G3')):
            g = r['G3'][i] if i < len(r['G3']) else None
            tag = f'G3 #{i + 1}'
            self.check('grip', 'G3_slip_force_N', g and g.get('slip_force'), tag)
            fn = (g or {}).get('normal', {})
            big = {k: v for k, v in fn.items() if v > 1.0}
            c = self.cfg['checks']['grip.G3_normal_force_N']
            # every finger link in contact must be in range, and at least 2 (one per side)
            val = min(big.values()) if len(big) >= 2 else None
            if big and val is not None and max(big.values()) > c['max']:
                val = max(big.values())
            self.check('grip', 'G3_normal_force_N', val, f'{tag}: ' + ' '.join(f'{k} {v:.1f}' for k, v in sorted(big.items())))
        for t in ('G1', 'G4', 'H'):
            if t not in tests:
                continue
            h = r.get(t)
            self.check('grip', f'{t}_slip_mm', h and h['slip'])
            if t == 'G1':
                self.check('grip', 'G1_box_jitter_mm', h and h['box_jitter'])
            if t == 'G4':
                self.check('grip', 'G4_joint_jitter_deg', h and h['joint_jitter'])
                self.check('grip', 'G4_box_jitter_mm', h and h['box_jitter'])
            if t == 'H':
                self.check('grip', 'H_box_jitter_mm', h and h['box_jitter'], f'{s["mass"]} kg')

    def stage_pstop(self, s):
        out = os.path.join(self.out_dir, 'pstop', 'pstop.json')
        wcfg = {k: s[k] for k in ('ramp_joint', 'ramp_rate', 'ramp_max', 'jump_joint', 'jump', 'command_hz', 'pose_tol')}
        rc, to = run_logged(worker_cmd('pstop', '--out', out, '--pstop-config', json.dumps(wcfg)),
                            os.path.join(self.out_dir, 'pstop', 'pstop.log'), s['timeout_s'], self.env)
        r = load_json(out) or {}
        self.stage_info['pstop']['worker'] = r
        if rc != 0 or to or 'error' in r:
            self.fail('pstop', 'worker', f'exit code {rc}, timed out {to}, error {r.get("error")}')
        stops = self.protective_stops()
        self.stage_info['pstop']['stops'] = stops
        crash = stops[0][1] if r.get('crash_stopped') and stops else None
        jump = stops[1][1] if r.get('jump_stopped') and len(stops) > 1 else None
        self.check('pstop', 'reset_initial', bool(r.get('reset_initial') and r.get('reset_initial_pose')))
        self.check('pstop', 'crash_stopped', r.get('crash_stopped'),
                   f'{s["ramp_joint"]} moved {r.get("crash_delta_rad")} rad in {r.get("crash_elapsed_s")} s')
        self.check('pstop', 'crash_by_force', bool(crash and re.search(r'(^|; )force ', crash)), crash or 'no stop log line')
        m = re.search(r'force ([0-9.]+) N \(([^)]*)\)', crash or '')
        self.check('pstop', 'crash_force_N', float(m.group(1)) if m else None, m.group(2) if m else '')
        self.check('pstop', 'crash_reset', r.get('crash_reset'), str(r.get('crash_reset_info', ''))[:100]
                   if not r.get('crash_reset') else '')
        self.check('pstop', 'crash_reset_pose', r.get('crash_reset_pose'), f'off {r.get("crash_reset_pose_off_rad")} rad')
        self.check('pstop', 'crash_cleared', r.get('crash_reset_cleared'))
        self.check('pstop', 'jump_stopped', r.get('jump_stopped'), f'after {r.get("jump_elapsed_s")} s')
        self.check('pstop', 'jump_reason', jump)
        self.check('pstop', 'jump_reset', bool(r.get('jump_reset') and r.get('jump_reset_pose')
                                                and r.get('jump_reset_cleared')),
                   f'off {r.get("jump_reset_pose_off_rad")} rad, cleared {r.get("jump_reset_cleared")}')
        extra = stops[2:] if len(stops) > 2 else []
        if extra:
            self.log(f'  note: more protective stops than expected: {extra}')

    def stage_demo(self, s):
        samples = self.rtf('demo', 'static_hover', s['rtf_hover_s'])
        self.check('demo', 'rtf_static_hover', median([x['rtf'] for x in samples]), f'{len(samples)} samples')
        cases = s['cases_quick'] if self.args.quick else s['cases']
        d = os.path.join(self.out_dir, 'demo')
        rtf_path = os.path.join(d, 'rtf_demo.jsonl')
        sampler = subprocess.Popen(worker_cmd('rtf', '--out', rtf_path, '--duration', '0', '--window', '1.0'),
                                   stdout=open(os.path.join(d, 'workers.log'), 'a'), stderr=subprocess.STDOUT,
                                   start_new_session=True, env=self.env)
        try:
            cmd = ['ros2', 'run', 'ocean_sim2sim_bringup', 'grasp_demo.py', '--case', *cases, '--report-dir', d]
            self.log(f'[demo] {" ".join(cmd)}')
            rc, to = run_logged(cmd, os.path.join(d, 'grasp_demo.log'), s['timeout_s'], self.env)
        finally:
            kill_group(sampler, signal.SIGINT, 10.0)
        if rc != 0 or to:
            self.fail('demo', 'grasp_demo', f'exit code {rc}, timed out {to} (see {d}/grasp_demo.log)')
        runs = sorted(glob.glob(os.path.join(d, 'grasp_demo_*')))
        rd = runs[-1] if runs else None
        names = ('success', 'offset_y', 'offset_x', 'empty', 'miss') if 'all' in cases else cases
        holding = []
        for c in names:
            res = load_json(os.path.join(rd, f'result_{c}.json')) if rd else None
            if res is None:
                self.check('demo', c, None, 'no result file')
                continue
            note = (f"{res['state']}/{res['reason']}, abort {res.get('abort')}, slip {res.get('max_slip_mm')} mm, "
                    f"rot {res.get('max_rot_deg')} deg, push {res.get('push_max_mm')} mm")
            self.check('demo', c, bool(res.get('as_expected')), note)
            if c == 'success':
                self.check('demo', 'success_rot_deg', res.get('max_rot_deg'))
            holding += holding_times(os.path.join(rd, f'timeline_{c}.csv'))
        samples = load_jsonl(rtf_path)
        ps_true = samples[-1]['ps_true'] if samples else None
        stops = self.protective_stops()
        self.check('demo', 'protective_stop_never', ps_true == 0 and not stops,
                   f'/protective_stop true msgs {ps_true}, stop log lines {len(stops)}'
                   + (f': {stops[0][1]}' if stops else ''))
        holding.sort()
        hold_s = [x['rtf'] for x in samples if any_in(holding, x['sim0'], x['sim1'])]
        other = [x['rtf'] for x in samples if not any_in(holding, x['sim0'], x['sim1'])]
        self.check('demo', 'rtf_holding', median(hold_s), f'{len(hold_s)} samples, min {fmt(min(hold_s) if hold_s else None)}')
        self.check('demo', 'rtf_not_holding', median(other), f'{len(other)} samples (incl. resets), '
                                                              f'min {fmt(min(other) if other else None)}')
        self.stage_info['demo']['report_dir'] = rd

    def stage_waves(self, s):
        samples = self.rtf('waves', 'hover', s['rtf_hover_s'])
        self.check('waves', 'rtf_hover', median([x['rtf'] for x in samples]),
                   f'{len(samples)} samples, min {fmt(min((x["rtf"] for x in samples), default=None))}')
        ps_true = samples[-1]['ps_true'] if samples else None
        stops = self.protective_stops()
        self.check('waves', 'protective_stop_never', ps_true == 0 and not stops,
                   f'/protective_stop true msgs {ps_true}, stop log lines {len(stops)}')

    def run_stage(self, stage):
        s = self.cfg['stages'][stage]
        os.makedirs(os.path.join(self.out_dir, stage), exist_ok=True)
        t0 = time.time()
        self.stage_info[stage] = {}
        self.log(f'===== stage {stage} =====')
        try:
            self.start_launch(stage, s)
            getattr(self, f'stage_{stage}')(s)
        except StageError as e:
            self.fail(stage, 'stage', str(e))
        except Abort:
            raise
        except Exception as e:  # noqa: BLE001  a suite bug or an unexpected output: report it, go on
            self.fail(stage, 'stage', f'{type(e).__name__}: {e}')
        finally:
            self.stop_launch()
            self.stage_info[stage]['duration_s'] = round(time.time() - t0, 1)
            self.log(f'===== stage {stage} done in {(time.time() - t0) / 60:.1f} min =====')


class StageError(Exception):
    pass


class Abort(Exception):
    pass


# ====================================================================================================== parsing
def parse_grasp_test(text):
    """grasp_test.py stdout -> {'E_r1': rad, 'G3': [{slip_force, normal{link: N}}], 'G1'|'G4'|'H': {slip, box_jitter,
    joint_jitter}}"""
    r = {'G3': []}
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.startswith('E empty close') and i + 1 < len(lines):
            m = re.search(r'rh_r1_joint\s+([-0-9.]+) rad', lines[i + 1])
            if m:
                r['E_r1'] = float(m.group(1))
        elif line.startswith('G3 pull:'):
            r['G3'].append({'slip_force': None, 'normal': {}})
        elif r['G3'] and 'contact normal force per finger link' in line:
            r['G3'][-1]['normal'] = {k: float(v) for k, v in re.findall(r'(rh_p12_rn_\w+) ([-0-9.]+)', line.split(':', 1)[1])}
        elif r['G3'] and line.strip().startswith('slips (1 mm)'):
            m = re.search(r'slip force ([-0-9.]+) N', line)
            r['G3'][-1]['slip_force'] = float(m.group(1)) if m else None
        else:
            m = re.match(r'(G1|G4|H): .*slip ([0-9.]+) mm, box jitter ([0-9.]+) mm, joint jitter ([0-9.]+) deg', line)
            if m:
                r[m.group(1)] = {'slip': float(m.group(2)), 'box_jitter': float(m.group(3)),
                                 'joint_jitter': float(m.group(4))}
    return r


def holding_times(path):
    """Sim times of a grasp_demo timeline where both fingers touch the drone (> 1 N)."""
    out = []
    try:
        with open(path, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                if float(row['f_right']) > 1.0 and float(row['f_left']) > 1.0:
                    out.append(float(row['t']))
    except (OSError, KeyError, ValueError):
        pass
    return out


def any_in(sorted_list, a, b):
    i = bisect.bisect_left(sorted_list, a)
    return i < len(sorted_list) and sorted_list[i] <= b


def load_json(path):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_jsonl(path):
    out = []
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def median(v):
    v = sorted(x for x in v if x is not None)
    if not v:
        return None
    m = len(v) // 2
    return round(v[m] if len(v) % 2 else (v[m - 1] + v[m]) / 2, 4)


def fmt(v):
    if isinstance(v, float):
        return f'{v:.4g}'
    return str(v)


def table(items):
    rows = [('stage', 'item', 'status', 'value', 'expected', 'note')]
    rows += [(i['stage'], i['name'], i['status'], fmt(i['value']), i['expected'], i['note']) for i in items]
    w = [min(max(len(r[k]) for r in rows), 60) for k in range(5)]
    lines = []
    for n, r in enumerate(rows):
        lines.append('  '.join(r[k][:60].ljust(w[k]) for k in range(5)) + '  ' + r[5])
        if n == 0:
            lines.append('-' * (sum(w) + 12))
    return '\n'.join(lines)


# ====================================================================================================== main
def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--quick', action='store_true', help='E + G3 x1 + pstop + demo success (no waves)')
    a.add_argument('--only', default=None, help=f'comma-separated stages out of {",".join(STAGES)}')
    a.add_argument('--config', default=None, help='expected values (default: <share>/ocean_sim2sim_bringup/config/regression.yaml)')
    a.add_argument('--report-dir', default='~/ocean_sim2sim_reports')
    # internal: the rclpy workers run as separate processes (own timeout, no rclpy state in the suite)
    a.add_argument('--worker', choices=('rtf', 'pstop'), help=argparse.SUPPRESS)
    a.add_argument('--out', help=argparse.SUPPRESS)
    a.add_argument('--duration', type=float, default=10.0, help=argparse.SUPPRESS)
    a.add_argument('--window', type=float, default=1.0, help=argparse.SUPPRESS)
    a.add_argument('--pstop-config', help=argparse.SUPPRESS)
    args = a.parse_args()

    if args.worker == 'rtf':
        return worker_rtf(args.out, args.duration, args.window)
    if args.worker == 'pstop':
        return worker_pstop(args.out, json.loads(args.pstop_config))

    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = os.path.join(get_package_share_directory('ocean_sim2sim_bringup'), 'config', 'regression.yaml')
    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    stages = [s.strip() for s in args.only.split(',')] if args.only else list(STAGES)
    bad = [s for s in stages if s not in STAGES]
    if bad:
        sys.exit(f'unknown stage(s) {bad}, choose from {STAGES}')
    if args.quick and not args.only:
        stages = ['grip', 'pstop', 'demo']
    stages = [s for s in STAGES if s in stages]

    if gazebo_servers():
        sys.exit(f'ABORT: a Gazebo server is already running (pids {gazebo_servers()}). Stop it first: two servers '
                 'corrupt the results.')
    if any(cfg['stages'][s]['launch_args'].get('drone_mode') == 'px4' for s in stages):
        r = subprocess.run(['python3', '-c', 'import pymavlink'], capture_output=True)
        if r.returncode != 0:
            sys.exit('ABORT: the PX4 stages need pymavlink in python3 (activate the venv, README)')

    out_dir = os.path.join(os.path.expanduser(args.report_dir),
                           'regression_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    suite = Suite(args, cfg, out_dir)
    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)       # SIGTERM -> the same clean teardown as Ctrl+C
    t0 = time.time()
    suite.log(f'regression: stages {stages}, quick {args.quick}, config {args.config}, report {out_dir}')
    aborted = None
    try:
        for st in stages:
            suite.run_stage(st)
    except Abort as e:
        aborted = str(e)
        suite.log(f'ABORT: {e}')
    except KeyboardInterrupt:
        aborted = 'interrupted'
        suite.log('ABORT: interrupted')
    finally:
        suite.stop_launch()
        sweep(suite.log)
    if aborted:
        suite.items.append({'stage': '-', 'name': 'suite', 'status': 'FAIL', 'value': None, 'expected': 'runs',
                            'source': '', 'note': aborted})
    total = time.time() - t0
    n_fail = sum(i['status'] == 'FAIL' for i in suite.items)
    n_pass = sum(i['status'] == 'PASS' for i in suite.items)
    n_info = sum(i['status'] == 'INFO' for i in suite.items)
    summary = (f'{"PASS" if n_fail == 0 else "FAIL"}: {n_pass} pass, {n_fail} fail, {n_info} info; '
               f'total {total / 60:.1f} min; stages ' + ', '.join(
                   f'{s} {suite.stage_info.get(s, {}).get("duration_s", 0) / 60:.1f} min' for s in stages))
    txt = (f'regression {datetime.datetime.now().isoformat(timespec="seconds")}  stages {stages}  quick {args.quick}\n'
           f'config {args.config}\n\n{table(suite.items)}\n\n{summary}\n'
           + ''.join(f'\n{i["stage"]}.{i["name"]}: {i["source"]}' for i in suite.items if i['source']) + '\n')
    with open(os.path.join(out_dir, 'report.txt'), 'w', encoding='utf-8') as f:
        f.write(txt)
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as f:
        json.dump({'stages': stages, 'quick': args.quick, 'config': args.config, 'total_s': round(total, 1),
                   'pass': n_fail == 0, 'aborted': aborted, 'items': suite.items, 'stage_info': suite.stage_info},
                  f, indent=1, default=str, ensure_ascii=False)
    print('\n' + table(suite.items) + '\n\n' + summary + f'\nreport: {out_dir}', flush=True)
    sys.exit(0 if n_fail == 0 else 1)


if __name__ == '__main__':
    main()
