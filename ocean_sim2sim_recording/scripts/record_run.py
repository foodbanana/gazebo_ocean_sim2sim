#!/usr/bin/env python3
# =============================================================
# record_run.py - one boat motion recording run in one command (docs/recording.md)
#
#   1. checks that no simulation is running, starts wamv_arm.launch.py headless with drone_mode none and the boat
#      ground truth (config/recording.yaml launch.args) and the wave arguments
#   2. waits for "scene ready", /sim/reset (arm to the init pose, gripper open), settle time
#   3. ros2 bag record (mcap, --use-sim-time) of config topics -> <run>/bag
#   4. arm_motion_node.py --mode fixed | scripted for --duration sim seconds (B5 / B8 in the bag)
#   5. stops the bag, the arm node and the simulation; RTF once per wall second -> <run>/rtf.jsonl
#   6. <run>/meta.json (spec C + E3 mounting + E6 waves), then the 100 Hz table (data_100hz.npz / .csv) and
#      e1_timing.json (E1)
#
#   Run directory: <out>/<YYYYMMDD_HHMMSS>_<name>_<arm>/ ; default out = ocean_gazebo_recordings/ at the top of
#   this repository (git-ignored), else ~/ocean_gazebo_recordings.
#
#       ros2 run ocean_sim2sim_recording record_run.py gain03_dir0 --arm fixed --duration 900
#       ros2 run ocean_sim2sim_recording record_run.py gain03_dir0 --arm scripted --seed 1 --duration 900
#       ros2 run ocean_sim2sim_recording record_run.py calm_test --world empty_ocean_calm --arm fixed --duration 60
#   Wave arguments (empty = world file value): --wave-gain 0.3 --wave-period 5 --wave-direction-deg 45
#   The Python venv is not needed (no PX4). Exit code 0 = run valid (no protective stop, table written).
# =============================================================

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from ocean_sim2sim_recording import convert, meta
from ocean_sim2sim_recording.proc import gazebo_servers, kill_group, read_argv, sim_processes, sweep


class Watch(Node):
    """Sim clock, protective stop and robot_description of the running simulation."""

    def __init__(self):
        super().__init__('record_run_watch')
        self.sim, self.pstop, self.urdf = None, False, None
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim', m.clock.sec + m.clock.nanosec * 1e-9), 10)
        self.create_subscription(Bool, '/protective_stop', lambda m: setattr(self, 'pstop', self.pstop or m.data), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data), latched)
        self.reset_cli = self.create_client(Trigger, '/sim/reset')


class Run:
    def __init__(self, args, cfg):
        self.a, self.cfg = args, cfg
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        root = meta.repo_root()
        out = args.out_dir or (os.path.join(root, 'ocean_gazebo_recordings') if root
                               else os.path.expanduser('~/ocean_gazebo_recordings'))
        self.dir = os.path.join(out, f'{stamp}_{args.name}_{args.arm}')
        os.makedirs(self.dir)
        self.launch = self.bag = self.arm = None
        self.logf = open(os.path.join(self.dir, 'record_run.log'), 'a', encoding='utf-8')
        self.info = {'started_wall': datetime.now().isoformat(timespec='seconds')}

    def log(self, msg):
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        self.logf.write(line + '\n')
        self.logf.flush()

    # ------------------------------------------------------------------ processes
    def popen(self, cmd, name):
        self.log(f'$ {" ".join(cmd)}')
        f = open(os.path.join(self.dir, f'{name}.log'), 'w', encoding='utf-8')
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        f.close()
        return p

    def text(self, name):
        try:
            with open(os.path.join(self.dir, f'{name}.log'), encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError:
            return ''

    def start_sim(self):
        if gazebo_servers() or sim_processes():
            raise RuntimeError('a simulation is already running '
                               f'{[(p, " ".join(read_argv(p))[:80]) for p in sim_processes()]}: stop it first')
        lc = self.cfg['launch']
        largs = dict(lc['args'], world=self.a.world)
        if self.a.wave_gain is not None:
            largs['wave_gain'] = repr(float(self.a.wave_gain))
        if self.a.wave_period is not None:
            largs['wave_period'] = repr(float(self.a.wave_period))
        if self.a.wave_direction_deg is not None:
            largs['wave_direction'] = repr(math.radians(self.a.wave_direction_deg))
        self.info['launch_args'] = largs
        self.launch = self.popen(['ros2', 'launch', lc['package'], lc['file']] + [f'{k}:={v}' for k, v in largs.items()],
                                 'launch')
        t0 = time.time()
        while lc['ready_log'] not in self.text('launch'):
            if self.launch.poll() is not None:
                raise RuntimeError(f'launch exited (code {self.launch.returncode}), see {self.dir}/launch.log')
            if time.time() - t0 > lc['ready_timeout_s']:
                raise RuntimeError(f'scene not ready within {lc["ready_timeout_s"]} s, see {self.dir}/launch.log')
            time.sleep(1.0)
        self.log(f'scene ready after {time.time() - t0:.0f} s wall')

    def stop_all(self):
        for p, sig, name in ((self.arm, signal.SIGINT, 'arm'), (self.bag, signal.SIGINT, 'bag')):
            if p is not None and p.poll() is None:
                self.log(f'stopping {name}')
                kill_group(p, sig, 30.0)
        if self.launch is not None:
            self.log('stopping the simulation')
            kill_group(self.launch, signal.SIGINT, self.cfg['launch']['shutdown_timeout_s'])
            left = sweep(self.log)
            if left:
                self.log(f'WARNING: still running after the sweep: {left}')
        self.arm = self.bag = self.launch = None

    # ------------------------------------------------------------------ steps
    def reset(self, w):
        t0 = time.time()
        while time.time() - t0 < 180:
            if not w.reset_cli.wait_for_service(timeout_sec=2.0):
                continue
            fut = w.reset_cli.call_async(Trigger.Request())
            while not fut.done():
                time.sleep(0.05)
            res = fut.result()
            if res.success:
                self.log(f'/sim/reset ok: {res.message[:160]}')
                return json.loads(res.message)
            self.log(f'/sim/reset refused, retry: {res.message[:120]}')
            time.sleep(3.0)
        raise RuntimeError('/sim/reset did not succeed within 180 s')

    def wait_sim(self, w, sec, check=None):
        t0 = w.sim
        while w.sim is None or w.sim - t0 < sec:
            if self.launch.poll() is not None:
                raise RuntimeError('simulation exited')
            if check:
                check()
            time.sleep(0.05)
            if t0 is None:
                t0 = w.sim

    def start_bag(self):
        topics = list(self.cfg['topics'])
        bag_dir = os.path.join(self.dir, 'bag')
        self.bag = self.popen(['ros2', 'bag', 'record', '-s', 'mcap', '--use-sim-time', '-o', bag_dir,
                               '--topics'] + topics, 'bag_record')
        t0 = time.time()
        while time.time() - t0 < 30:
            txt = self.text('bag_record')
            missing = [t for t in topics if f"Subscribed to topic '{t}'" not in txt]
            # command / event topics appear when the arm node starts
            if not [t for t in missing if t not in ('/joint_command', '/gripper/command', '/recording/arm_event')]:
                self.log(f'bag recording ({len(topics) - len(missing)} topics subscribed, waiting for {missing})')
                return
            if self.bag.poll() is not None:
                raise RuntimeError(f'ros2 bag record exited, see {self.dir}/bag_record.log')
            time.sleep(0.5)
        raise RuntimeError(f'bag did not subscribe to all state topics: {missing}')

    def rtf_sampler(self, w, stop):
        with open(os.path.join(self.dir, 'rtf.jsonl'), 'w') as f:
            last = None
            while not stop.is_set():
                time.sleep(1.0)
                now = (time.time(), w.sim)
                if last and now[1] is not None and last[1] is not None:
                    f.write(json.dumps({'wall': round(now[0], 3), 'sim': round(now[1], 4),
                                        'rtf': round((now[1] - last[1]) / (now[0] - last[0]), 4)}) + '\n')
                    f.flush()
                last = now

    def go(self):
        a = self.a
        self.log(f'run directory {self.dir}')
        rclpy.init()
        w = Watch()
        ex = SingleThreadedExecutor()
        ex.add_node(w)
        spin = threading.Thread(target=ex.spin, daemon=True)
        spin.start()
        stop_rtf = threading.Event()
        valid = False
        waves = sent = None
        try:
            self.start_sim()
            reset = self.reset(w)
            self.log(f'settle {a.settle:g} s sim')
            self.wait_sim(w, a.settle)
            waves, sent = meta.read_wave_params()
            threading.Thread(target=self.rtf_sampler, args=(w, stop_rtf), daemon=True).start()
            self.start_bag()
            self.wait_sim(w, 1.0)
            t_arm = w.sim
            self.arm = self.popen(['ros2', 'run', 'ocean_sim2sim_recording', 'arm_motion_node.py', '--mode',
                                   'fixed' if a.arm == 'fixed' else 'scripted', '--seed', str(a.seed),
                                   '--duration', str(a.duration), '--wait-subscribers', '2']
                                  + (['--config', a.config] if a.config else []),
                                  'arm_motion')
            last = time.time()
            while self.arm.poll() is None:
                if self.launch.poll() is not None:
                    raise RuntimeError('simulation exited during the run')
                if w.pstop:
                    raise RuntimeError('protective stop during the run')
                if time.time() - last > 60:
                    self.log(f'  sim {w.sim - t_arm:.0f} / {a.duration:g} s')
                    last = time.time()
                time.sleep(0.2)
            code = self.arm.returncode
            if code != 0:
                raise RuntimeError(f'arm_motion_node exited with code {code} (3 = protective stop), '
                                   f'see {self.dir}/arm_motion.log')
            self.wait_sim(w, 1.0)
            t_end = w.sim
            valid = not w.pstop
            self.info.update(sim_arm_start_s=round(t_arm, 4), sim_end_s=round(t_end, 4), reset=reset,
                             protective_stop=w.pstop)
        except BaseException as e:
            self.log(f'ERROR: {e!r}')
            self.info['error'] = repr(e)
            raise
        finally:
            urdf = w.urdf
            stop_rtf.set()
            self.stop_all()
            ex.shutdown()                 # stop the spin thread before rclpy goes away (else: C++ terminate)
            spin.join(timeout=5.0)
            w.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.write_meta(urdf, waves, sent, valid)
        if not a.no_convert:
            e1, header = convert.convert(self.dir, log=self.log)
            m = json.load(open(os.path.join(self.dir, 'meta.json')))
            m['table'] = {'file_npz': 'data_100hz.npz', 'file_csv': 'data_100hz.csv', 'columns': header,
                          'e1_summary': {k: e1[k] for k in ('grid', 'rtf') if k in e1}}
            meta.write_json(os.path.join(self.dir, 'meta.json'), m)
        self.log(f'done: {self.dir}')
        return 0 if valid else 1

    def write_meta(self, urdf, waves, sent, valid):
        a, cfg = self.a, self.cfg
        arm_cfg = cfg['arm_motion']
        m = {'spec': 'moving_base_prediction docs/gazebo_ocean_recording/01_Recording_Spec.md, stage 1 (A, C, E1 E2 E3 '
                     'E6; arm motion runs also B5, B8)',
             'run': {'name': a.name, 'directory': os.path.basename(self.dir), 'valid': valid,
                     'duration_request_sim_s': a.duration, 'settle_sim_s': a.settle, **self.info},
             'arm': {'mode': 'fixed (capture ready pose, gripper open)' if a.arm == 'fixed' else
                     'scripted capture-like motion (arm_motion_node.py; not a policy)',
                     'seed': a.seed, 'config': arm_cfg},
             'boat': {'state': 'stationary (no thrust); drifts freely with the waves', 'target_speed_mps': 0.0,
                      'heading_at_start_deg': 0.0},
             'wind': {'mean_velocity_mps': 0.0, 'gusts': 0.0, 'note': 'vrx::USVWind in the world, mean 0 m/s'},
             'current': {'note': 'none (VRX has no current model in this world)'},
             'waves': meta.wave_summary(waves) if waves else None,
             'waves_keys_sent_by_world': sent,
             'frames': {'world': 'Gazebo world, ENU (x east, y north, z up); the boat starts at the origin, bow on +x',
                        'body': 'wamv/base_link, FLU (x bow, y port, z up)',
                        'imu': 'wamv/imu_wamv_link, axes = body axes (FLU)',
                        'quaternion_order': 'x, y, z, w; A3 / A6 rotate body / IMU vectors into the world frame',
                        'velocities': 'A4 / A5 = velocity of the body origin, expressed in the body frame'},
             'simulation': {'physics_step_s': 1 / 240, 'physics_rate_hz': 240, 'engine': 'DART + Bullet collision',
                            'real_time_factor_target': 1.0, 'lockstep': True, **meta.versions()},
             'topics': {t: dict(v, type=None) for t, v in cfg['topics'].items()},
             'units': 'SI: m, m/s, rad, rad/s, m/s^2; gripper raw 0 (open) .. 1150 (closed); time = sim time [s]'}
        types = self._bag_types()
        for t in m['topics']:
            m['topics'][t]['type'] = types.get(t)
        if urdf:
            m['mounting_e3'] = meta.mounting(urdf, arm_cfg['init_pose'])
            m['imu_sensor'] = meta.imu_sensor(urdf)
        meta.write_json(os.path.join(self.dir, 'meta.json'), m)
        self.log(f'meta.json written (valid {valid})')

    def _bag_types(self):
        try:
            with open(os.path.join(self.dir, 'bag', 'metadata.yaml')) as f:
                md = yaml.safe_load(f)['rosbag2_bagfile_information']
            return {x['topic_metadata']['name']: x['topic_metadata']['type'] for x in md['topics_with_message_count']}
        except (OSError, KeyError, TypeError):
            return {}


def main():
    p = argparse.ArgumentParser(description='One boat motion recording run (sim + bag + meta.json + 100 Hz table).')
    p.add_argument('name', help='run name (condition), e.g. gain03_dir0')
    p.add_argument('--arm', choices=('fixed', 'scripted'), required=True)
    p.add_argument('--duration', type=float, default=900.0, help='arm motion time [s sim] (default 900 = 15 min)')
    p.add_argument('--seed', type=int, default=0, help='scripted motion seed')
    p.add_argument('--world', default='empty_ocean')
    p.add_argument('--wave-gain', type=float, default=None)
    p.add_argument('--wave-period', type=float, default=None)
    p.add_argument('--wave-direction-deg', type=float, default=None,
                   help='where the waves travel to, from world +x (bow at start); 0 = following sea, 180 = head sea')
    p.add_argument('--settle', type=float, default=5.0, help='sim seconds after the reset before recording')
    p.add_argument('--out-dir', default='')
    p.add_argument('--config', default='', help='recording.yaml (default: installed config)')
    p.add_argument('--no-convert', action='store_true', help='skip the 100 Hz table (bag_to_table.py later)')
    a = p.parse_args()
    cfg_path = a.config or os.path.join(get_package_share_directory('ocean_sim2sim_recording'), 'config',
                                        'recording.yaml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    if shutil.which('gz') is None:
        sys.exit('gz not found (source /opt/ros/jazzy/setup.bash)')
    run = Run(a, cfg)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        sys.exit(run.go())
    except KeyboardInterrupt:
        run.log('interrupted')
        sys.exit(130)
    except RuntimeError as e:
        run.log(f'run failed: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
