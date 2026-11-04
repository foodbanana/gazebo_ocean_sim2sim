"""openpi policy client for the Gazebo ocean sim (library: PolicyClient + pure helpers). docs/policy.md

Ported from data_collection_ur5_gripper policy/policy_client_node.py (Isaac Sim, 2026-10-27; docs/policy_client.md).
Same threads (ROS spin / 60 Hz command loop / background inference), same observation pairing (t_obs = latest stamp
common to the cameras, every other input = latest at or before t_obs, older than obs_max_age 66 ms -> error), same chunk
timing (step i = sim t_obs + i / control_hz, replan at replan_steps, steps passed during inference skipped, linear
interpolation at publish_hz, last step held), same gripper threshold, safety limits (max_joint_speed, max_lead,
limit_margin), desync / protective stop stop, auto-kill rule and episode result fields.

Changed for Gazebo / the next policies (docs/policy.md "Isaac Sim 과 다른 점"):
  - joint names: config arm_joints (Isaac: teleop/ik_model.py ARM_JOINTS); joint limits: <limit> of those joints in the
    /robot_description URDF of the running sim (Isaac: the same URDF read from a file in that repository)
  - cameras: config map {policy observation key: image topic} (Isaac: fixed wrist + third view). Gazebo has the wrist
    camera only. A configured camera that is not published is an error; blank_images can send a black image for an
    observation key the sim does not have (plumbing test with the 2-camera checkpoint), off by default
  - base_imu (optional, off by default): mean of /base/imu [gyro xyz rad/s, accel xyz m/s^2] over (t_obs - window, t_obs]
    = Isaac stage1 lerobot_stage1_extract_bag.py --base-imu topic; as its own observation key or appended to the state.
    t_obs is also limited to the latest IMU stamp (so the window is complete), which with the IMU arriving before the
    images leaves t_obs = the image stamp as in Isaac
  - /sim/reset: retried while the scene answers "not ready" (first drone still starting), as grasp_demo.py
  - per-chunk inference timing in the result
"""
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import yaml

REQUIRED = ('server', 'prompt', 'control_hz', 'replan_steps', 'publish_hz', 'obs_max_age', 'gripper', 'max_joint_speed',
            'max_lead', 'limit_margin', 'desync', 'arm_joints', 'cameras', 'blank_images', 'base_imu', 'topics')
TOPICS = ('clock', 'joint_states', 'gripper_state', 'protective_stop', 'joint_command', 'gripper_command',
          'reset_service', 'kill_service', 'robot_description')
BASE_IMU = ('enabled', 'topic', 'window', 'mode', 'key')
BASE_IMU_NAMES = ('gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z')   # stage1 meta.json base_imu_names
STATE_KEY = 'observation/state'


def load_config(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    check_config(c, path)
    return c


def check_config(c, path='config'):
    """No defaults in code (as Isaac): every key must be in the file. Raises ValueError."""
    missing = [k for k in REQUIRED if k not in c] + [f'topics.{k}' for k in TOPICS if k not in (c.get('topics') or {})]
    missing += [f'gripper.{k}' for k in ('threshold', 'raw_open', 'raw_close', 'present_scale') if k not in c.get('gripper', {})]
    missing += [f'base_imu.{k}' for k in BASE_IMU if k not in (c.get('base_imu') or {})]
    if missing:
        raise ValueError(f'{path}: missing keys: {missing}')
    lat = c.get('latency') or {}
    if lat.get('mode') not in ('measured', 'fixed') or 'fixed_s' not in lat:
        raise ValueError(f"{path}: latency must be {{mode: measured|fixed, fixed_s: <s>}}: {lat}")
    if not (1 <= int(c['replan_steps'])):
        raise ValueError(f"{path}: replan_steps must be >= 1: {c['replan_steps']}")
    if float(c['max_lead']) >= float(c['desync']):
        raise ValueError(f"{path}: max_lead ({c['max_lead']}) must be smaller than desync ({c['desync']})")
    cams, blank = c['cameras'] or {}, c['blank_images'] or {}
    if not cams:
        raise ValueError(f'{path}: cameras is empty (at least one camera: its stamps are the observation time t_obs)')
    both = sorted(set(cams) & set(blank))
    if both:
        raise ValueError(f'{path}: {both} in both cameras and blank_images')
    for k, hw in blank.items():
        if not (isinstance(hw, (list, tuple)) and len(hw) == 2 and all(int(v) > 0 for v in hw)):
            raise ValueError(f'{path}: blank_images.{k} must be [height, width]: {hw}')
    b = c['base_imu']
    if b['mode'] not in ('key', 'state'):
        raise ValueError(f"{path}: base_imu.mode must be key or state: {b['mode']}")
    if not float(b['window']) > 0:
        raise ValueError(f"{path}: base_imu.window must be > 0: {b['window']}")
    if b['enabled'] and b['mode'] == 'key' and b['key'] in set(cams) | set(blank) | {STATE_KEY, 'prompt'}:
        raise ValueError(f"{path}: base_imu.key {b['key']} is already an observation key")
    if len(c['arm_joints']) != 6:
        raise ValueError(f"{path}: arm_joints must be the 6 arm joints (action = 6 joints + gripper): {c['arm_joints']}")


def t_of(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def urdf_limits(urdf_xml, joints):
    """(lo, hi) [rad] from <limit lower upper> of the named joints (continuous: -inf / inf)."""
    by_name = {j.get('name'): j for j in ET.fromstring(urdf_xml).findall('joint')}
    lo, hi = [], []
    for n in joints:
        j = by_name.get(n)
        if j is None:
            raise ValueError(f'joint {n} is not in the URDF')
        lim = j.find('limit')
        if j.get('type') == 'continuous':
            lo.append(-math.inf), hi.append(math.inf)
        elif lim is None or lim.get('lower') is None or lim.get('upper') is None:
            raise ValueError(f'joint {n} ({j.get("type")}) has no <limit lower upper>')
        else:
            lo.append(float(lim.get('lower'))), hi.append(float(lim.get('upper')))
    return np.array(lo), np.array(hi)


def imu_window_mean(imu, t, window):
    """Mean of the 6 IMU values with stamp in (t - window, t] (stage1 --base-imu topic). imu: [(stamp, (6,))].
    -> ((6,) float, stamp of the newest one used). RuntimeError if the window is empty."""
    sel = [v for s, v in imu if t - window < s <= t + 1e-9]
    if not sel:
        newest = imu[-1][0] if imu else None
        raise RuntimeError(f'base_imu: no IMU message in ({t - window:.3f}, {t:.3f}] (newest {newest})')
    used = [s for s, _ in imu if t - window < s <= t + 1e-9]
    return np.mean(np.asarray(sel, dtype=float), axis=0), max(used)


def build_observation(snap, cfg):
    """Pair the buffered messages into one policy observation, as the training dataset (stage1) did.

    snap: {'js': [(stamp, q (6,))], 'grip': [(stamp, raw)], 'img': {obs key: [(stamp, HxWx3 uint8)]},
           'imu': [(stamp, (6,))] or None}
    t_obs = the oldest of the latest camera stamps (and of the latest IMU stamp when base_imu is on): every stream has
    arrived up to t_obs. Each input = its latest message at or before t_obs; older than obs_max_age -> RuntimeError.
    -> (obs dict, t_obs)."""
    img, imu_cfg = snap['img'], cfg['base_imu']
    use_imu = bool(imu_cfg['enabled'])
    empty = [k for k, v in img.items() if not v] + (['base_imu'] if use_imu and not snap['imu'] else [])
    if empty:
        raise RuntimeError(f'no message yet: {empty}')
    t_obs = min(v[-1][0] for v in img.values())
    if use_imu:
        t_obs = min(t_obs, snap['imu'][-1][0])

    def pick(buf, name):
        m = [x for x in buf if x[0] <= t_obs + 1e-6]
        if not m:
            first = f'{buf[0][0]:.3f}' if buf else 'none'
            raise RuntimeError(f'{name}: no message at or before t_obs {t_obs:.3f} (oldest {first})')
        return m[-1]

    sel = {'joint_states': pick(snap['js'], 'joint_states'), 'gripper': pick(snap['grip'], 'gripper')}
    sel.update({k: pick(v, k) for k, v in img.items()})
    imu_mean = None
    if use_imu:
        imu_mean, t_imu = imu_window_mean(snap['imu'], t_obs, float(imu_cfg['window']))
        sel['base_imu'] = (t_imu, imu_mean)
    old = {k: round(t_obs - v[0], 3) for k, v in sel.items() if t_obs - v[0] > float(cfg['obs_max_age'])}
    if old:
        raise RuntimeError(f"observation older than t_obs [s]: {old} (limit {cfg['obs_max_age']})")
    sc = float(cfg['gripper']['present_scale'])
    state = np.concatenate([sel['joint_states'][1], [sel['gripper'][1] / sc]]).astype(np.float32)
    obs = {}
    if use_imu and imu_cfg['mode'] == 'state':
        state = np.concatenate([state, imu_mean.astype(np.float32)])
    elif use_imu:
        obs[imu_cfg['key']] = imu_mean.astype(np.float32)
    obs[STATE_KEY] = state
    for k in img:
        obs[k] = sel[k][1]
    for k, (h, w) in (cfg['blank_images'] or {}).items():
        obs[k] = np.zeros((int(h), int(w), 3), dtype=np.uint8)
    obs['prompt'] = cfg['prompt']
    return obs, t_obs


def chunk_step(act, x):
    """Linear interpolation between chunk steps at position x [steps] (clamped to [0, last]).
    -> (interpolated action (7,), i0 = step whose gripper value is used, held = x is at / past the last step)."""
    last = act.shape[0] - 1
    xc = min(max(x, 0.0), last)
    i0 = min(int(math.floor(xc)), last)
    i1 = min(i0 + 1, last)
    return act[i0] + (xc - i0) * (act[i1] - act[i0]), i0, x >= last


def safe_target(q_target, q_cmd, q_meas, dt, max_joint_speed, max_lead, lo, hi):
    """Teleop ik_step limits: at most max_joint_speed * dt per command, measured +- max_lead, inside the joint limits."""
    max_dq = float(max_joint_speed) * dt
    q = q_cmd + np.clip(q_target - q_cmd, -max_dq, max_dq)
    q = np.clip(q, q_meas - float(max_lead), q_meas + float(max_lead))
    return np.clip(q, lo, hi)


class PolicyClient:
    def __init__(self, cfg, policy, log=lambda m: print(m, flush=True), node_name='policy_client'):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, Imu, JointState
        from std_msgs.msg import Bool, Float64, String
        from std_srvs.srv import Trigger

        self.cfg, self.policy, self.log = cfg, policy, log
        self.JointState, self.Float64, self.Trigger = JointState, Float64, Trigger
        self._rclpy = rclpy
        self.node = Node(node_name)
        self.arm_joints = list(cfg['arm_joints'])
        self.lo = self.hi = None                     # from /robot_description (wait_ready)
        self.urdf = None
        self.lock = threading.Lock()
        self.clock = None                            # latest sim time [s]
        # buffer the messages briefly: the cameras arrive 1 - 2 frames after /joint_states (rendering, transport), so the
        # joints / gripper are picked at the image time (as stage1 paired equal times)
        self.js = deque(maxlen=120)                  # (stamp, q 6), 120 Hz -> 1 s
        self.grip = deque(maxlen=30)                 # (stamp, present raw), ~30 Hz -> 1 s
        self.img = {k: deque(maxlen=4) for k in cfg['cameras']}      # obs key -> (stamp, HxWx3 uint8)
        self.imu = deque(maxlen=200) if cfg['base_imu']['enabled'] else None   # (stamp, (6,)), 100 Hz -> 2 s
        self.pstop = False
        self.err = None                              # input problem found in a callback (raised in the main thread)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        t = cfg['topics']
        self.node.create_subscription(Clock, t['clock'], self._on_clock, latest)
        self.node.create_subscription(JointState, t['joint_states'], self._on_js, latest)
        self.node.create_subscription(JointState, t['gripper_state'], self._on_grip, latest)
        for key, topic in cfg['cameras'].items():
            self.node.create_subscription(Image, topic, lambda msg, k=key: self._on_img(k, msg), latest)
        if self.imu is not None:
            # 100 Hz, averaged over 40 ms: keep every message (depth 1 would drop some between two spins)
            self.node.create_subscription(Imu, cfg['base_imu']['topic'], self._on_imu, 50)
        self.node.create_subscription(Bool, t['protective_stop'], self._on_pstop, latest)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, t['robot_description'], self._on_urdf, latched)
        self.pub_arm = self.node.create_publisher(JointState, t['joint_command'], 10)
        self.pub_grip = self.node.create_publisher(Float64, t['gripper_command'], 10)
        self.cli_reset = self.node.create_client(Trigger, t['reset_service'])
        self.cli_kill = self.node.create_client(Trigger, t['kill_service'])
        self._stop = threading.Event()
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    # -- ROS callbacks (spin thread)
    def _on_clock(self, msg):
        with self.lock:
            self.clock = t_of(msg.clock)

    def _on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        missing = [n for n in self.arm_joints if n not in d]
        if missing:
            self.err = f"{self.cfg['topics']['joint_states']} has no joints {missing} (names {list(msg.name)})"
            return
        with self.lock:
            self.js.append((t_of(msg.header.stamp), np.array([d[n] for n in self.arm_joints], dtype=float)))

    def _on_grip(self, msg):
        with self.lock:
            self.grip.append((t_of(msg.header.stamp), float(msg.position[0])))

    def _on_img(self, key, msg):
        if msg.encoding != 'rgb8':
            self.err = f'{key} image encoding is not rgb8: {msg.encoding} (training data is rgb8)'
            return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        with self.lock:
            self.img[key].append((t_of(msg.header.stamp), a))

    def _on_imu(self, msg):
        w, a = msg.angular_velocity, msg.linear_acceleration
        with self.lock:
            self.imu.append((t_of(msg.header.stamp), np.array([w.x, w.y, w.z, a.x, a.y, a.z], dtype=float)))

    def _on_pstop(self, msg):
        with self.lock:
            self.pstop = bool(msg.data)

    def _on_urdf(self, msg):
        with self.lock:
            self.urdf = msg.data

    def _spin_loop(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop.is_set() and self._rclpy.ok():
            ex.spin_once(timeout_sec=0.05)

    # -- helpers
    def now(self):
        with self.lock:
            return self.clock

    def wait_sim_until(self, t_target, poll=0.002):
        while not self._stop.is_set():
            now = self.now()
            if now is not None and now >= t_target:
                return now
            time.sleep(poll)
        raise KeyboardInterrupt

    def _input_topics(self):
        t = self.cfg['topics']
        topics = {'clock': t['clock'], 'joint_states': t['joint_states'], 'gripper': t['gripper_state']}
        topics.update({k: v for k, v in self.cfg['cameras'].items()})
        if self.imu is not None:
            topics['base_imu'] = self.cfg['base_imu']['topic']
        return topics

    def wait_ready(self, timeout=60.0, no_publisher_after=10.0):
        """Until every observation arrives and the sim subscribes to both command topics (commands sent before the
        connection are lost and the target jumps). An input topic with no publisher at all after no_publisher_after s
        (while the sim is up) is an error at once (e.g. a camera the Gazebo scene does not have)."""
        t0 = time.monotonic()
        topics = self._input_topics()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                have = {'clock': self.clock is not None, 'joint_states': bool(self.js), 'gripper': bool(self.grip),
                        'robot_description': self.urdf is not None}
                have.update({k: bool(v) for k, v in self.img.items()})
                if self.imu is not None:
                    have['base_imu'] = bool(self.imu)
            subs = {'joint_command': self.pub_arm.get_subscription_count() > 0,
                    'gripper_command': self.pub_grip.get_subscription_count() > 0}
            if self.err:
                raise RuntimeError(self.err)
            if time.monotonic() - t0 > no_publisher_after and have['clock'] and have['joint_states']:   # sim is up
                none = {k: v for k, v in topics.items() if not have.get(k) and self.node.count_publishers(v) == 0}
                if none:
                    cams = [k for k in none if k in self.cfg['cameras']]
                    hint = (f' -- camera(s) {cams} are not in this sim (Gazebo has the wrist camera only): remove them from'
                            ' cameras, or (plumbing test only) list them under blank_images') if cams else ''
                    raise RuntimeError(f'no publisher for {none}{hint}')
            if all(have.values()) and all(subs.values()):
                if self.lo is None:
                    lo, hi = urdf_limits(self.urdf, self.arm_joints)
                    m = float(self.cfg['limit_margin'])
                    self.lo, self.hi = lo + m, hi - m
                # just after start the joints before the (late) image may not be there yet -> until pairing works
                try:
                    self.observe()
                    return
                except RuntimeError as e:
                    if time.monotonic() - t0 > 10.0:
                        raise RuntimeError(f'cannot pair the observation: {e}') from e
            time.sleep(0.2)
        raise RuntimeError(f'not ready within {timeout:g} s: inputs {have}, command subscribers {subs} (is the sim up?)')

    def call(self, client, name, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'service {name} not available (is the sim up?)')
        fut = client.call_async(self.Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'service {name} did not answer within {timeout:g} s')
            time.sleep(0.05)
        return fut.result()

    def reset(self, timeout=120.0, not_ready_wait=300.0):
        """/sim/reset -> its JSON answer (dict). The Gazebo scene answers 'not ready' until its first drone is up
        (PX4: ~20 s sim after the launch): wait and try again, as grasp_demo.py."""
        name = self.cfg['topics']['reset_service']
        t_wait = time.monotonic()
        while True:
            r = self.call(self.cli_reset, name, timeout)
            if r.success or 'not ready' not in r.message or time.monotonic() - t_wait > not_ready_wait:
                break
            self.log('[policy] scene not ready yet (first drone still starting), waiting ...')
            time.sleep(2.0)
        if not r.success:
            raise RuntimeError(f'{name} failed: {r.message[:300]}')
        return json.loads(r.message)

    def snapshot(self):
        with self.lock:
            return {'js': list(self.js), 'grip': list(self.grip), 'img': {k: list(v) for k, v in self.img.items()},
                    'imu': list(self.imu) if self.imu is not None else None}, self.pstop

    def observe(self):
        """Observation as in the training dataset (build_observation). -> obs, t_obs, measured joints now,
        present now (0..1), protective stop."""
        if self.err:
            raise RuntimeError(self.err)
        snap, pstop = self.snapshot()
        obs, t_obs = build_observation(snap, self.cfg)
        return obs, t_obs, snap['js'][-1][1].copy(), snap['grip'][-1][1] / float(self.cfg['gripper']['present_scale']), pstop

    def safe_target(self, q_target, q_cmd, q_meas, dt):
        c = self.cfg
        return safe_target(q_target, q_cmd, q_meas, dt, c['max_joint_speed'], c['max_lead'], self.lo, self.hi)

    def send_arm(self, q, t_sim):
        msg = self.JointState()
        sec = int(math.floor(t_sim))
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((t_sim - sec) * 1e9)) % 1_000_000_000
        msg.name = list(self.arm_joints)
        msg.position = [float(x) for x in q]
        self.pub_arm.publish(msg)

    def send_grip(self, closed):
        g = self.cfg['gripper']
        self.pub_grip.publish(self.Float64(data=float(g['raw_close'] if closed else g['raw_open'])))

    # -- episode
    def _infer(self):
        """observation -> policy. -> (chunk (H, 7), t_obs, wall time, server inference ms or None)."""
        obs, t_obs, _, _, _ = self.observe()
        w0 = time.monotonic()
        out = self.policy.infer(obs)
        act = np.asarray(out['actions'], dtype=float)
        if act.ndim != 2 or act.shape[1] != 7 or not np.all(np.isfinite(act)):
            raise RuntimeError(f'policy output is not finite (H, 7): {act.shape}')
        server_ms = (out.get('server_timing') or {}).get('infer_ms')
        return act, t_obs, time.monotonic() - w0, server_ms

    def warmup(self):
        """The server's first inference compiles (JIT, ~11 s): once before the episodes (the robot does not move)."""
        _, _, w, _ = self._infer()
        self.log(f'[policy] first inference (incl. server compile) {w:.1f} s')

    def run_episode(self, duration, auto_kill):
        """Chunk step i = command for sim t_obs + i * dt_c (dataset action[t] = latest command before frame t).
        At publish_hz: position in the current chunk from the current sim time, linearly interpolated target. At step
        replan_steps the next inference starts in the background (the current chunk keeps running) and replaces it on
        arrival. At the end of the chunk the last step is held.
"""
        cfg = self.cfg
        dt_c, dt_p, K = 1.0 / float(cfg['control_hz']), 1.0 / float(cfg['publish_hz']), int(cfg['replan_steps'])
        thr = float(cfg['gripper']['threshold'])
        _, _, q_meas, _, _ = self.observe()
        q_cmd, closed = q_meas.copy(), False
        self.send_grip(False)
        t_start = self.now()
        t_closed, killed = None, False
        st = {'chunks': 0, 'infer_wall': [], 'server_ms': [], 'lag_steps': [], 'limited': 0, 'sent': 0, 'held': 0,
              'stop': 'duration', 'gripper_events': []}
        self.log(f"[policy] episode start: sim t {t_start:.2f}, {duration:g} s, replan every {K} steps / {cfg['control_hz']:g} Hz")
        act, t_obs, w, sms = self._infer()
        st['chunks'] += 1
        st['infer_wall'].append(w)
        st['server_ms'].append(sms)
        # latency.mode fixed: a chunk is used only from sim t_obs + fixed_s on (docs/policy.md "추론 지연"): the policy sees
        # the same observation -> action delay in sim time whatever the RTF (Gazebo slows down in contact -> the wall
        # inference time would otherwise shrink in sim time). measured = Isaac client behaviour (use on arrival).
        lat = cfg['latency']
        fixed = float(lat['fixed_s']) if lat['mode'] == 'fixed' else None
        st['late'] = 0                               # chunks that arrived after t_obs + fixed_s (delay longer than wanted)
        if fixed is not None:
            if self.now() > t_obs + fixed:
                st['late'] += 1
            self.wait_sim_until(t_obs + fixed)
        box, worker, pending = {}, None, None        # next inference in the background, arrived but not due yet

        def bg():
            try:
                box['res'] = self._infer()
            except Exception as e:                   # noqa: BLE001  (raised again in the main loop)
                box['err'] = e

        t_next = self.now()
        while True:
            t_next += dt_p
            now = self.wait_sim_until(t_next)
            if now - t_start >= duration:
                break
            with self.lock:
                pstop, q_meas = self.pstop, self.js[-1][1].copy()
            if pstop:
                st['stop'] = 'protective_stop'
                break
            dev = float(np.max(np.abs(q_meas - q_cmd)))
            if dev > float(cfg['desync']):
                st['stop'] = f'desync {math.degrees(dev):.1f} deg'
                break
            if worker is not None and not worker.is_alive():   # next chunk arrived
                worker = None
                if 'err' in box:
                    raise box['err']
                pending = box.pop('res')
                if fixed is not None and now > pending[1] + fixed + 1e-9:
                    st['late'] += 1
            if pending is not None and (fixed is None or now >= pending[1] + fixed - 1e-9):
                act, t_obs, w, sms = pending
                pending = None
                st['chunks'] += 1
                st['infer_wall'].append(w)
                st['server_ms'].append(sms)
                st['lag_steps'].append((now - t_obs) / dt_c)
            x = (now - t_obs) / dt_c                 # position in the current chunk [steps]
            if worker is None and pending is None and x >= K:
                worker = threading.Thread(target=bg, daemon=True)
                worker.start()
            a, i0, held = chunk_step(act, x)
            st['held'] += int(held)                  # next chunk late -> last step held
            q_new = self.safe_target(a[:6], q_cmd, q_meas, dt_p)
            st['limited'] += int(not np.allclose(q_new, a[:6], atol=1e-6))
            q_cmd = q_new
            self.send_arm(q_cmd, now)
            st['sent'] += 1
            want = bool(act[i0, 6] > thr)
            if want != closed:
                closed = want
                self.send_grip(closed)
                t_closed = now if closed else None
                st['gripper_events'].append((round(now - t_start, 3), 'close' if closed else 'open', round(float(act[i0, 6]), 3)))
                self.log(f"[policy] gripper {'close' if closed else 'open'} (episode t {now - t_start:.2f} s, output {act[i0, 6]:.2f})")
            if auto_kill is not None and closed and not killed and t_closed is not None and now - t_closed >= auto_kill:
                with self.lock:
                    present = self.grip[-1][1] / float(cfg['gripper']['present_scale'])
                if present < 0.9:
                    r = self.call(self.cli_kill, cfg['topics']['kill_service'], 10.0)
                    killed = True
                    self.log(f'[policy] drone motors off (gripper present {present:.2f} = blocked by the object): '
                             f'{r.success} {r.message[:80]}')
        if worker is not None:
            worker.join(timeout=10.0)
        _, _, _, present, _ = self.observe()
        iw = np.array(st['infer_wall'])
        lag = np.array(st['lag_steps']) if st['lag_steps'] else np.array([np.nan])
        res = {'stop': st['stop'], 'sim_s': round(self.now() - t_start, 2), 'chunks': st['chunks'], 'sent': st['sent'],
               'infer_wall_median_s': round(float(np.median(iw)), 3), 'infer_wall_max_s': round(float(np.max(iw)), 3),
               'chunk_arrive_step_median': round(float(np.median(lag)), 2), 'held_frac': round(st['held'] / max(st['sent'], 1), 3),
               'limited_frac': round(st['limited'] / max(st['sent'], 1), 3),
               'latency': lat['mode'] if fixed is None else f'fixed {fixed:g} s', 'late_chunks': st['late'],
               'gripper_closed': closed, 'gripper_present_end': round(present, 3), 'drone_killed': killed}
        self.log(f'[policy] episode end: {json.dumps(res, ensure_ascii=False)}')
        # extra detail (not printed): per chunk timing, gripper changes
        res['detail'] = {'t_start': t_start, 'infer_wall_s': [round(v, 4) for v in st['infer_wall']],
                         'server_infer_ms': [None if v is None else round(float(v), 1) for v in st['server_ms']],
                         'chunk_arrive_steps': [round(v, 3) for v in st['lag_steps']],
                         'gripper_events': st['gripper_events']}
        return res

    def shutdown(self):
        self._stop.set()
        self._spin.join(timeout=1.0)
        self.node.destroy_node()
