# =============================================================
# xr_teleop.py - arm + gripper teleoperation with the PICO 4 Ultra controllers (library of scripts/xr_teleop_node.py)
#
#   Ported from: data_collection_ur5_gripper teleop/xr_teleop_node.py (load_config, heading_map, OneEuroPose, XrSample,
#     XrInput, wait_xr, XrTeleop) - same control law, filter, buttons, safety checks and messages (in English).
#   Controller pose (XRoboToolkit binding, 90 Hz) -> TCP target -> differential IK (ik_model.py) -> /joint_command,
#   /gripper/command (the only published topics that are recorded) + recorder requests /teleop/request
#   (std_msgs/String, not recorded): controller buttons -> recorder (record_toggle.py) actions.
#
#   Operation (buttons from the config teleop_button / buttons):
#     - hold the controller in the same start pose every time and press the TELEOP BUTTON (Y, left hand) -> start. The
#       right controller pose at that moment and the TCP pose (current /joint_states) are the reference. The controller
#       forward axis (its own -z) projected on the horizontal plane = robot base +x, up = base +z. The TCP follows the
#       change of the hand position / rotation.
#     - teleop button again -> stop (no more commands, the arm stays at the last target). Again -> new reference, start
#     - trigger: above close_above -> close (raw_close), below open_below -> open (raw_open). Only while teleoperating
#     - recorder requests (config buttons, sent once at the press, also when not teleoperating): A = record (start /
#       stop without a reset; start only after a reset), B = reset (episode reset; while recording: failed episode,
#       moved to _discarded/, then reset), X (left) = drone_kill (drone motors off)
#     - order: B reset -> Y teleop on -> hold still -> A record -> grasp -> X motors off -> A stop -> B next episode
#   One Euro filter on the controller pose (config filter): strong when the hand is still (tremor), weak when fast (lag).
#     Re-initialised at the start.
#   Wrist near the mount (min_wrist_height): targets that go further down are not sent (notice).
#   Start refused (reason printed): controller tip near vertical, /joint_states or /clock missing, protective stop.
#   Stops by itself (reason printed): measured joints far from the joint target (reset moved the arm, protective stop),
#     /protective_stop true, reset in progress (/ocean_sim2sim/hold_state true), /joint_states stale. Continuing would make the arm jump to the old target after a reset.
#   Tracking loss / headset disconnection are not judged automatically (the operator discards the episode); a frozen
#     timestamp only gives a notice.
#   Frames: everything is in the robot base frame (robot_mount). On the boat the base moves with the waves, so the arm
#     moves with the boat when the hand is still (same as the Isaac Sim Stewart moving base; user decision 2026-11-04).
#
#   Changes from the Isaac Sim file: the arm model comes from /robot_description (ik_model.ArmModel(urdf_xml)), the
#     config file is in this package, messages in English, the "no recorder" notice names this repo's recorder, and
#     the reset hold of the interface (topics.reset_hold) stops the teleop: the Gazebo reset moves the arm smoothly in
#     2 s, the lead-limited joint target follows it, so the Isaac desync check (teleported arm) never fires and the arm
#     went back to the old hand target after the reset (check 9 failed before, 2026-11-04).
# =============================================================

import math
import os
import threading
import time

import numpy as np
import yaml

from ocean_sim2sim_teleop import ik_model as km

REQUIRED = ("rate_hz", "stamp", "topics", "hand", "scale", "rotation", "max_start_elevation_deg", "max_joint_speed", "max_lead",
            "damping", "limit_margin", "warn_pos_err", "warn_manip", "min_wrist_height", "gripper", "filter", "teleop_button",
            "buttons", "xr_connect_timeout", "xr_stale_warn", "state_stale", "desync", "status_sec")
REQUESTS = ("record", "reset", "drone_kill")    # requests the recorder knows (record_toggle.py TELEOP_REQUESTS)
BUTTONS = ("A", "B", "X", "Y")                  # A / B right, X / Y left controller
XR_UP = np.array([0.0, 1.0, 0.0])               # binding frame: right handed, +y up
CTRL_FORWARD = np.array([0.0, 0.0, -1.0])       # controller own frame forward (+x right, +y up, -z forward)


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ocean_sim2sim_teleop"), "config", "xr_teleop.yaml")


def load_config(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in REQUIRED:
        if k not in c:
            raise KeyError(f"{path}: missing '{k}'")
    for k in ("joint_states", "joint_command", "gripper_command", "clock", "protective_stop", "reset_hold", "request",
              "robot_description"):
        if k not in c["topics"]:
            raise KeyError(f"{path}: missing 'topics.{k}'")
    for k in ("close_above", "open_below", "raw_open", "raw_close"):
        if k not in c["gripper"]:
            raise KeyError(f"{path}: missing 'gripper.{k}'")
    if c["stamp"] not in ("sim", "wall"):
        raise ValueError(f"{path}: stamp must be sim or wall: {c['stamp']}")
    if c["hand"] != "right":
        raise ValueError(f"{path}: hand: only right for now (right pose / trigger): {c['hand']}")
    for k in ("enabled", "pos_min_cutoff_hz", "pos_beta", "rot_min_cutoff_hz", "rot_beta", "d_cutoff_hz"):
        if k not in c["filter"]:
            raise KeyError(f"{path}: missing 'filter.{k}'")
    for k in ("pos_min_cutoff_hz", "rot_min_cutoff_hz", "d_cutoff_hz"):
        if not float(c["filter"][k]) > 0:
            raise ValueError(f"{path}: filter.{k} must be > 0: {c['filter'][k]}")
    for k in ("pos_beta", "rot_beta"):
        if float(c["filter"][k]) < 0:
            raise ValueError(f"{path}: filter.{k} must be >= 0: {c['filter'][k]}")
    if not c["gripper"]["open_below"] < c["gripper"]["close_above"]:
        raise ValueError(f"{path}: gripper.open_below < close_above needed")
    b = c["buttons"] or {}
    for req, btn in b.items():
        if req not in REQUESTS:
            raise ValueError(f"{path}: buttons request names are {REQUESTS}: {req}")
        if btn not in BUTTONS:
            raise ValueError(f"{path}: buttons.{req} must be one of {BUTTONS}: {btn}")
    if c["teleop_button"] not in BUTTONS:
        raise ValueError(f"{path}: teleop_button must be one of {BUTTONS}: {c['teleop_button']}")
    used = list(b.values()) + [c["teleop_button"]]
    if len(set(used)) != len(used):
        raise ValueError(f"{path}: a button is used twice: teleop_button {c['teleop_button']}, buttons {b}")
    c["buttons"] = b
    if c["rate_hz"] < 50:
        raise ValueError(f"{path}: rate_hz must be >= 50 (twice the 25 Hz dataset): {c['rate_hz']}")
    c["path"] = path
    return c


# ---------------------------------------------------------------- rotations
def quat_to_R(q):
    """Quaternion (x, y, z, w) -> rotation matrix."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def heading_map(R_ctrl, max_elev_deg):
    """Controller pose -> (M, elevation [deg]). M = rotation from the binding frame to the robot base frame (3x3):
    controller forward projected on the horizontal plane -> base +x, its left -> +y, up (+y_xr) -> +z.
    Tip near vertical -> M = None."""
    f = R_ctrl @ CTRL_FORWARD
    elev = math.degrees(math.asin(max(-1.0, min(1.0, float(f @ XR_UP)))))
    if abs(elev) > max_elev_deg:
        return None, elev
    fh = f - (f @ XR_UP) * XR_UP
    fh /= np.linalg.norm(fh)
    right = np.cross(fh, XR_UP)
    return np.array([fh, -right, XR_UP]), elev


# ---------------------------------------------------------------- filter (One Euro, Casiez et al. CHI 2012)
def _alpha(cutoff_hz, dt):
    """One step coefficient of a first order low pass (cutoff cutoff_hz, time step dt)."""
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt)


def _rotvec_to_R(w):
    a = float(np.linalg.norm(w))
    return np.eye(3) if a < 1e-12 else km.axis_angle_R(w / a, a)


class OneEuroPose:
    """One Euro filter for position and orientation. cutoff = min_cutoff + beta x speed (speed filtered with d_cutoff).
    Position: speed [m/s], beta [Hz/(m/s)]. Orientation: angular speed [rad/s], beta [Hz/(rad/s)]; the output rotates
    from the last output towards the new pose by alpha (rotation vector)."""

    def __init__(self, cfg):
        self.c = {k: float(v) for k, v in cfg.items() if k != "enabled"}
        self.p = self.R = None

    def reset(self, p, R):
        self.p, self.R = np.array(p, float), np.array(R, float)
        self.v, self.w = 0.0, 0.0                   # filtered speed / angular speed

    def __call__(self, p, R, dt):
        if self.p is None or dt <= 0:
            self.reset(p, R)
            return self.p.copy(), self.R.copy()
        c, ad = self.c, _alpha(self.c["d_cutoff_hz"], dt)
        dp = np.asarray(p, float) - self.p
        self.v += ad * (np.linalg.norm(dp) / dt - self.v)
        self.p = self.p + _alpha(c["pos_min_cutoff_hz"] + c["pos_beta"] * self.v, dt) * dp
        dw = km.R_to_rotvec(self.R.T @ R)             # last output -> new pose (own frame)
        self.w += ad * (np.linalg.norm(dw) / dt - self.w)
        self.R = self.R @ _rotvec_to_R(_alpha(c["rot_min_cutoff_hz"] + c["rot_beta"] * self.w, dt) * dw)
        return self.p.copy(), self.R.copy()


# ---------------------------------------------------------------- input
class XrSample:
    __slots__ = ("ts_ns", "pos", "quat", "trigger", "buttons")

    def __init__(self, ts_ns, pos, quat, trigger, buttons=None):
        self.ts_ns, self.pos, self.quat, self.trigger = int(ts_ns), np.asarray(pos, float), np.asarray(quat, float), float(trigger)
        self.buttons = {k: bool(v) for k, v in (buttons or {}).items()}     # A / B / X / Y (missing = not pressed)


class XrInput:
    """XRoboToolkit binding (right hand). The binding only gives the latest values: old values when nothing new
    arrives, all zeros if nothing ever arrived."""

    def __init__(self):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as e:
            raise RuntimeError("cannot import xrobotoolkit_sdk: run with the XRoboToolkit venv python "
                               "(~/xrobotoolkit/venv/bin/python or XR_PYTHON, docs/teleop.md)") from e
        self.xrt = xrt
        xrt.init()

    def read(self):
        x = self.xrt
        for _ in range(5):                       # new data while reading -> read again (use it only if the stamp is unchanged)
            ts = x.get_time_stamp_ns()
            p = x.get_right_controller_pose()
            s = XrSample(ts, p[:3], p[3:7], x.get_right_trigger(),
                         {"A": x.get_A_button(), "B": x.get_B_button(), "X": x.get_X_button(), "Y": x.get_Y_button()})
            if x.get_time_stamp_ns() == ts:
                break
        return s

    def left_ok(self):
        """Whether the left controller pose arrives (X / Y are on the left). Not sent by the headset app -> zero quaternion."""
        q = np.asarray(self.xrt.get_left_controller_pose()[3:7], float)
        return abs(np.linalg.norm(q) - 1.0) < 0.05

    def close(self):
        self.xrt.close()


def wait_xr(inp, timeout):
    """Check that headset data really arrives: timestamp increasing and quaternion norm near 1. Else error (no start on zeros)."""
    t_end = time.monotonic() + timeout
    first, n_new, s = None, 0, None
    while time.monotonic() < t_end:
        s = inp.read()
        if s.ts_ns != 0 and abs(np.linalg.norm(s.quat) - 1.0) < 0.05:
            if first is None:
                first = s.ts_ns
            elif s.ts_ns != first:
                n_new += 1
                first = s.ts_ns
                if n_new >= 5:
                    return
        time.sleep(0.02)
    why = ("all values 0 (PC service not running, headset not connected, or Send not pressed in the headset app)"
           if s is None or s.ts_ns == 0 else
           f"timestamp not increasing or quaternion norm {np.linalg.norm(s.quat):.3f} (headset app Send / controller tracking)")
    raise RuntimeError(f"no headset data within {timeout:g} s: {why}")


def wait_robot_description(topic, timeout=20.0):
    """URDF text from the latched robot_description topic (own short-lived node). rclpy must be initialised."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    node = rclpy.create_node("xr_teleop_urdf")
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, topic, lambda m: got.append(m.data), qos)
    t_end = time.monotonic() + timeout
    try:
        while not got and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
    if not got:
        raise RuntimeError(f"no {topic} within {timeout:g} s (is the sim running?)")
    return got[0]


# ---------------------------------------------------------------- node
class XrTeleop:
    """Control loop (own thread, wall rate_hz) + ROS subscriptions (rclpy spin thread). inp: read() -> XrSample."""

    def __init__(self, cfg, inp, urdf_xml, log=lambda m: print(m, flush=True)):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String

        self.cfg, self.inp, self.log = cfg, inp, log
        self.model = km.ArmModel(urdf_xml)
        self.JointState, self.Float64, self.String = JointState, Float64, String
        self.node = Node("xr_teleop")
        self.lock = threading.Lock()
        self.q_meas, self.t_js = None, None          # latest /joint_states (6 arm joints, ARM_JOINTS order), wall receive time
        self.clock_msg, self.t_clock = None, None    # latest /clock (builtin Time), wall receive time
        self.pstop = False
        self.reset_hold = False
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)   # latest only
        t = cfg["topics"]
        self.node.create_subscription(JointState, t["joint_states"], self._on_js, latest)
        self.node.create_subscription(Clock, t["clock"], self._on_clock, latest)
        self.node.create_subscription(Bool, t["protective_stop"], self._on_pstop, latest)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool, t["reset_hold"], self._on_hold, latched)
        self.pub_arm = self.node.create_publisher(JointState, t["joint_command"], 10)
        self.pub_grip = self.node.create_publisher(Float64, t["gripper_command"], 10)
        self.pub_req = self.node.create_publisher(String, t["request"], 10)       # recorder requests (not recorded)
        self.dt = 1.0 / float(cfg["rate_hz"])
        self.max_dq = float(cfg["max_joint_speed"]) * self.dt
        # state
        self.engaged = False
        self.q_cmd = None
        self.ref = None                              # (p_ctrl0, R_ctrl0, M, T_tcp0)
        self.filt = OneEuroPose(cfg["filter"]) if cfg["filter"]["enabled"] else None
        self._t_filt = None                          # monotonic time of the last filter input
        self.grip_closed = False
        self.tb = cfg["teleop_button"]
        self.btn_prev = {b: True for b in BUTTONS}   # buttons held when the node starts are not a press
        self.n_req = {r: 0 for r in cfg["buttons"]}
        self.xr_ts, self.t_xr = None, time.monotonic()
        self.n_sent = 0
        self.last = None                             # last IkStep
        self._warn_t = {}
        self._t_status = 0.0
        self._stop = threading.Event()
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._rclpy = rclpy

    # -- ROS callbacks (spin thread) --
    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        try:
            q = np.array([d[n] for n in km.ARM_JOINTS], dtype=float)
        except KeyError as e:
            self._warn("js_name", f"joint {e} missing in /joint_states (names {list(m.name)})", 5.0)
            return
        with self.lock:
            self.q_meas, self.t_js = q, time.monotonic()

    def _on_clock(self, m):
        with self.lock:
            self.clock_msg, self.t_clock = m.clock, time.monotonic()

    def _on_pstop(self, m):
        with self.lock:
            self.pstop = bool(m.data)

    def _on_hold(self, m):
        with self.lock:
            self.reset_hold = bool(m.data)

    def _spin_loop(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)
        while not self._stop.is_set() and self._rclpy.ok():
            ex.spin_once(timeout_sec=0.05)

    # -- helpers --
    def _warn(self, key, msg, every=2.0):
        now = time.monotonic()
        if now - self._warn_t.get(key, -1e9) >= every:
            self._warn_t[key] = now
            self.log(f"[xr_teleop] notice: {msg}")

    def _state(self):
        with self.lock:
            return ((None if self.q_meas is None else self.q_meas.copy()), self.t_js, self.clock_msg, self.t_clock,
                    self.pstop, self.reset_hold)

    def _state_problem(self, now, t_js, t_clock, pstop, hold=False):
        stale = float(self.cfg["state_stale"])
        if t_js is None or now - t_js > stale:
            return f"/joint_states missing for more than {stale:g} s (check the sim)"
        if self.cfg["stamp"] == "sim" and (t_clock is None or now - t_clock > stale):
            return f"/clock missing for more than {stale:g} s (stamp: sim)"
        if hold:
            return "episode reset in progress (/ocean_sim2sim/hold_state true). Start again after the reset"
        if pstop:
            return "protective stop (/protective_stop true). Start again after a reset"
        return None

    def _stamp(self, clock_msg):
        if self.cfg["stamp"] == "sim":
            return clock_msg
        return self.node.get_clock().now().to_msg()

    def _request(self, req, btn):
        self.pub_req.publish(self.String(data=req))
        self.n_req[req] += 1
        what = {"record": "record start / stop", "reset": "episode reset (discards a running recording)",
                "drone_kill": "drone motors off"}[req]
        if self.pub_req.get_subscription_count() == 0:
            self.log(f"\a[xr_teleop] {btn} button -> {req} ({what}) sent, but no recorder is listening "
                     "(ros2 run ocean_sim2sim_teleop record_toggle.py <task name> --sim)")
        else:
            self.log(f"[xr_teleop] {btn} button -> {req} ({what}) request")

    # -- start / stop --
    def _engage(self, s, q_meas):
        M, elev = heading_map(quat_to_R(s.quat), float(self.cfg["max_start_elevation_deg"]))
        if M is None:
            self.log(f"\a[xr_teleop] start refused: controller tip {elev:+.0f} deg from horizontal (limit "
                     f"+-{self.cfg['max_start_elevation_deg']:g} deg). Point the controller forward, then {self.cfg['teleop_button']}")
            return
        self.q_cmd = q_meas.copy()
        self.ref = (s.pos.copy(), quat_to_R(s.quat), M, self.model.fk(self.q_cmd))
        if self.filt is not None:                    # filter starts at the current value (no pull from old values)
            self.filt.reset(s.pos, quat_to_R(s.quat))
            self._t_filt = time.monotonic()
        self.engaged, self.last = True, None
        self._t_status = time.monotonic()
        p = self.ref[3][:3, 3]
        self.log(f"\a[xr_teleop] start: TCP (base) [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m, controller tip elevation {elev:+.0f} deg. "
                 f"{self.tb} again = stop")

    def _disengage(self, why):
        self.engaged, self.ref = False, None
        self.log(f"\a[xr_teleop] stopped: {why} ({self.n_sent} commands sent so far). {self.tb} = start again from the "
                 "current pose (a running recording continues)")

    # -- one cycle --
    def tick(self):
        now = time.monotonic()
        s = self.inp.read()
        if s.ts_ns != self.xr_ts:
            self.xr_ts, self.t_xr = s.ts_ns, now
        elif now - self.t_xr > float(self.cfg["xr_stale_warn"]):
            self._warn("xr_stale", f"headset data unchanged for {now - self.t_xr:.1f} s (check the connection; the arm stays "
                                   "at the last target)", 2.0)
        edge = {b: s.buttons.get(b, False) and not self.btn_prev[b] for b in BUTTONS}   # press moment
        self.btn_prev = {b: s.buttons.get(b, False) for b in BUTTONS}
        a_edge = edge[self.tb]
        for req, btn in self.cfg["buttons"].items():  # recorder requests: also when not teleoperating
            if edge[btn]:
                self._request(req, btn)
        q_meas, t_js, clock_msg, t_clock, pstop, hold = self._state()
        problem = self._state_problem(now, t_js, t_clock, pstop, hold)

        if not self.engaged:
            if a_edge:
                if problem:
                    self.log(f"\a[xr_teleop] start refused: {problem}")
                elif abs(np.linalg.norm(s.quat) - 1.0) > 0.05:
                    self.log(f"\a[xr_teleop] start refused: bad controller pose (quaternion norm {np.linalg.norm(s.quat):.3f})")
                else:
                    self._engage(s, q_meas)
            if not self.engaged:
                return

        # teleoperating
        if a_edge and self.last is not None:          # the press of the starting cycle is not a stop
            return self._disengage(f"{self.tb} button")
        if problem:
            return self._disengage(problem)
        dev = float(np.max(np.abs(q_meas - self.q_cmd)))
        if dev > float(self.cfg["desync"]):
            return self._disengage(f"measured joints {math.degrees(dev):.1f} deg away from the joint target (a reset moved "
                                   "the arm or the arm stopped)")

        p0, R0, M, T0 = self.ref
        p_h, R_h = s.pos, quat_to_R(s.quat)          # hand (controller) pose
        if self.filt is not None:
            p_h, R_h = self.filt(p_h, R_h, now - self._t_filt)
            self._t_filt = now
        p_t = T0[:3, 3] + float(self.cfg["scale"]) * (M @ (p_h - p0))
        R_t = (M @ (R_h @ R0.T) @ M.T) @ T0[:3, :3] if self.cfg["rotation"] else T0[:3, :3]
        st = km.ik_step(self.model, self.q_cmd, q_meas, p_t, R_t, max_dq=self.max_dq, damping=float(self.cfg["damping"]),
                        max_lead=float(self.cfg["max_lead"]), limit_margin=float(self.cfg["limit_margin"]))
        # mount protection: a target that brings a wrist joint origin further below the limit height is not sent
        z_new = float(self.model.joint_origins(st.q)[3:, 2].min())
        if z_new < float(self.cfg["min_wrist_height"]) and z_new < float(self.model.joint_origins(self.q_cmd)[3:, 2].min()):
            self._warn("table", f"wrist close to the mount (joint origin height {z_new * 100:.1f} cm < "
                                f"{float(self.cfg['min_wrist_height']) * 100:g} cm). Not going lower: raise the hand")
            st.q = self.q_cmd.copy()
        self.q_cmd, self.last = st.q, st
        m = self.JointState()
        m.header.stamp = self._stamp(clock_msg)
        m.name = list(km.ARM_JOINTS)
        m.position = self.q_cmd.tolist()
        self.pub_arm.publish(m)
        self.n_sent += 1

        g = self.cfg["gripper"]
        want = True if s.trigger > float(g["close_above"]) else False if s.trigger < float(g["open_below"]) else self.grip_closed
        if want != self.grip_closed or self.n_sent == 1:
            self.grip_closed = want
            self.pub_grip.publish(self.Float64(data=float(g["raw_close"] if want else g["raw_open"])))

        lim = [x for x in st.limited if x.endswith(":limit")]
        if lim:
            self._warn("limit", f"joint limit: {', '.join(x.split(':')[0] for x in lim)}")
        if st.manip < float(self.cfg["warn_manip"]):
            self._warn("manip", f"near a singularity (index {st.manip:.4f})")
        if st.pos_err > float(self.cfg["warn_pos_err"]):
            self._warn("lag", f"TCP target {st.pos_err * 100:.0f} cm ahead of the arm (hand faster than the arm or unreachable)")
        if now - self._t_status >= float(self.cfg["status_sec"]):
            self._t_status = now
            self.log(f"[xr_teleop] teleoperating: {self.n_sent} commands, TCP target error {st.pos_err * 1000:.1f} mm / "
                     f"{math.degrees(st.rot_err):.1f} deg, singularity index {st.manip:.3f}, gripper "
                     f"{'close' if self.grip_closed else 'open'}")

    # -- run --
    def start(self):
        self._spin.start()

    def run(self, should_stop=lambda: False):
        """tick at rate_hz. A late cycle is not caught up (no burst), the next cycle follows."""
        self.start()
        nxt = time.monotonic()
        while not should_stop() and self._rclpy.ok():
            self.tick()
            nxt += self.dt
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def shutdown(self):
        self._stop.set()
        if self._spin.is_alive():
            self._spin.join(timeout=1.0)
        self.node.destroy_node()
