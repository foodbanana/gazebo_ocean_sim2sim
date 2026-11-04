#!/usr/bin/env python3
# =============================================================
# check_xr_teleop.py - test the teleop node (ocean_sim2sim_teleop/xr_teleop.py) without the headset (docs/teleop.md)
#
#   Ported from: data_collection_ur5_gripper teleop/check_xr_teleop.py - same scripted controller and checks.
#   The script makes the controller input (ScriptedInput) and feeds it to the real node code (XrTeleop) against the
#   running sim. TCP pose = FK of /joint_states (ik_model). The scripted controller is turned 35 deg from the frame
#   axes and its tip raised 20 deg ("forward" not along an axis). "teleop button" = config teleop_button (Y).
#     1  vertical refused   tip raised 80 deg + teleop button -> no command
#     2  start              teleop button -> commands, TCP unchanged while the hand is still
#     3  forward 10 cm      -> TCP base +x 10 cm x scale
#     4  up 8 cm, right 5 cm -> TCP +z 8 cm, -y 5 cm (x scale)
#     5  turn left 20 deg   -> TCP +20 deg about base z, position unchanged
#     5b wrist twist +-25 deg -> TCP turns 25 deg, no protective stop
#     6  gripper            trigger 1.0 -> /gripper/target 1150, 0.5 -> unchanged (hysteresis), 0.0 -> 0
#     7  command interval   received interval (wall) median ~ 1 / rate_hz, no zero / backwards stamp
#     7b filter             still hand + tremor (1 mm, 0.3 deg std) -> commanded TCP jitter with filter < 0.7 x without
#     8  stop / restart     teleop button -> no commands; hand moved 30 cm, teleop button -> TCP does not jump
#     9  reset              /sim/reset while teleoperating -> node stops by itself, arm at the start pose afterwards
#    10  protective stop    /protective_stop true 0 times during the whole test
#    11  button requests    buttons of the config (A / B / X) pressed 0.5 s -> each request once on /teleop/request
#
#   Gazebo note: the boat floats, so TCP checks use the robot base frame (FK of /joint_states), as the node does.
#   Run (sim first; calm water keeps the base still enough for the mm checks):
#       ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm      # other terminal
#       ros2 run ocean_sim2sim_teleop check_xr_teleop.py
#   Result: console + ~/.ros/ocean_sim2sim_teleop/check_xr_teleop_<time>.txt. Any FAIL -> exit code 1
#
#   Changes from the Isaac Sim file: URDF from /robot_description, messages in English, report directory.
# =============================================================

import argparse
import datetime
import math
import os
import sys
import threading
import time

import numpy as np

from ocean_sim2sim_teleop import ik_model as km
from ocean_sim2sim_teleop import xr_teleop as tn

LINES, RESULTS = [], []


def log(s=""):
    print(s, flush=True)
    LINES.append(s)


def result(name, ok, detail):
    RESULTS.append((name, bool(ok)))
    log(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class ScriptedInput:
    """Scripted controller. Position in operator terms (forward, right, up), orientation by yaw / pitch. The stamp grows at every read."""

    def __init__(self, teleop_button, heading_deg=35.0, pitch_deg=20.0):
        self.tb = teleop_button
        self.h = math.radians(heading_deg)
        self.lock = threading.Lock()
        self.fru = np.zeros(3)             # forward / right / up [m]
        self.yaw, self.pitch = 0.0, math.radians(pitch_deg)   # yaw: left about up + [rad], pitch: tip raised + [rad]
        self.roll = 0.0                    # wrist twist: clockwise seen by the operator + [rad] (about the controller forward axis)
        self.trigger, self.a = 0.0, False   # a: teleop button pressed
        self.buttons = {}                  # other buttons {'B': True, ...}
        self.noise_pos, self.noise_rot = 0.0, 0.0   # hand tremor: position [m] / orientation [rad] std
        self.rng = np.random.default_rng(0)
        Rh = Ry(self.h)
        self.fwd = Rh @ np.array([0.0, 0.0, -1.0])
        self.right = np.cross(self.fwd, np.array([0.0, 1.0, 0.0]))
        self.origin = np.array([0.1, 1.2, -0.3])

    def read(self):
        with self.lock:
            pos = self.origin + self.fru[0] * self.fwd + self.fru[1] * self.right + self.fru[2] * np.array([0.0, 1.0, 0.0])
            R = Ry(self.h + self.yaw) @ Rx(self.pitch) @ Rz(-self.roll)
            if self.noise_pos > 0 or self.noise_rot > 0:
                pos = pos + self.rng.normal(0, self.noise_pos, 3)
                w = self.rng.normal(0, self.noise_rot, 3)
                a = np.linalg.norm(w)
                R = R @ (km.axis_angle_R(w / a, a) if a > 1e-12 else np.eye(3))
            return tn.XrSample(time.monotonic_ns(), pos, km.R_to_quat(R), self.trigger, {**self.buttons, self.tb: self.a})

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


def main():
    ap = argparse.ArgumentParser(description="teleop node test (scripted controller input, running sim)")
    ap.add_argument("--config", default=tn.default_config())
    ap.add_argument("--report-dir", default=os.path.expanduser("~/.ros/ocean_sim2sim_teleop"))
    args = ap.parse_args()
    cfg = tn.load_config(args.config)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    rclpy.init()
    urdf = tn.wait_robot_description(cfg["topics"]["robot_description"])
    model = km.ArmModel(urdf)
    n = Node("check_xr_teleop")
    st = {"q": None, "cmd": [], "grip_t": None, "pstop_true": 0, "pstop": None, "req": []}

    def on_js(m):
        d = dict(zip(m.name, m.position))
        st["q"] = np.array([d[k] for k in km.ARM_JOINTS])

    def on_cmd(m):
        st["cmd"].append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array(m.position)))

    def on_ps(m):
        st["pstop"] = m.data
        st["pstop_true"] += int(m.data)

    t = cfg["topics"]
    n.create_subscription(JointState, t["joint_states"], on_js, 10)
    n.create_subscription(JointState, t["joint_command"], on_cmd, 1000)
    n.create_subscription(JointState, "/gripper/target", lambda m: st.__setitem__("grip_t", m.position[0]), 10)
    n.create_subscription(Bool, t["protective_stop"], on_ps, 10)
    n.create_subscription(String, t["request"], lambda m: st["req"].append(m.data), 10)
    reset_cli = n.create_client(Trigger, "/sim/reset")

    def pump(sec, fn=None):
        """Spin the test node for sec. fn(u) (u = 0 -> 1, minimum jerk) moves the scripted controller."""
        t0 = time.monotonic()
        while True:
            u = min((time.monotonic() - t0) / sec, 1.0)
            if fn is not None:
                fn(10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
            rclpy.spin_once(n, timeout_sec=0.004)
            if u >= 1.0:
                return

    def reset():
        f = reset_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not f.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t0 > 240:
                raise RuntimeError("/sim/reset: no answer (240 s)")
        if not f.result().success:
            raise RuntimeError(f"/sim/reset failed: {f.result().message[:300]}")

    log(f"check_xr_teleop  {datetime.datetime.now().isoformat(timespec='seconds')}  config {cfg['path']}")
    t0 = time.monotonic()
    while st["q"] is None or not reset_cli.service_is_ready():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - t0 > 20:
            sys.exit("[ERROR] no sim (/joint_states, /sim/reset). Launch wamv_arm.launch.py first")
    reset()
    pump(1.0)
    q_init = st["q"].copy()
    T_init = model.fk(q_init)

    inp = ScriptedInput(cfg["teleop_button"])
    node_log = []
    tele = tn.XrTeleop(cfg, inp, urdf, log=lambda m: (node_log.append(m), print("    " + m.replace("\a", ""), flush=True)))
    stop = threading.Event()
    th = threading.Thread(target=lambda: tele.run(stop.is_set), daemon=True)
    th.start()
    pump(1.0)

    def press_a():
        inp.set(a=True)
        pump(0.15)
        inp.set(a=False)
        pump(0.15)

    def tcp():
        return model.fk(st["q"])

    try:
        # 1 vertical refused
        n0 = len(st["cmd"])
        inp.set(pitch=math.radians(80))
        press_a()
        pump(1.0)
        result("1 vertical refused", len(st["cmd"]) == n0 and not tele.engaged, f"tip raised 80 deg + teleop button -> {len(st['cmd']) - n0} commands (expected 0)")
        inp.set(pitch=math.radians(20))

        # 2 start
        press_a()
        pump(1.5)
        d = np.linalg.norm(tcp()[:3, 3] - T_init[:3, 3])
        result("2 start", tele.engaged and len(st["cmd"]) - n0 > 50 and d < 1e-3,
               f"teleop button -> {len(st['cmd']) - n0} commands (1.8 s), TCP motion with the hand still {d * 1000:.2f} mm (expected < 1)")
        i_start = n0

        # 3 forward 10 cm
        lead = [0.0]

        def track():
            if st["cmd"]:
                lead[0] = max(lead[0], float(np.max(np.abs(st["cmd"][-1][2] - st["q"]))))

        pump(2.0, lambda u: (inp.set(fru=np.array([0.10 * u, 0, 0])), track()))
        pump(1.5)
        dp = tcp()[:3, 3] - T_init[:3, 3]
        k = float(cfg["scale"])                # hand motion x scale = TCP motion
        result("3 forward 10 cm", np.linalg.norm(dp - k * np.array([0.10, 0, 0])) < 3e-3,
               f"TCP motion (base) [{dp[0] * 1000:+.1f}, {dp[1] * 1000:+.1f}, {dp[2] * 1000:+.1f}] mm (expected [{100 * k:+.0f}, 0, 0] +- 3, scale {k:g}), max command - measurement while moving {math.degrees(lead[0]):.2f} deg")

        # 4 up 8 cm, right 5 cm
        pump(2.0, lambda u: inp.set(fru=np.array([0.10, 0.05 * u, 0.08 * u])))
        pump(1.5)
        dp = tcp()[:3, 3] - T_init[:3, 3]
        result("4 up 8 cm, right 5 cm", np.linalg.norm(dp - k * np.array([0.10, -0.05, 0.08])) < 3e-3,
               f"TCP motion (base) [{dp[0] * 1000:+.1f}, {dp[1] * 1000:+.1f}, {dp[2] * 1000:+.1f}] mm (expected [{100 * k:+.0f}, {-50 * k:+.0f}, {80 * k:+.0f}] +- 3)")

        # 5 turn left 20 deg
        p_before = tcp()[:3, 3].copy()
        pump(2.0, lambda u: inp.set(yaw=math.radians(20) * u))
        pump(1.5)
        T = tcp()
        w = km.R_to_rotvec(T[:3, :3] @ T_init[:3, :3].T)
        dpos = np.linalg.norm(T[:3, 3] - p_before)
        result("5 turn left 20 deg", abs(math.degrees(w[2]) - 20) < 1.0 and np.linalg.norm(w[:2]) < math.radians(1.0) and dpos < 3e-3,
               f"TCP rotation vector (base) [{math.degrees(w[0]):+.2f}, {math.degrees(w[1]):+.2f}, {math.degrees(w[2]):+.2f}] deg (expected [0, 0, +20] +- 1), position change {dpos * 1000:.2f} mm (expected < 3)")

        # 5b wrist twist +-25 deg (Isaac Sim 2026-10-12: with the first start pose 10 deg made the wrist mount hit the forearm)
        T_a = tcp()
        pump(1.0, lambda u: inp.set(roll=math.radians(25) * u))
        pump(1.0)
        w_cw = km.R_to_rotvec(tcp()[:3, :3] @ T_a[:3, :3].T)
        pump(2.0, lambda u: inp.set(roll=math.radians(25 - 50 * u)))
        pump(1.0)
        w_ccw = km.R_to_rotvec(tcp()[:3, :3] @ T_a[:3, :3].T)
        pump(1.0, lambda u: inp.set(roll=math.radians(-25 + 25 * u)))
        pump(1.0)
        a_cw, a_ccw = math.degrees(np.linalg.norm(w_cw)), math.degrees(np.linalg.norm(w_ccw))
        result("5b wrist twist +-25 deg", abs(a_cw - 25) < 1.5 and abs(a_ccw - 25) < 1.5 and st["pstop_true"] == 0 and tele.engaged,
               f"TCP rotation {a_cw:.1f} / {a_ccw:.1f} deg (expected 25 +- 1.5), protective stops {st['pstop_true']}, teleop {'on' if tele.engaged else 'stopped'}")

        # 6 gripper
        inp.set(trigger=1.0)
        pump(1.0)
        g1 = st["grip_t"]
        inp.set(trigger=0.5)
        pump(0.7)
        g2 = st["grip_t"]
        inp.set(trigger=0.0)
        pump(1.0)
        g3 = st["grip_t"]
        result("6 gripper", g1 == 1150.0 and g2 == 1150.0 and g3 == 0.0, f"/gripper/target: trigger 1.0 -> {g1}, 0.5 -> {g2}, 0.0 -> {g3} (expected 1150, 1150, 0)")

        # 7 command interval
        c = st["cmd"][i_start:]
        dw = np.diff([x[0] for x in c]) * 1000
        stamps = np.array([x[1] for x in c])
        dq = np.max(np.abs(np.diff(np.array([x[2] for x in c]), axis=0)))
        ds = np.diff(stamps) * 1000
        ok7 = abs(np.median(dw) - 1000 / cfg["rate_hz"]) < 2 and np.all(stamps > 0) and np.all(ds >= 0) and dq <= tele.max_dq + 1e-9
        result("7 command interval", ok7,
               f"{len(c)} commands, received interval (wall) median {np.median(dw):.2f} ms, 95 % {np.percentile(dw, 95):.2f}, max {dw.max():.2f} (expected median {1000 / cfg['rate_hz']:.2f} +- 2), "
               f"zero stamps {int(np.sum(stamps <= 0))}, max stamp interval {ds.max():.1f} ms (above 40 ms: {int(np.sum(ds > 40))}), backwards {int(np.sum(ds < 0))}, "
               f"max joint change per cycle {dq:.4f} rad (limit {tele.max_dq:.4f})")

        # 7b filter: same tremor, commanded TCP jitter with the filter on / off
        if cfg["filter"]["enabled"]:
            def cmd_jitter(sec):
                n0 = len(st["cmd"])
                pump(sec)
                P = np.array([model.fk(c[2])[:3, 3] for c in st["cmd"][n0:]])
                return float(np.mean(np.std(P, axis=0))) * 1000, len(P)
            inp.set(noise_pos=0.001, noise_rot=math.radians(0.3))
            pump(0.5)
            j_on, n_on = cmd_jitter(2.0)
            filt, tele.filt = tele.filt, None      # filter off for a moment
            j_off, n_off = cmd_jitter(2.0)
            s0 = inp.read()                         # back on: initialise with the current value (as at the node start)
            filt.reset(s0.pos, tn.quat_to_R(s0.quat))
            tele._t_filt = time.monotonic()
            tele.filt = filt
            inp.set(noise_pos=0.0, noise_rot=0.0)
            pump(1.0)
            result("7b filter", j_on < 0.7 * j_off and tele.engaged,
                   f"hand tremor 1 mm / 0.3 deg -> commanded TCP jitter (mean per-axis std): filter on {j_on:.3f} mm ({n_on}) / off {j_off:.3f} mm ({n_off}) = {j_on / j_off:.2f} x (expected < 0.7)")

        # 8 stop / restart
        press_a()
        n1 = len(st["cmd"])
        p_stop = tcp()[:3, 3].copy()
        pump(1.0, lambda u: inp.set(fru=np.array([0.10 - 0.30 * u, 0.05, 0.08])))
        stopped = len(st["cmd"]) - n1
        press_a()
        pump(1.5)
        jump = np.linalg.norm(tcp()[:3, 3] - p_stop)
        result("8 stop / restart", stopped == 0 and tele.engaged and jump < 2e-3,
               f"teleop button -> {stopped} commands in the stopped 1 s (expected 0). Hand moved 30 cm, again -> TCP motion {jump * 1000:.2f} mm (expected < 2)")

        # 9 reset
        reset()
        pump(0.5)
        n2 = len(st["cmd"])
        pump(2.0)
        err = math.degrees(float(np.max(np.abs(st["q"] - q_init))))
        result("9 reset", not tele.engaged and len(st["cmd"]) == n2 and err < 1.0 and st["pstop"] is False,
               f"/sim/reset while teleoperating -> node teleop {'on' if tele.engaged else 'stopped'}, {len(st['cmd']) - n2} commands in 2 s after the reset (expected 0), "
               f"arm {err:.3f} deg from the start pose (expected < 1), protective stop {st['pstop']}")

        # 11 button requests (teleop stopped after the reset: requests must go out also when not teleoperating)
        got = {}
        btns = sorted(cfg["buttons"].values())
        for btn in btns:
            n3 = len(st["req"])
            inp.set(buttons={btn: True})
            pump(0.5)
            inp.set(buttons={})
            pump(0.3)
            got[btn] = st["req"][n3:]
        want = {b: [r for r, bb in cfg["buttons"].items() if bb == b] for b in btns}
        result("11 button requests", got == want and not tele.engaged,
               f"0.5 s press -> /teleop/request: {', '.join(f'{b} {got[b]}' for b in btns)} (expected {', '.join(f'{b} {want[b]}' for b in btns)}), "
               f"teleop {'on' if tele.engaged else 'still stopped'}")

        # 10 protective stop
        result("10 protective stop", st["pstop_true"] == 0, f"/protective_stop true {st['pstop_true']} times (expected 0)")
    finally:
        stop.set()
        th.join(timeout=2.0)
        tele.shutdown()

    ok = all(r for _, r in RESULTS)
    log(f"\nresult: {sum(r for _, r in RESULTS)}/{len(RESULTS)} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_xr_teleop_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(LINES) + "\n\n[node output]\n" + "\n".join(m.replace("\a", "") for m in node_log) + "\n")
    print(f"report: {path}", flush=True)
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
