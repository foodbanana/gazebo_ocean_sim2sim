#!/usr/bin/env python3
# =============================================================
# check_rc.py - transmitter hand-over test without the transmitter (docs/teleop.md)
#
#   Same checks as the Isaac Sim fake-serial run of sim_ros2.py --drone-rc on (docs/drone_teleop.md 8, 10/10):
#   this script makes a pseudo terminal that looks like the Pico serial device (symlink --link) and writes Pico lines
#   ("RC,<status 1>,<status 2>,<8 channels>", 142.9 Hz) from scripted stick values. The sim reads it as the real device.
#
#   1. start this script (it creates the device and waits), 2. launch the sim with that device:
#       ros2 run ocean_sim2sim_drone check_rc.py --link /tmp/fake_rc        # terminal 1 (any free path)
#       ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm drone_motion:=rc rc_port:=/tmp/fake_rc
#
#   C1  hand-over after the air start (sticks centred): POSCTL, rc.manual true
#   C2  holds where handed over: HOLD_SEC, drift < HOLD_TOL
#   C3  right stick up (pitch +PUSH): drone moves forward (body x); stops after release
#   C4  right stick right (roll +PUSH): body right (-y)
#   C5  left stick up (throttle +PUSH): up
#   C6  left stick right (yaw +PUSH): yaw right (ENU yaw decreases)
#   C7  transmitter off (status values 1, 1): taken back within TAKEBACK_SEC (rc.manual false), PX4 OFFBOARD within
#       MODE_SEC (the mode arrives with the 1 Hz heartbeat), stays (drift < HOLD_TOL)
#   C8  signal back, centred: handed over again
#   C9  Pico lines stop (stale): back to OFFBOARD; lines again: handed over again
#   C10 /sim/reset with the right stick pushed: not handed over; centred -> handed over
#   C11 /sim/drone_kill in manual mode: disarmed
# =============================================================
import argparse
import json
import math
import os
import pty
import sys
import threading
import time
import tty

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ocean_sim2sim_drone import rc_input

PUSH, PUSH_SEC, STOP_SEC = 0.6, 2.0, 2.5
HOLD_SEC, HOLD_TOL = 10.0, 0.10
TAKEBACK_SEC, MODE_SEC = 0.5, 2.0       # take-back decision (rc.manual false) / PX4 reports OFFBOARD (heartbeat 1 Hz)
MOVE_MIN = 0.10                     # [m] stick push PUSH for PUSH_SEC must move at least this much along the axis
STOP_SPEED = 0.05                   # [m/s] after the release
YAW_MIN = 15.0                      # [deg]

RESULTS = []


def log(msg=""):
    print(msg, flush=True)


def result(no, title, ok, why=""):
    RESULTS.append((no, title, ok))
    log(f"  {no} {title}: {'PASS' if ok else 'FAIL'}" + (f" ({why})" if why else ""))


class FakePico:
    """Pseudo terminal writing Pico lines at rate_hz from the current sticks / status / on-off."""

    def __init__(self, cfg, link, rate_hz=142.9):
        self.cfg = cfg
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.slave_name = os.ttyname(slave)
        self._slave = slave                 # keep open: the pty stays usable while the bridge reopens it
        if os.path.lexists(link):
            if not os.path.islink(link):
                raise RuntimeError(f"{link} exists and is not a symlink")
            os.remove(link)
        os.symlink(self.slave_name, link)
        self.link = link
        self.sticks = (0.0, 0.0, 0.0, 0.0)
        self.status = (0, 0)
        self.sending = True
        self.period = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def raw(self, name, x):
        ax = self.cfg["axes"][name]
        x *= ax["sign"]
        span = (ax["max"] - ax["center"]) if x >= 0 else (ax["center"] - ax["min"])
        return int(round(ax["center"] + x * span))

    def line(self):
        vals = [0] * rc_input.N_VALUES
        for i, f in zip(self.cfg["lost_flags"], self.status):
            vals[i - 1] = f
        for name, x in zip(rc_input.AXES, self.sticks):
            vals[self.cfg["axes"][name]["index"] - 1] = self.raw(name, x)
        for i in range(rc_input.N_VALUES):          # unused channels as the Pico sends them
            if vals[i] == 0 and (i + 1) not in self.cfg["lost_flags"] and i >= 6:
                vals[i] = 200
        return "RC," + ",".join(str(v) for v in vals) + "\r\n"

    def _run(self):
        t_next = time.monotonic()
        while not self._stop.is_set():
            if self.sending:
                try:
                    os.write(self.master, self.line().encode())
                except OSError:
                    pass
            t_next += self.period
            time.sleep(max(0.0, t_next - time.monotonic()))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            os.remove(self.link)
        except OSError:
            pass
        os.close(self.master)
        os.close(self._slave)


class Watch(Node):
    def __init__(self):
        super().__init__("check_rc")
        self.st = None
        self.create_subscription(String, "/drone/px4/state", self._on_state, 10)
        self.reset_cli = self.create_client(Trigger, "/sim/reset")
        self.kill_cli = self.create_client(Trigger, "/sim/drone_kill")

    def _on_state(self, m):
        self.st = json.loads(m.data)


def main():
    ap = argparse.ArgumentParser(description="transmitter hand-over test with a fake Pico serial device")
    ap.add_argument("--link", required=True, help="symlink path for the fake device (give it to the launch as rc_port)")
    ap.add_argument("--config", default=os.path.join(get_package_share_directory("ocean_sim2sim_drone"), "config", "rc_input.yaml"))
    ap.add_argument("--start-timeout", type=float, default=600.0, help="wall s to wait for the first hand-over")
    a = ap.parse_args()
    cfg = rc_input.load_config(a.config)
    pico = FakePico(cfg, os.path.abspath(os.path.expanduser(a.link)))
    log(f"[check_rc] fake Pico on {pico.slave_name} -> {pico.link}. Launch now with rc_port:={pico.link}")
    rclpy.init()
    n = Watch()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def st():
        return n.st or {}

    def wait(cond, wall, step=0.02):
        t_end = time.monotonic() + wall
        while time.monotonic() < t_end:
            if cond(st()):
                return True
            time.sleep(step)
        return cond(st())

    def manual(s):
        return bool((s.get("rc") or {}).get("manual")) and s.get("mode") == "POSCTL"

    def offboard(s):
        return not (s.get("rc") or {}).get("manual") and s.get("mode") == "OFFBOARD"

    def pos(s=None):
        s = s or st()
        return np.array(s["true_position"]), float(s["true_yaw_deg"])

    def run_for(sim_sec):
        t0 = st().get("t") or 0.0
        wait(lambda s: (s.get("t") or 0.0) - t0 >= sim_sec, sim_sec * 20 + 10)

    def call(cli, name, timeout):
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{name} not available")
        fut = cli.call_async(Trigger.Request())
        t_end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.05)
        if not fut.done():
            raise RuntimeError(f"{name}: no answer in {timeout} s")
        return fut.result()

    def push(no, title, axis, check):
        k = rc_input.AXES.index(axis)
        p0, y0 = pos()
        s = [0.0] * 4
        s[k] = PUSH
        pico.sticks = tuple(s)
        run_for(PUSH_SEC)
        pico.sticks = (0.0, 0.0, 0.0, 0.0)
        p1, y1 = pos()
        run_for(STOP_SEC)
        p2, y2 = pos()
        run_for(0.5)
        p3, _ = pos()
        speed = float(np.linalg.norm(p3 - p2) / 0.5)
        c, sy = math.cos(math.radians(y0)), math.sin(math.radians(y0))
        d = p2 - p0
        body = np.array([c * d[0] + sy * d[1], -sy * d[0] + c * d[1], d[2]])        # world -> drone heading frame
        dyaw = (y2 - y0 + 180.0) % 360.0 - 180.0
        ok, why = check(body, dyaw)
        ok = ok and speed < STOP_SPEED and manual(st())
        result(no, title, ok, f"{why}, body move [{body[0]:+.3f} {body[1]:+.3f} {body[2]:+.3f}] m, yaw {dyaw:+.1f} deg, "
                              f"speed after release {speed:.3f} m/s")

    try:
        log("[check_rc] waiting for the scene (air start) and the first hand-over ...")
        if not wait(manual, a.start_timeout, 0.2):
            result("C1", "hand-over after the air start", False, f"state {json.dumps(st())[:400]}")
            raise SystemExit(1)
        result("C1", "hand-over after the air start", True, f"rc {st().get('rc')}")
        p0, _ = pos()
        run_for(HOLD_SEC)
        drift = float(np.linalg.norm(pos()[0] - p0))
        result("C2", f"holds {HOLD_SEC:g} s after the hand-over", drift < HOLD_TOL and manual(st()), f"drift {drift * 1000:.0f} mm")

        push("C3", "pitch + = forward", "pitch", lambda b, y: (b[0] > MOVE_MIN, "forward"))
        push("C4", "roll + = right", "roll", lambda b, y: (b[1] < -MOVE_MIN, "right"))
        push("C5", "throttle + = up", "throttle", lambda b, y: (b[2] > MOVE_MIN, "up"))
        push("C6", "yaw + = yaw right", "yaw", lambda b, y: (y < -YAW_MIN, "yaw right"))

        def takeback(no, title, lose, restore):
            t0 = st().get("t")
            lose()
            wait(lambda s: not (s.get("rc") or {}).get("manual"), 10.0, 0.005)
            dt_take = (st().get("t") or 0) - t0
            ok = wait(offboard, 10.0)
            dt_mode = (st().get("t") or 0) - t0
            p0, _ = pos()
            run_for(3.0)
            drift = float(np.linalg.norm(pos()[0] - p0))
            result(no, title, ok and dt_take <= TAKEBACK_SEC and dt_mode <= MODE_SEC and drift < HOLD_TOL and offboard(st()),
                   f"taken back after {dt_take:.2f} s, OFFBOARD after {dt_mode:.2f} s sim (state rate limits the resolution), "
                   f"then drift {drift * 1000:.0f} mm in 3 s")
            restore()
            ok = wait(manual, 15.0)
            return ok

        def lose_signal():
            pico.status = (1, 1)

        def back_signal():
            pico.status = (0, 0)

        ok = takeback("C7", "transmitter off -> Offboard hover", lose_signal, back_signal)
        result("C8", "signal back -> handed over again", ok, f"rc {st().get('rc')}")
        ok = takeback("C9", "Pico lines stop -> Offboard hover", lambda: setattr(pico, "sending", False),
                      lambda: setattr(pico, "sending", True))
        result("C9b", "lines again -> handed over again", ok, f"rc {st().get('rc')}")

        pico.sticks = (0.5, 0.0, 0.0, 0.0)
        log("[check_rc] /sim/reset with the right stick pushed ...")
        res = call(n.reset_cli, "/sim/reset", 300.0)
        if not res.success:
            result("C10", "reset with sticks pushed: no hand-over", False, res.message[:300])
        else:
            run_for(5.0)
            not_handed = not (st().get("rc") or {}).get("manual") and st().get("mode") == "OFFBOARD"
            pico.sticks = (0.0, 0.0, 0.0, 0.0)
            ok = wait(manual, 15.0)
            result("C10", "reset with sticks pushed: no hand-over, centred -> handed over", not_handed and ok,
                   f"5 s after the reset: mode {st().get('mode')}; then {st().get('rc')}")
        res = call(n.kill_cli, "/sim/drone_kill", 30.0)
        ok = res.success and wait(lambda s: not s.get("armed") and not s.get("link_armed"), 5.0)
        result("C11", "kill in manual mode", ok, res.message[:200])
    finally:
        n_ok = sum(ok for _, _, ok in RESULTS)
        log(f"[check_rc] {n_ok}/{len(RESULTS)} PASS")
        pico.close()
        ex.shutdown()
        spin.join(timeout=2.0)
        n.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if RESULTS and n_ok == len(RESULTS) else 1)


if __name__ == "__main__":
    main()
