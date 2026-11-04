# =============================================================
# commander.py - PX4 drone commands over MAVLink (offboard link) + CLI command server + hover reference
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/drone_cmd.py (PX4Commander, CommandServer, mode_name) and
#     isaacsim/scripts/drone_flight.py (Reference). The Isaac Sim PX4 link it belongs to is based on
#     livealive7/PegasusSimulator b1256ca (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto, LICENSE.PegasusSimulator).
#
#   PX4Commander: listens on the PX4 SITL offboard API link (PX4 -> UDP 14540 + instance), same MAVLink as a real drone
#     - position target = Offboard SET_POSITION_TARGET_LOCAL_NED (position + yaw), sent continuously at setpoint_hz in
#       sim time (PX4 keeps Offboard only above 2 Hz)
#     - fly_to(target): setpoint first -> request Offboard -> request arm (re-requested every retry_sec until done)
#     - goto(target, yaw) = new target, hold() = current estimate as target, land() = AUTO.LAND,
#       kill() = forced disarm in the air (MAV_CMD_COMPONENT_ARM_DISARM, param2 21196 -> motors stop at once)
#   Frames: world ENU [m] <-> PX4 local NED. PX4 local origin = where the EKF initialised (origin_enu, aligned at arm):
#     NED = (y - oy, x - ox, -(z - oz)), yaw_ned = pi/2 - yaw_enu. The PX4 estimate (estimate_enu) can differ from the
#     true pose (as on the real drone).
#   Time: tick(t) with sim time (lockstep: PX4 clock = sim time).
#   CLI (scripts/drone_cmd.py in another terminal): JSON over UDP localhost command_port (14600 + instance). The
#     commander keeps sending the setpoint; the CLI only changes it (companion computer + high level human commands).
#     hover mode: goto = moves the sine reference position only (sine continues), hold = freezes the sine at the current
#     target. land / kill are taken over by the CLI. Not hover: goto / hold take over the target (external, cleared by a
#     PX4 restart). Move commands are refused until the scene is ready (status "ready"); kill is always accepted.
#
#   Reference: target position / velocity / acceleration / jerk (world). static: p0. hover: p0 + per-axis sum of sines
#     (amplitude sum = preset amplitude). freeze(t): stop at the target of that time. reseed(seed, p0): new frequencies
#     and phases, unfreeze.
#
#   RC handover (Isaac Sim drone_cmd.py, docs/drone_teleop.md): set_sticks() then handover() -> PX4 manual mode (POSCTL),
#     sticks go out as MANUAL_CONTROL on the same link (position target / Offboard re-request stop). takeback() = Offboard
#     hold at the current estimate (signal lost). Once MANUAL_CONTROL has started it is never stopped (centre values while
#     not handed over): PX4 would go to the manual-control-loss failsafe. set_params(dict): runtime PX4 float parameters
#     (PARAM_SET, re-sent until PARAM_VALUE confirms, params_confirmed()).
#   Changes from the Isaac Sim files: trajectory mode removed (not implemented in Isaac either); messages and comments in
#     English; a RuntimeError inside a CLI command (hold before any estimate) is answered as an error instead of
#     propagating and stopping the bridge. Equations, rates, retry logic, message order and the CLI JSON protocol are identical.
# =============================================================

import json
import math
import socket

import numpy as np

mavutil = M = None


def _load_mavlink():
    """pymavlink only when a PX4Commander is created (the CLI does not need it)."""
    global mavutil, M
    if mavutil is None:
        from pymavlink import mavutil as mu
        mavutil, M = mu, mu.mavlink


MAIN_MODE = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 8: "FOLLOW", 9: "PRECLAND"}
POS_YAW_MASK = 0b0000_1001_1111_1000     # use position + yaw only (ignore velocity, acceleration, yaw rate)
KILL_MAGIC = 21196

MODES = ("static", "hover")
HOVER_KEYS = ("amplitude", "freq_range", "n_sines")


def mode_name(custom_mode):
    main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
    name = MAIN_MODE.get(main, f"main{main}")
    if main == 4:
        name += "." + AUTO_SUB.get(sub, f"sub{sub}")
    return name


class CommandServer:
    """JSON commands from the CLI in another terminal (UDP localhost). poll() does not block."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", int(port)))
        except OSError as e:
            raise RuntimeError(f"cannot open the drone command port {port} (used by another scene?): {e}") from e
        self.sock.setblocking(False)

    def poll(self):
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return out
            try:
                out.append((json.loads(data.decode()), addr))
            except ValueError:
                self.reply(addr, {"ok": False, "error": "not JSON"})

    def reply(self, addr, obj):
        self.sock.sendto(json.dumps(obj, ensure_ascii=False).encode(), addr)

    def close(self):
        self.sock.close()


class PX4Commander:
    def __init__(self, port, origin_enu, setpoint_hz=20.0, retry_sec=1.0, target_system=1, command_port=None, disable_streams=()):
        _load_mavlink()
        # streams to switch off once connected (commander.disable_streams). Unknown names are an error.
        #   IDs from MAVLink 2 common (M may be the v1 dialect, which lacks e.g. ODOMETRY; PX4 speaks MAVLink 2)
        from pymavlink.dialects.v20 import common as mav2

        self.disable_ids = []
        for name in disable_streams:
            mid = getattr(mav2, f"MAVLINK_MSG_ID_{name}", None)
            if mid is None:
                raise ValueError(f"unknown MAVLink message name: {name}")
            self.disable_ids.append(int(mid))
        self._streams_sent = False
        self.conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{int(port)}")
        self.server = CommandServer(command_port) if command_port is not None else None
        self.external = False       # the CLI took over the target (scene hover updates stop). Cleared on PX4 restart
        self.hover_hook = None      # hover mode: CLI goto = move the reference only, hold = freeze the sine
        self.cli_ready = False      # True once the scene finished take-off (move commands refused before; status, kill always)
        self.cli_log = []           # (t, command, ok, error)
        self.origin = np.asarray(origin_enu, dtype=float)
        self.sp_period, self.retry = 1.0 / float(setpoint_hz), float(retry_sec)
        self.sys = int(target_system)
        self.target, self.yaw = None, 0.0
        self.want_offboard, self.want_armed = False, False
        self.want_manual = False    # handed over to the transmitter (PX4 POSCTL + MANUAL_CONTROL)
        self.sticks = None          # (pitch, roll, throttle, yaw) -1 ~ 1, centre 0. None = MANUAL_CONTROL never sent
        self.armed, self.mode, self.landed = False, None, None
        self.estimate_enu, self.estimate_yaw = None, None
        self.param_values = {}      # values received as PARAM_VALUE {name: float}
        self.param_wanted = {}      # values requested with set_params (re-sent until confirmed)
        self.messages = []          # (t, severity, text) PX4 STATUSTEXT
        self.acks = []              # (t, command, result)
        self.t, self._t_sp, self._t_req = 0.0, -1e9, -1e9
        self.connected = False

    def reset_link(self):
        """After a PX4 restart: connection and state as at the start (stream switch-off is sent again). Socket kept."""
        self.connected, self._streams_sent = False, False
        self.armed, self.mode, self.landed = False, None, None
        self.estimate_enu, self.estimate_yaw = None, None
        self.target, self.want_offboard, self.want_armed = None, False, False
        self.want_manual, self.sticks = False, None
        self.external = False
        self.param_values, self.param_wanted = {}, {}
        self._t_sp, self._t_req = -1e9, -1e9

    def close(self):
        self.conn.close()
        if self.server is not None:
            self.server.close()

    # -- CLI commands --
    def status(self):
        f = lambda a: None if a is None else [round(float(x), 4) for x in a]  # noqa: E731
        return {"t": round(self.t, 3), "connected": self.connected, "mode": self.mode, "armed": self.armed, "landed": self.landed,
                "estimate": f(self.estimate_enu), "target": f(self.target), "yaw_deg": round(math.degrees(self.yaw), 2),
                "external": self.external, "ready": self.cli_ready, "manual": self.want_manual}

    def _handle_cli(self, req):
        cmd = req.get("cmd")
        if cmd == "status":
            return {"ok": True, **self.status()}
        if cmd in ("goto", "hold", "land") and not self.cli_ready:
            return {"ok": False, "error": "drone not ready: the scene has not started the reference yet (ready false)", **self.status()}
        if cmd in ("goto", "hold") and not (self.armed and self.mode == "OFFBOARD"):
            return {"ok": False, "error": f"not flying (Offboard, armed): mode {self.mode}, armed {self.armed}", **self.status()}
        if cmd == "goto":
            pos = req.get("pos")
            if not (isinstance(pos, list) and len(pos) == 3):
                return {"ok": False, "error": "goto needs pos [x, y, z] (world m)"}
            yaw = req.get("yaw_deg")
            yaw = None if yaw is None else math.radians(float(yaw))
            try:
                hooked = self.hover_hook is not None and self.hover_hook("goto", [float(v) for v in pos], yaw)
            except ValueError as e:
                return {"ok": False, "error": str(e), **self.status()}
            if not hooked:                          # not hover: the CLI takes over the target
                self.goto([float(v) for v in pos], yaw)
                self.external = True
        elif cmd == "hold":
            if not (self.hover_hook is not None and self.hover_hook("hold")):
                self.hold()
                self.external = True
        elif cmd == "land":
            self.land()
            self.external = True
        elif cmd == "kill":
            self.kill()
            self.external = True
        else:
            return {"ok": False, "error": f"unknown command: {cmd} (goto, hold, land, kill, status)"}
        return {"ok": True, **self.status()}

    # -- frames --
    def enu_to_ned(self, p):
        d = np.asarray(p, dtype=float) - self.origin
        return np.array([d[1], d[0], -d[2]])

    def ned_to_enu(self, n):
        return self.origin + np.array([n[1], n[0], -n[2]])

    # -- receive --
    def poll(self):
        while True:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                return
            if m.get_srcSystem() != self.sys:
                continue
            k = m.get_type()
            if k == "HEARTBEAT" and m.get_srcComponent() == 1:
                self.connected = True
                self.armed = bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = mode_name(m.custom_mode)
            elif k == "LOCAL_POSITION_NED":
                self.estimate_enu = self.ned_to_enu([m.x, m.y, m.z])
            elif k == "ATTITUDE":
                self.estimate_yaw = math.pi / 2 - m.yaw
            elif k == "EXTENDED_SYS_STATE":
                self.landed = {0: "undefined", 1: "on_ground", 2: "in_air", 3: "takeoff", 4: "landing"}.get(m.landed_state)
            elif k == "PARAM_VALUE":
                name = m.param_id.decode() if isinstance(m.param_id, bytes) else str(m.param_id)
                name = name.rstrip("\x00")
                if name in self.param_wanted and m.param_type != M.MAV_PARAM_TYPE_REAL32:
                    raise RuntimeError(f"PX4 parameter {name} is not a float (type {m.param_type}): set_params is float only")
                self.param_values[name] = float(m.param_value)
            elif k == "STATUSTEXT":
                self.messages.append((round(self.t, 3), int(m.severity), m.text))
            elif k == "COMMAND_ACK":
                self.acks.append((round(self.t, 3), int(m.command), int(m.result)))
                if m.command == M.MAV_CMD_SET_MESSAGE_INTERVAL and m.result != M.MAV_RESULT_ACCEPTED:
                    raise RuntimeError(f"PX4 refused the stream switch-off (SET_MESSAGE_INTERVAL): result {m.result}")

    # -- send --
    def _command(self, cmd, *params):
        p = list(params) + [0.0] * (7 - len(params))
        self.conn.mav.command_long_send(self.sys, 1, cmd, 0, *p)

    def _send_setpoint(self):
        n = self.enu_to_ned(self.target)
        self.conn.mav.set_position_target_local_ned_send(int(self.t * 1000), self.sys, 1, M.MAV_FRAME_LOCAL_NED, POS_YAW_MASK,
                                                         n[0], n[1], n[2], 0, 0, 0, 0, 0, 0, math.pi / 2 - self.yaw, 0)

    def _send_manual(self):
        # PX4 MANUAL_CONTROL: x = pitch (forward +), y = roll (right +), r = yaw (right +) -1000 ~ 1000, z = throttle 0 ~ 1000 (centre 500)
        pitch, roll, thr, yaw = self.sticks if self.want_manual else (0.0, 0.0, 0.0, 0.0)
        self.conn.mav.manual_control_send(self.sys, int(round(pitch * 1000)), int(round(roll * 1000)),
                                          int(round((thr + 1.0) * 500)), int(round(yaw * 1000)), 0)

    def _send_params(self):
        for name, v in self.param_wanted.items():
            if not self._param_ok(name):
                self.conn.mav.param_set_send(self.sys, 1, name.encode(), float(v), M.MAV_PARAM_TYPE_REAL32)

    def _param_ok(self, name):
        got = self.param_values.get(name)
        return got is not None and abs(got - float(self.param_wanted[name])) <= 1e-4 * max(1.0, abs(float(self.param_wanted[name])))

    def _send_gcs_heartbeat(self):
        self.conn.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # -- commands --
    def fly_to(self, target_enu, yaw_enu=0.0):
        """Offboard position target + arm. tick() re-requests until done."""
        self.goto(target_enu, yaw_enu)
        self.want_offboard, self.want_armed = True, True

    def set_params(self, params):
        """Runtime PX4 float parameters. tick() re-sends until PARAM_VALUE returns the same value (params_confirmed)."""
        for name, v in params.items():
            if not math.isfinite(float(v)):
                raise ValueError(f"PX4 parameter value is not finite: {name}={v}")
            self.param_wanted[str(name)] = float(v)
            self.param_values.pop(str(name), None)
        if self.connected:
            self._send_params()

    def params_confirmed(self, names=None):
        names = self.param_wanted if names is None else names
        return all(self._param_ok(n) for n in names)

    def goto(self, target_enu, yaw_enu=None):
        self.target = np.asarray(target_enu, dtype=float)
        if yaw_enu is not None:
            self.yaw = float(yaw_enu)

    def hold(self):
        if self.estimate_enu is None:
            raise RuntimeError("no PX4 position estimate yet")
        self.goto(self.estimate_enu.copy())

    def land(self):
        self.want_offboard = False
        self.target = None
        self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 6)

    def set_sticks(self, pitch, roll, throttle, yaw):
        """Transmitter sticks (-1 ~ 1, centre 0. pitch forward +, roll right +, throttle up +, yaw right +). Out of range = error."""
        v = (float(pitch), float(roll), float(throttle), float(yaw))
        if not all(math.isfinite(x) and -1.0 <= x <= 1.0 for x in v):
            raise ValueError(f"stick values outside -1 ~ 1: {v}")
        self.sticks = v

    def handover(self):
        """Hand over to the transmitter: request PX4 manual mode (POSCTL); the position target stops once the mode changed.
        Only while flying Offboard."""
        if self.sticks is None:
            raise RuntimeError("no stick values (set_sticks first)")
        if not (self.connected and self.armed and self.mode == "OFFBOARD"):
            raise RuntimeError(f"not flying Offboard: mode {self.mode}, armed {self.armed}")
        self.want_manual, self.want_offboard = True, False
        self.external = True
        self._t_req = -1e9

    def takeback(self):
        """Take back from the transmitter: Offboard with the current estimate as target (signal lost, before a reset)."""
        self.want_manual = False
        self.hold()
        self.want_offboard = True
        self._t_req = -1e9

    def kill(self):
        """Forced stop in the air (force disarm). Motors stop at once."""
        self.want_offboard, self.want_armed = False, False
        self.want_manual = False
        self.target = None
        self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, KILL_MAGIC)

    def tick(self, t):
        """Call every loop with sim time t: receive, send the setpoint, re-request mode / arm."""
        self.t = float(t)
        self.poll()
        if self.connected and not self._streams_sent:
            for mid in self.disable_ids:
                self._command(M.MAV_CMD_SET_MESSAGE_INTERVAL, mid, -1)
            self._streams_sent = True
        if self.server is not None:
            for req, addr in self.server.poll():
                try:
                    res = self._handle_cli(req)
                except RuntimeError as e:          # e.g. hold before any estimate: answer instead of stopping the bridge
                    res = {"ok": False, "error": str(e), **self.status()}
                self.cli_log.append((round(self.t, 3), req, res.get("ok"), res.get("error")))
                self.server.reply(addr, res)
        if self.sticks is not None and self.connected:
            self._send_manual()
        if self.want_manual and self.mode == "POSCTL":
            self.target = None                      # handed over -> stop the position target (sent until then so Offboard holds)
        if self.target is not None and self.t - self._t_sp >= self.sp_period:
            self._send_setpoint()
            self._t_sp = self.t
        if self.t - self._t_req >= self.retry:
            self._send_gcs_heartbeat()
            if self.connected and self.param_wanted and not self.params_confirmed():
                self._send_params()
            if self.connected and self.want_manual and self.armed and self.mode != "POSCTL":
                self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0)
            elif self.connected and self.want_offboard and self.target is not None and self.mode != "OFFBOARD":
                self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0)
            elif self.connected and self.want_armed and self.mode == "OFFBOARD" and not self.armed:
                self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self._t_req = self.t


class Reference:
    """Target position / velocity / acceleration / jerk (world). static: p0. hover: p0 + per-axis sum of sines
    (amplitude sum = amplitude of preset `preset`). freeze(t): stop at that target (position fixed, derivatives 0).
    reseed(seed, p0): new frequencies / phases, unfreeze."""

    def __init__(self, mode, p0, presets=None, seed=0, preset=None):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}: {mode}")
        if mode == "hover":
            if preset is None:
                raise ValueError(f"hover mode needs a preset name ({' | '.join(presets or {})})")
            if not presets or preset not in presets:
                raise KeyError(f"hover preset '{preset}' not found (hover_presets: {list(presets or {})})")
            for k in HOVER_KEYS:
                if k not in presets[preset]:
                    raise KeyError(f"hover_presets.{preset}.{k} missing")
            self.hc = presets[preset]
        elif preset is not None:
            raise ValueError(f"hover preset ({preset}) only in hover mode")
        self.mode, self.preset = mode, preset
        self.reseed(seed, p0)

    def reseed(self, seed, p0):
        self.p0 = np.asarray(p0, dtype=float).copy()
        self.t_freeze, self.p_freeze, self.freeze_reason = None, None, None
        self.params = {"mode": self.mode, "p0": self.p0.tolist(), "seed": int(seed)}
        if self.mode == "hover":
            rng = np.random.default_rng(seed)
            n = int(self.hc["n_sines"])
            amp = np.asarray(self.hc["amplitude"], dtype=float)
            lo, hi = (float(x) for x in self.hc["freq_range"])
            self.A = np.repeat(amp[:, None] / n, n, axis=1)                  # (3, n)
            self.f = rng.uniform(lo, hi, size=(3, n))
            self.phi = rng.uniform(0, 2 * math.pi, size=(3, n))
            self.params.update(preset=self.preset, amplitude=amp.tolist(), freq=self.f.tolist(), phase=self.phi.tolist(),
                               max_speed=np.round((self.A * 2 * math.pi * self.f).sum(axis=1), 4).tolist())

    def set_p0(self, p0):
        """Move the reference position only (the sine continues; a frozen target moves by the same amount)."""
        p0 = np.asarray(p0, dtype=float)
        if self.p_freeze is not None:
            self.p_freeze = self.p_freeze + (p0 - self.p0)
        self.p0 = p0.copy()
        self.params["p0"] = self.p0.tolist()

    def freeze(self, t, reason):
        """Stop at the target of time t (this Reference's time). Already frozen: unchanged (first reason / time kept)."""
        if self.mode != "hover" or self.t_freeze is not None:
            return False
        self.p_freeze = self._eval(t)[0]
        self.t_freeze, self.freeze_reason = float(t), reason
        return True

    def _eval(self, t):
        if self.mode == "static":
            return self.p0.copy(), np.zeros(3), np.zeros(3), np.zeros(3)
        w = 2 * math.pi * self.f
        arg = w * t + self.phi
        # subtract sin(phi) so the target equals p0 at t = 0 (no jump)
        p = self.p0 + (self.A * (np.sin(arg) - np.sin(self.phi))).sum(axis=1)
        v = (self.A * w * np.cos(arg)).sum(axis=1)
        a = (-self.A * w * w * np.sin(arg)).sum(axis=1)
        j = (-self.A * w ** 3 * np.cos(arg)).sum(axis=1)
        return p, v, a, j

    def __call__(self, t):
        if self.p_freeze is not None:
            return self.p_freeze.copy(), np.zeros(3), np.zeros(3), np.zeros(3)
        return self._eval(t)
