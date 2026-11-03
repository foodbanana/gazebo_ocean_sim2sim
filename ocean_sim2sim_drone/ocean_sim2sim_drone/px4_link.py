# =============================================================
# px4_link.py - PX4 SITL <-> simulator MAVLink link (HIL lockstep), PX4 process launcher, PX4 airframe parameters
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/px4_bridge.py (PX4Launcher, PX4Bridge) and
#     isaacsim/scripts/drone_flight.py (DroneFlight._init_px4 config assembly, _px4_airframe_params), which come from
#     livealive7/PegasusSimulator b1256ca logic/backends/px4_mavlink_backend.py, tools/px4_launch_tool.py
#     (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto, license: LICENSE.PegasusSimulator next to this file).
#
#   PX4Launcher: runs PX4 SITL as a subprocess (same command as Pegasus:
#     PX4_SIM_MODEL=<airframe> build/px4_sitl_default/bin/px4 ROMFS/px4fmu_common -s rcS -i <instance> -d).
#     Working dir (rootfs: parameters, ulog) = a new folder under the report dir (Pegasus: temp dir) -> flight logs (.ulg)
#     are kept. Parameter overrides through the PX4 rcS environment variables PX4_PARAM_<name>=<value>.
#     PR_SET_PDEATHSIG: PX4 gets SIGTERM when this process dies, whatever the way.
#   PX4Bridge: TCP 4560 (simulator = server, PX4 connects), lockstep. Every physics step (Pegasus
#     PX4MavlinkBackend.update order):
#     1. (caller) apply the thrust of the motor command received in the previous step
#     2. after the first motor command: wait until HIL_ACTUATOR_CONTROLS arrives (lockstep, error after lockstep_timeout)
#     3. heartbeat (wall clock 1 Hz), HIL_SENSOR (IMU, mag, baro; fields_updated bits only for values new in this step),
#        VISION_POSITION_ESTIMATE (mocap). time_usec = accumulated sim time (PX4 runs its clock on it)
#     4. update the sensors (sent in the next step: in Pegasus the sensor callbacks run after the backend, so the values
#        sent are one step old -> same order)
#   Motor command -> omega: armed (SAFETY_ARMED bit of mode) -> omega_i = (u_i + offset_i) scaling_i + zero_position_armed_i,
#     else 0, clipped to [0, omega_max] (Pegasus QuadraticThrustCurve). Rotor order = PX4 motor order.
#   Differences to Pegasus (as in Isaac): HIL_SENSOR temperature argument = barometer temperature (Pegasus passes the GPS
#     altitude in mm, argument order bug), PX4 silence -> error (Pegasus waits forever), status before lockstep exposed.
#
#   Changes from the Isaac Sim files: GPS / distance sensor / optical flow messages removed (mocap position source only),
#     no ground raycast argument, PX4 directory and report dir passed in by the caller (ROS parameters), config read from
#     drone_px4.yaml + isaac_flight.yaml (airframe exported from Isaac Sim) instead of px4_sitl.yaml + drone_iris.yaml,
#     rotor positions / inertia for the airframe parameters read from isaac_flight.yaml instead of PhysX (same values),
#     start() is separated from construction (PX4 is started by the restart service), comments in English,
#     no motor command history recording. Equations, message order, rates and PX4 parameters are identical.
# =============================================================

import atexit
import datetime
import math
import os
import subprocess
import time

import numpy as np
from pymavlink import mavutil

from .sensors import SensorSuite

SENSOR_ACCEL, SENSOR_GYRO, SENSOR_MAG, SENSOR_BARO = 0b111, 0b111000, 0b111000000, 0b1101000000000
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED   # 128

PX4C_KEYS = ("airframe", "input_offset", "input_scaling", "zero_position_armed", "params", "rotor_geometry_from_sim",
             "rate_gain_scaling", "motor_dynamics", "sensor_overrides", "sensor_kinematics")


def _die_with_parent():
    """PX4 child process: the kernel sends SIGTERM when the parent dies (Linux prctl PR_SET_PDEATHSIG)."""
    import ctypes
    import signal

    ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)    # PR_SET_PDEATHSIG = 1


def running_px4(binary, instance):
    """pids of processes whose executable is `binary` with -i instance (/proc/*/cmdline)."""
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as fh:
                argv = fh.read().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].decode(errors="ignore") == binary and b"-i" in argv:
            i = argv.index(b"-i")
            if i + 1 < len(argv) and argv[i + 1] == str(instance).encode():
                pids.append(int(d))
    return pids


class PX4Launcher:
    def __init__(self, px4_dir, airframe, instance, run_dir, params=None):
        px4_dir = os.path.expanduser(px4_dir)
        self.binary = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
        self.romfs = os.path.join(px4_dir, "ROMFS/px4fmu_common")
        self.rcs = os.path.join(self.romfs, "init.d-posix/rcS")
        for f in (self.binary, self.rcs):
            if not os.path.exists(f):
                raise FileNotFoundError(f"PX4 SITL file missing: {f} (build with `make px4_sitl_default none` in the PX4 dir)")
        af = os.path.join(self.romfs, "init.d-posix/airframes")
        if not any(n.split("_", 1)[-1] == airframe for n in os.listdir(af)):
            raise ValueError(f"PX4 airframe '{airframe}' not found in {af}")
        self.airframe, self.instance = airframe, int(instance)
        self.run_dir = run_dir
        self.params = dict(params or {})
        self.proc, self.log = None, None

    def launch(self):
        os.makedirs(self.run_dir, exist_ok=False)
        env = dict(os.environ)
        env["PX4_SIM_MODEL"] = self.airframe
        for k, v in self.params.items():
            env[f"PX4_PARAM_{k}"] = str(v)
        self.log = open(os.path.join(self.run_dir, "px4_console.log"), "w")
        # a PX4 with the same instance would make the new one exit at once ("server already running") -> say so first
        other = running_px4(self.binary, self.instance)
        if other:
            raise RuntimeError(f"PX4 instance {self.instance} already running (pid {other}). "
                               f"If no other sim uses it: kill {' '.join(map(str, other))}")
        self.proc = subprocess.Popen([self.binary, self.romfs, "-s", self.rcs, "-i", str(self.instance), "-d"],
                                     cwd=self.run_dir, env=env, stdin=subprocess.DEVNULL, stdout=self.log,
                                     stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)
        atexit.register(self.kill)              # normal exit; PDEATHSIG is the main safety net

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def kill(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
            self.proc = None
        if self.log is not None:
            self.log.close()
            self.log = None


class PX4Bridge:
    """Call step() every physics step. Once PX4 arms it returns the rotor angular velocities [rad/s]."""

    def __init__(self, sitl, px4c, sensors_cfg, w_max, n_rotors, seed, report_dir):
        self.sitl, self.c = sitl, px4c
        self.sensors_cfg, self.seed = sensors_cfg, seed
        self.w_max, self.n = float(w_max), int(n_rotors)
        self.offset = np.asarray(px4c["input_offset"], dtype=float)
        self.scaling = np.asarray(px4c["input_scaling"], dtype=float)
        self.zero_armed = np.asarray(px4c["zero_position_armed"], dtype=float)
        for name, a in (("input_offset", self.offset), ("input_scaling", self.scaling), ("zero_position_armed", self.zero_armed)):
            if a.shape != (self.n,):
                raise ValueError(f"px4.{name} length {a.shape} != rotors {self.n}")
        stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
        self.run_dir = os.path.join(os.path.expanduser(report_dir), f"px4_{stamp}")
        self.port = int(sitl["sim_port"]) + int(sitl["instance"])
        self.launcher = PX4Launcher(sitl["px4_dir"], px4c["airframe"], sitl["instance"], self.run_dir, px4c.get("params"))
        self.conn = None
        self.started = False
        self.n_restart = 0
        self._reset_state()
        self.n_actuator = 0

    def _reset_state(self):
        self.sensors, self.dt = None, None
        self.t_us = 0
        self.heartbeat, self.first_actuator = False, False
        self.armed, self.u = False, np.zeros(self.n)
        self.omega = np.zeros(self.n)
        self.pending = None
        self.last = {"imu": None, "baro": None, "mag": None}
        self.last_hb_wall = 0.0

    @property
    def status(self):
        if not self.started:
            return "stopped"
        if not self.heartbeat:
            return "waiting_heartbeat"
        return "lockstep" if self.first_actuator else "waiting_actuator"

    @property
    def current_run_dir(self):
        return self.launcher.run_dir

    def start(self):
        self.conn = mavutil.mavlink_connection(f"tcpin:localhost:{self.port}")   # open first, then launch PX4
        self.started = True
        if self.sitl["autolaunch"]:
            self.launcher.launch()

    def stop(self):
        self.started = False
        if self.conn is not None:
            self.conn.close()
            self.conn = None
        if self.sitl["autolaunch"]:
            self.launcher.kill()

    def restart(self):
        """Fresh PX4 process (reset: an EKF broken by kill / fall = rebooting the real drone). State and virtual sensors
        (incl. biases, same seed) start over. Log in a new folder (<run_dir>_r<n>). The first call just starts PX4."""
        if not self.sitl["autolaunch"]:
            raise RuntimeError("autolaunch is off: cannot (re)start PX4")
        if self.started or self.launcher.proc is not None:
            self.stop()
            self.n_restart += 1
            self.launcher = PX4Launcher(self.sitl["px4_dir"], self.c["airframe"], self.sitl["instance"],
                                        f"{self.run_dir}_r{self.n_restart}", self.c.get("params"))
        elif os.path.exists(self.launcher.run_dir):          # start after a failed start: new folder
            self.n_restart += 1
            self.launcher = PX4Launcher(self.sitl["px4_dir"], self.c["airframe"], self.sitl["instance"],
                                        f"{self.run_dir}_r{self.n_restart}", self.c.get("params"))
        self._reset_state()
        self.start()

    # -- receive --
    def _handle(self, msg):
        if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
            self.first_actuator = True
            self.n_actuator += 1
            self.armed = bool(msg.mode & ARMED_FLAG)
            self.u = np.asarray(msg.controls[: self.n], dtype=float)
            if self.armed:
                w = (self.u + self.offset) * self.scaling + self.zero_armed
                self.omega = np.clip(w, 0.0, self.w_max)
            else:
                self.omega = np.zeros(self.n)
            return True
        return False

    def _poll(self):
        if not self.first_actuator:
            while True:
                m = self.conn.recv_match(blocking=False)
                if m is None:
                    return
                self._handle(m)
        deadline = time.monotonic() + float(self.sitl["lockstep_timeout"])
        while True:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is not None and self._handle(m):
                return
            if self.sitl["autolaunch"] and not self.launcher.alive():
                raise RuntimeError(f"PX4 process ended (log {self.launcher.run_dir}/px4_console.log)")
            if time.monotonic() > deadline:
                raise RuntimeError(f"no PX4 motor command for {self.sitl['lockstep_timeout']} s (lockstep broken)")

    # -- send --
    def _send_sensors(self):
        d = self.pending
        fields = 0
        if d["imu"] is not None:
            fields |= SENSOR_ACCEL | SENSOR_GYRO
            self.last["imu"] = d["imu"]
        if d["mag"] is not None:
            fields |= SENSOR_MAG
            self.last["mag"] = d["mag"]
        if d["baro"] is not None:
            fields |= SENSOR_BARO
            self.last["baro"] = d["baro"]
        imu, mag, baro = self.last["imu"], self.last["mag"], self.last["baro"]
        acc, gyro = (imu["acc"], imu["gyro"]) if imu else (np.zeros(3), np.zeros(3))
        m = mag["mag"] if mag else np.zeros(3)
        b = baro or {"abs_pressure_hpa": 0.0, "pressure_alt": 0.0, "temperature": 0.0}
        self.conn.mav.hil_sensor_send(self.t_us, *map(float, acc), *map(float, gyro), *map(float, m),
                                      float(b["abs_pressure_hpa"]), 0.0, float(b["pressure_alt"]), float(b["temperature"]),
                                      fields)
        mc = d.get("mocap")
        if mc is not None:
            cov = [float("nan")] + [0.0] * 20                   # unknown variance -> PX4 uses EKF2_EVP/EVA_NOISE (EKF2_EV_NOISE_MD 1)
            self.conn.mav.vision_position_estimate_send(self.t_us, *map(float, mc["pos_ned"]), *map(float, mc["euler"]), cov, 0)

    def step(self, dt, p, v, R, w_body):
        """Exchange with PX4 for this step's state; returns the omega command to apply in the NEXT step."""
        if self.dt is None:
            self.dt = dt
            self.sensors = SensorSuite(self.sensors_cfg, dt, self.seed)
        elif abs(dt - self.dt) > 1e-9:
            raise RuntimeError(f"physics dt changed {self.dt} -> {dt} (sensor rates use the first dt)")
        if self.sitl["autolaunch"] and not self.launcher.alive():
            raise RuntimeError(f"PX4 process ended (log {self.launcher.run_dir}/px4_console.log)")
        if not self.heartbeat:
            if self.conn.wait_heartbeat(blocking=False) is not None:
                self.heartbeat = True
        if self.heartbeat:
            self._poll()
            now = time.monotonic()
            if now - self.last_hb_wall > 1.0:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.last_hb_wall = now
            self.t_us += int(round(dt * 1e6))
            if self.pending is not None:
                self._send_sensors()
        self.pending = self.sensors.update(p, v, R, w_body)
        return self.omega.copy()


# -- configuration (Isaac drone_flight.DroneFlight._init_px4 / _px4_airframe_params) --

def airframe_params(fl):
    """PX4 parameters matched to our airframe (only the enabled ones), from isaac_flight.yaml.
    rotor_geometry_from_sim: CA_ROTORi_PX/PY = rotor positions (body origin, FRD).
    rate_gain_scaling: whole rate loop gain MC_{ROLL,PITCH,YAW}RATE_K = (I / I_ref) / (arm / arm_ref) per axis.
      angular acceleration = torque / I, max torque ~ arm (roll: mean rotor |y|, pitch: mean |x|, yaw: reaction torque,
      no arm) -> same rate response as the reference airframe. K multiplies P, I and D (PX4 MC_*RATE_K).
    Isaac reads the rotor prim positions relative to the body prim and the PhysX inertia about the total COM in the body
    frame; isaac_flight.yaml holds exactly these values (rotors[].position, inertia_about_com)."""
    px4c = fl["px4"]
    out = {}
    rel = np.array([r["position"] for r in fl["rotors"]], dtype=float)        # body FLU
    frd = np.column_stack([rel[:, 0], -rel[:, 1]])
    if px4c["rotor_geometry_from_sim"]:
        for i, (x, y) in enumerate(frd):
            out[f"CA_ROTOR{i}_PX"] = round(float(x), 4)
            out[f"CA_ROTOR{i}_PY"] = round(float(y), 4)
    rs = px4c["rate_gain_scaling"]
    if rs["enabled"]:
        I = np.diag(np.asarray(fl["inertia_about_com"], dtype=float))
        I_ref = np.asarray(rs["reference_inertia"], dtype=float)
        arm = np.abs(frd).mean(axis=0)                     # [|x|, |y|]
        arm_ref = np.asarray(rs["reference_arm_xy"], dtype=float)
        k = (I / I_ref) / np.array([arm[1] / arm_ref[1], arm[0] / arm_ref[0], 1.0])
        for name, v in zip(("MC_ROLLRATE_K", "MC_PITCHRATE_K", "MC_YAWRATE_K"), k):
            out[name] = round(float(v), 4)
    return out


def build_px4_config(cfg, fl, position_source=None):
    """cfg = drone_px4.yaml, fl = isaac_flight.yaml -> (px4c with merged params, sensors config, auto params).
    PX4 parameter priority (Isaac): position source < airframe auto values < isaac_flight.yaml px4.params."""
    px4c = dict(fl["px4"])
    for k in PX4C_KEYS:
        if k not in px4c:
            raise KeyError(f"isaac_flight.yaml: 'px4.{k}' missing")
    ps = cfg["position_source"] if position_source is None else position_source
    if ps not in cfg["position_sources"]:
        raise ValueError(f"position_source '{ps}' not in drone_px4.yaml position_sources (ported: mocap only)")
    src = cfg["position_sources"][ps]
    sensors = {k: (dict(v) if isinstance(v, dict) else v) for k, v in cfg["sensors"].items()}
    for k, v in (px4c["sensor_overrides"] or {}).items():                   # per-airframe sensor overrides (one level)
        if k not in sensors:
            raise KeyError(f"px4.sensor_overrides.{k} is not in drone_px4.yaml sensors")
        if isinstance(v, dict):
            sensors[k].update(v)
        else:
            sensors[k] = v
    if px4c["sensor_kinematics"] != "pose_difference":
        raise ValueError(f"px4.sensor_kinematics: only pose_difference is ported: {px4c['sensor_kinematics']}")
    for k in ("gps", "rangefinder", "optical_flow"):
        if src.get(k):
            raise ValueError(f"position source '{ps}' enables {k}: not ported (mocap only)")
    sensors["mocap"]["enabled"] = bool(src["mocap"])
    auto = airframe_params(fl)
    px4c["params"] = {**(src["params"] or {}), **auto, **(px4c["params"] or {})}
    return px4c, sensors, auto


# -- sensor kinematics (Isaac drone_flight.py: px4.sensor_kinematics pose_difference, _log_so3, DroneFlight._step) --

def _vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def log_so3(R):
    """Rotation matrix -> rotation vector [rad]."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = math.acos(c)
    w = _vee(R - R.T) / 2.0
    if th < 1e-9:
        return w
    return w * (th / math.sin(th))


class PoseDifference:
    """Velocity / angular velocity given to the virtual sensors (IMU) = pose differences instead of the physics engine's
    velocities (Isaac: when the gripper holds the drone, PhysX reported angular velocities that did not match the real
    attitude change, and the fake gyro made the PX4 attitude estimate diverge; a real IMU only measures real motion):
        v = (p_k - p_{k-1}) / dt,  w_body = log(R_{k-1}^T R_k) / dt
    The first sample after reset() uses the simulator velocities (world v, body R^T w_world)."""

    def __init__(self):
        self.prev = None

    def reset(self):
        self.prev = None

    def __call__(self, dt, p, R, v_world, w_world):
        v_s, w_s = v_world, R.T @ w_world
        if self.prev is not None:
            p0, R0 = self.prev
            v_s = (p - p0) / dt
            w_s = log_so3(R0.T @ R) / dt               # body frame angular velocity
        self.prev = (p.copy(), R.copy())
        return v_s, w_s
