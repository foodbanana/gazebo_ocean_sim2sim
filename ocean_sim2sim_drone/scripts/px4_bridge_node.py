#!/usr/bin/env python3
# =============================================================
# px4_bridge_node.py - ROS 2 node: Gazebo drone (DroneLockstep plugin) <-> PX4 SITL, flown like the Isaac Sim drone
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/drone_flight.py (DroneFlight px4 backend: _step, arm,
#     release, restart_px4, teleport, hover_freeze, _on_cli, check), drone_scene.px4_takeoff (connect + ekf_wait) and
#     sim_reset (restart semantics). Pegasus origin: livealive7/PegasusSimulator b1256ca (BSD-3-Clause,
#     Copyright (c) 2023 Marcelo Jacinto, ocean_sim2sim_drone/LICENSE.PegasusSimulator).
#
#   Every plugin exchange (= one Isaac Sim physics step, dt = physics_dt = 1/120 s), same order as Isaac DroneFlight._step:
#     1. ThrustModel.step: the omega command of the PREVIOUS step through the motor lag -> wrench (thrust, reaction
#        torque, linear drag with the plugin velocity) + prop joint speeds (Isaac prop spin rule from the rotor
#        thrusts of this step, flight.prop_speeds; zeros if parameter prop_spin is false) = reply to the plugin
#     2. sensor kinematics pose_difference (v = dp/dt, w_body = log(R_prev^T R)/dt; first sample: plugin velocities)
#     3. PX4Bridge.step (waits for HIL_ACTUATOR_CONTROLS only after the first one, sends HIL_SENSOR + mocap, updates the
#        sensors) -> omega_cmd -> ThrustModel.set_command (applied in the next exchange)
#     4. hover reference -> offboard target, PX4Commander.tick(sim t) (Isaac: DroneFlight.check after the physics step)
#   PX4 not running (before the first restart, after an error): zero wrench and props stopped at once (the drone
#   hangs on its anchor). Reply = 10 doubles: force (3), torque (3) [world], prop joint speeds (4) [rad/s].
#
#   ROS interface (all JSON replies in Trigger.message / String.data):
#     services (std_srvs/srv/Trigger)
#       /drone/px4/restart  stop + start PX4 (fresh rootfs under report_dir), wait for lockstep + offboard link, then
#                           ekf_wait s sim. Refused while armed (kill first). Clears CLI take-over, reference, sensors.
#       /drone/px4/arm      align the PX4 local origin to the true pose (origin += p_true - estimate), Offboard position
#                           target = current true position + yaw, arm; returns when PX4 is armed in OFFBOARD
#       /drone/px4/kill     PX4 kill (forced disarm), hover sine frozen ("kill"). success false if not armed
#       /drone/px4/status   state JSON (same as /drone/px4/state)
#     topics
#       /drone/px4/target        geometry_msgs/PointStamped (frame world or empty): reference position p0. Starts the
#                                reference if it is not running (mode / preset / seed from the parameters), else moves p0
#                                (hover sine continues)
#       /drone/px4/reference_cmd std_msgs/String JSON {"mode": "static"|"hover", "preset": "small"|"mid"|"big",
#                                "seed": int, "p0": [x, y, z], "ready": bool} (all optional: parameters / current p0 /
#                                true position / true). (Re)starts the reference: new sine frequencies / phases,
#                                unfreeze, sine time 0 now. ready true: CLI move commands and the transmitter hand-over
#                                accepted from now (status ready; false = scene still in the air start, e.g. lifting
#                                off the anchor). Errors: log + state "reference_error"
#       /drone/px4/state (pub)   std_msgs/String JSON at state_rate Hz sim
#     parameters: config, flight_config, px4_dir (~/PX4-Autopilot), report_dir, seed (virtual sensor noise seed),
#       hover_mode, hover_preset, hover_seed (defaults for the reference), prop_spin (true: props turn with the Isaac
#       rule, like Isaac Sim with GUI; false: held still, like Isaac Sim headless), use_sim_time (forced true),
#       rc (true: real transmitter, drone_rc.DroneRc every exchange before PX4Commander.tick = Isaac --drone-rc on),
#       rc_config (config/rc_input.yaml), rc_port (serial device, empty = rc_config port)
#   Transmitter (rc true): rc_input.yaml px4_params are added to the PX4 start parameters; the scene must fly a static
#     reference (hover sine + transmitter = two drivers, refused by the launch file as in Isaac). A serial error (Pico
#     unplugged) switches the transmitter off for this run: taken back to Offboard hover + error log (Isaac: exception).
#   Air start sequence of a scene node: spawn the drone on its anchor -> /drone/px4/restart -> /drone/px4/arm ->
#     detach the anchor (gz /drone/detach) -> wait until settled -> /drone/px4/reference_cmd (or /drone/px4/target).
#
#   Changes from Isaac: air start on an anchor instead of a ground take-off (no take-off legs / waypoints / settle:
#     scene node's job), the plugin's TCP link replaces the PhysX callback, reference driven by ROS topics instead of
#     the scene script, geometric controller / gps / flow dropped.
# =============================================================
import json
import math
import os
import socket
import struct
import sys
import threading
import time

try:
    from ocean_sim2sim_drone import px4_link
except ModuleNotFoundError as e:
    if e.name == "pymavlink":
        print("[px4_bridge] pymavlink not found: activate the venv: source ~/vrx_ws/.venv/bin/activate (README)",
              file=sys.stderr)
        sys.exit(1)
    raise

import numpy as np
import rclpy
import rclpy.clock
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ocean_sim2sim_drone import rc_input
from ocean_sim2sim_drone.commander import PX4Commander, Reference
from ocean_sim2sim_drone.drone_rc import DroneRc
from ocean_sim2sim_drone.flight import ThrustModel, load_flight, prop_speeds, quat_wxyz_to_R

IN = struct.Struct("<15d")
OUT = struct.Struct("<10d")                         # wrench (6) + prop joint speeds (4)


def _r(a, n=4):
    return None if a is None else [round(float(x), n) for x in a]


class ServiceError(RuntimeError):
    pass


class PX4BridgeNode(Node):
    def __init__(self):
        super().__init__("px4_bridge")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        share = get_package_share_directory("ocean_sim2sim_drone")
        gz_share = get_package_share_directory("ocean_sim2sim_gazebo")
        self.declare_parameter("config", os.path.join(share, "config", "drone_px4.yaml"))
        self.declare_parameter("flight_config", os.path.join(gz_share, "models", "iris_075", "meshes", "collision_isaac",
                                                             "isaac_flight.yaml"))
        self.declare_parameter("px4_dir", "~/PX4-Autopilot")
        self.declare_parameter("report_dir", "~/.ros/ocean_sim2sim_drone")
        self.declare_parameter("seed", 0)
        self.declare_parameter("hover_mode", "static")
        self.declare_parameter("hover_preset", "small")
        self.declare_parameter("hover_seed", 0)
        self.declare_parameter("prop_spin", True)
        self.declare_parameter("rc", False)
        self.declare_parameter("rc_config", os.path.join(share, "config", "rc_input.yaml"))
        self.declare_parameter("rc_port", "")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        with open(os.path.expanduser(gp("config")), encoding="utf-8") as f:
            self.cfg = cfg = yaml.safe_load(f)
        self.fl = load_flight(os.path.expanduser(gp("flight_config")))
        px4c, sensors, auto = px4_link.build_px4_config(cfg, self.fl)
        self.rc_cfg = rc_input.load_config(os.path.expanduser(gp("rc_config"))) if gp("rc") else None
        if self.rc_cfg is not None:
            px4c["params"].update(self.rc_cfg["px4_params"])
        self.px4_params, self.auto_params = px4c["params"], auto
        sitl = {"px4_dir": os.path.expanduser(gp("px4_dir")), "autolaunch": bool(cfg["autolaunch"]),
                "instance": int(cfg["instance"]), "sim_port": int(cfg["sim_port"]),
                "lockstep_timeout": float(cfg["lockstep_timeout"])}
        self.dt = float(cfg["physics_dt"])
        self.thrust = ThrustModel(self.fl)
        n = len(self.thrust.dir)
        if n != 4:
            raise ValueError(f"the plugin reply carries 4 prop speeds, the flight config has {n} rotors")
        self.prop_spin = bool(gp("prop_spin"))
        self.bridge = px4_link.PX4Bridge(sitl, px4c, sensors, self.thrust.w_max, n, int(gp("seed")),
                                         os.path.expanduser(gp("report_dir")))
        inst = int(cfg["instance"])
        cc = cfg["commander"]
        self.cmd = PX4Commander(int(cfg["offboard_port"]) + inst, np.zeros(3), cc["setpoint_hz"], cc["retry_sec"],
                                command_port=int(cfg["command_port"]) + inst, disable_streams=cc["disable_streams"])
        self.cmd.hover_hook = self._on_cli
        self.drone_rc, self.rc_error = None, None
        if self.rc_cfg is not None:
            port = gp("rc_port") or self.rc_cfg["port"]
            try:
                self.drone_rc = DroneRc(self.cmd, self.rc_cfg, lambda: 0.0 if self.t is None else self.t, port,
                                        log=self.get_logger().info)
            except OSError as e:
                raise RuntimeError(f"transmitter serial device {port}: {e} (plugged in? dialout group? rc_port:=...)") from e
            self.get_logger().info(f"transmitter: {port} ({gp('rc_config')}), PX4 start params + {self.rc_cfg['px4_params']}, "
                                   f"at hand-over {self.rc_cfg['handover_params']}")
        self.presets = cfg["hover_presets"]
        self.ref = self._make_reference({}, np.zeros(3))
        self.ref_active, self.t_ref0 = False, 0.0
        self.hover_freeze_info, self.reference_error = None, None
        self.kin = px4_link.PoseDifference()

        self.lock = threading.RLock()
        self.t = None                       # last plugin sim time
        self.pose = None                    # (p, q, R) last true pose
        self.vel = None                     # (v, w) world
        self.plugin_connected = False
        self.error = None                   # PX4 link error (PX4 stopped until the next restart)
        self.n_exchange = 0
        self.t_next_state = None
        self.dt_warned = False
        self.t_prev = None

        self.get_logger().info(f"PX4 airframe params from {gp('flight_config')}: {json.dumps(auto)}")
        self.get_logger().info(f"PX4 params (PX4_PARAM_*): {json.dumps(self.px4_params)}")

        self.pub_state = self.create_publisher(String, "/drone/px4/state", 10)
        cb_srv = ReentrantCallbackGroup()
        cb_sub = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "/drone/px4/restart", self._srv(self._restart), callback_group=cb_srv)
        self.create_service(Trigger, "/drone/px4/arm", self._srv(self._arm), callback_group=cb_srv)
        self.create_service(Trigger, "/drone/px4/kill", self._srv(self._kill), callback_group=cb_srv)
        self.create_service(Trigger, "/drone/px4/status", self._srv(lambda: self.state()), callback_group=cb_srv)
        self.create_subscription(PointStamped, "/drone/px4/target", self._on_target, 10, callback_group=cb_sub)
        self.create_subscription(String, "/drone/px4/reference_cmd", self._on_reference_cmd, 10, callback_group=cb_sub)

        # CLI answers while sim time stands still (Gazebo paused / plugin not connected): Isaac ticked the commander
        #   every app loop. Only runs when no exchange came for 0.5 s wall, so it never interleaves with lockstep.
        self.wall_last_exchange = time.monotonic()
        self.create_timer(0.25, self._idle_tick, callback_group=cb_sub, clock=rclpy.clock.Clock())

        self.running = True
        self.srv_sock = socket.create_server(("127.0.0.1", int(cfg["plugin_port"])))
        self.srv_sock.settimeout(0.5)
        self.thread = threading.Thread(target=self._plugin_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"waiting for the DroneLockstep plugin on 127.0.0.1:{cfg['plugin_port']}; PX4 "
                               f"{sitl['px4_dir']} airframe {px4c['airframe']} starts on /drone/px4/restart; "
                               f"offboard UDP {int(cfg['offboard_port']) + inst}, CLI UDP {int(cfg['command_port']) + inst}")

    # -- plugin link (lockstep thread) --
    def _plugin_loop(self):
        while self.running:
            try:
                conn, _ = self.srv_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(0.5)
            with self.lock:
                # new connection = drone (re)spawned: like Isaac DroneFlight.teleport, forget the pose / velocity
                # history (else one huge fake velocity / acceleration reaches PX4) and stop the rotors
                self.kin.reset()
                if self.bridge.sensors is not None:
                    self.bridge.sensors.imu.v_prev = None
                self.thrust.reset()
                self.plugin_connected = True
                self.t_prev = None
            self.get_logger().info("plugin connected (lockstep with Gazebo)")
            try:
                while self.running:
                    data = self._recv_exact(conn, IN.size)
                    if data is None:
                        break
                    w0 = time.monotonic()
                    reply = self._exchange(IN.unpack(data))
                    w1 = time.monotonic()
                    conn.sendall(OUT.pack(*reply))
                    if os.environ.get('PX4_BRIDGE_PROFILE'):
                        pr = self.__dict__.setdefault('_prof', {'n': 0, 'ex': 0.0, 'gap': 0.0, 'step': 0.0, 'last': None})
                        if pr['last'] is not None:
                            pr['gap'] += w0 - pr['last']
                        pr['n'] += 1
                        pr['ex'] += w1 - w0
                        pr['step'] += self.__dict__.get('_prof_step', 0.0)
                        pr['last'] = time.monotonic()
                        if pr['n'] % 240 == 0:
                            self.get_logger().info(f"profile per exchange: bridge {pr['ex'] / 240 * 1000:.2f} ms "
                                                   f"(PX4 step {pr['step'] / 240 * 1000:.2f} ms), waiting for the plugin "
                                                   f"{pr['gap'] / 240 * 1000:.2f} ms")
                            pr.update(ex=0.0, gap=0.0, step=0.0)
            except (ConnectionError, OSError) as e:
                self.get_logger().warn(f"plugin connection lost: {e!r}")
            finally:
                conn.close()
                with self.lock:
                    self.plugin_connected = False
            self.get_logger().info("plugin disconnected, waiting for it again")

    def _recv_exact(self, conn, n):
        b = b""
        while len(b) < n:
            try:
                k = conn.recv(n - len(b))
            except socket.timeout:
                if not self.running:
                    return None
                continue
            if not k:
                return None
            b += k
        return b

    def _exchange(self, d):
        t, p, q = d[0], np.array(d[1:4]), np.array(d[4:8])
        v, w = np.array(d[8:11]), np.array(d[11:14])
        R = quat_wxyz_to_R(q)
        dt = self.dt                                    # one exchange = one Isaac physics step
        with self.lock:
            if self.t_prev is not None and abs((t - self.t_prev) - dt) > 1e-6 and not self.dt_warned:
                self.get_logger().warn(f"plugin exchange interval {t - self.t_prev:.6f} s != physics_dt {dt:.6f} s "
                                       "(check the plugin rate)")
                self.dt_warned = True
            self.t_prev = t
            self.wall_last_exchange = time.monotonic()
            self.t, self.pose, self.vel = t, (p, q, R), (v, w)
            self.n_exchange += 1
            reply = np.zeros(10)                            # wrench + prop speeds, zero while PX4 is not running
            if self.bridge.started and self.error is None:
                force, torque = self.thrust.step(dt, R, v)       # previous step's command (one step delay, Isaac order)
                v_s, w_s = self.kin(dt, p, R, v, w)
                try:
                    _w = time.monotonic()
                    self.thrust.set_command(self.bridge.step(dt, p, v_s, R, w_s))
                    self._prof_step = time.monotonic() - _w
                    props = prop_speeds(self.thrust.forces, self.thrust.dir) if self.prop_spin else np.zeros(4)
                    reply = np.concatenate([force, torque, props])
                except Exception as e:  # noqa: BLE001  (lockstep timeout, PX4 died, ...)
                    self._fail(f"PX4 link error at sim {t:.3f} s: {e}")
            else:
                self.thrust.reset()
            try:
                if self.ref_active and self.cmd.target is not None and not self.cmd.external:
                    self.cmd.goto(self.ref(t - self.t_ref0)[0])
                self._rc_update()
                self.cmd.tick(t)
            except Exception as e:  # noqa: BLE001
                self._fail(f"PX4 commander error at sim {t:.3f} s: {e}")
            if self.t_next_state is None or t + 1e-9 >= self.t_next_state:
                self.t_next_state = t + 1.0 / float(self.cfg["state_rate"])
                self.pub_state.publish(String(data=json.dumps(self.state())))
            self.wall_last_exchange = time.monotonic()
        return reply

    def _rc_update(self):
        if self.drone_rc is None:
            return
        try:
            self.drone_rc.update()
        except RuntimeError as e:                          # serial read error (unplugged): transmitter off for this run
            if "serial" not in str(e):
                raise
            self.rc_error = str(e)
            self.get_logger().error(f"{e} -> transmitter off until the bridge restarts, drone back to Offboard hover")
            if self.cmd.want_manual:
                self.cmd.takeback()
            try:
                self.drone_rc.close()
            except OSError:
                pass
            self.drone_rc = None

    def _idle_tick(self):
        with self.lock:
            if time.monotonic() - self.wall_last_exchange < 0.5:
                return
            try:
                self.cmd.tick(0.0 if self.t is None else self.t)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"PX4 commander error (idle): {e!r}")

    def _fail(self, msg):
        """Error: stop PX4 (no silent fallback), zero wrench until the next restart."""
        self.error = msg
        self.get_logger().error(msg + " -> PX4 stopped, zero wrench; call /drone/px4/restart")
        try:
            self.bridge.stop()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"stopping PX4 failed: {e!r}")
        self.thrust.reset()

    # -- state --
    def state(self):
        with self.lock:
            p = q = None
            yaw = None
            if self.pose is not None:
                p, q, R = self.pose
                yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            om = self.thrust.omega
            return {"t": None if self.t is None else round(self.t, 4), "plugin_connected": self.plugin_connected,
                    "px4": self.bridge.status, "lockstep": self.bridge.status == "lockstep", "error": self.error,
                    "armed": bool(self.bridge.armed), "mode": self.cmd.mode, "link_armed": self.cmd.armed,
                    "connected": self.cmd.connected, "landed": self.cmd.landed,
                    "estimate_enu": _r(self.cmd.estimate_enu),
                    "estimate_yaw_deg": None if self.cmd.estimate_yaw is None else round(math.degrees(self.cmd.estimate_yaw), 2),
                    "true_position": _r(p), "true_quat_wxyz": _r(q, 6), "true_yaw_deg": None if yaw is None else round(yaw, 2),
                    "omega": _r(om, 2), "omega_cmd": _r(self.thrust.omega_cmd, 2), "u": _r(self.bridge.u, 4),
                    "thrust_sum": round(float(self.thrust.k * (om ** 2).sum()), 4),
                    "target": _r(self.cmd.target), "target_yaw_deg": round(math.degrees(self.cmd.yaw), 2),
                    "origin": _r(self.cmd.origin), "external": self.cmd.external, "ready": self.cmd.cli_ready,
                    "reference": {**self.ref.params, "active": self.ref_active,
                                  "t0": round(self.t_ref0, 4) if self.ref_active else None,
                                  "freeze": self.hover_freeze_info},
                    "reference_error": self.reference_error,
                    "rc": None if self.rc_cfg is None else {**(self.drone_rc.status() if self.drone_rc else {}),
                                                            "error": self.rc_error}, "run_dir": self.bridge.current_run_dir,
                    "statustext": [m for _, _, m in self.cmd.messages[-3:]]}

    # -- services --
    def _srv(self, fn):
        def cb(req, res):
            try:
                out = fn()
                res.success, res.message = True, json.dumps(out)
            except ServiceError as e:
                res.success, res.message = False, json.dumps({**self.state(), "error": str(e)})
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"service error: {e!r}")
                res.success, res.message = False, json.dumps({**self.state(), "error": repr(e)})
            return res
        return cb

    def _wait(self, cond, timeout_sim, what):
        """Wait (wall sleep) until cond() under the lock; timeout in sim seconds; error when sim time stalls."""
        stall = float(self.cfg["stall_timeout"])
        with self.lock:
            t0 = self.t
        t_last, wall_last = t0, time.monotonic()
        while True:
            with self.lock:
                if self.error is not None:
                    raise ServiceError(f"{what}: {self.error}")
                if cond():
                    return
                t = self.t
            now = time.monotonic()
            if t is not None and t0 is None:
                t0 = t
            if t is not None and t0 is not None and t - t0 > timeout_sim:
                raise ServiceError(f"{what}: timeout {timeout_sim:g} s sim")
            if t != t_last:
                t_last, wall_last = t, now
            elif now - wall_last > stall:
                raise ServiceError(f"{what}: sim time not advancing for {stall:g} s "
                                   f"(Gazebo paused or DroneLockstep plugin not connected: {self.plugin_connected})")
            time.sleep(0.005)

    def _restart(self):
        wall0 = time.monotonic()
        with self.lock:
            if self.bridge.started and self.error is None and self.bridge.armed:
                raise ServiceError("PX4 is armed: kill first (Isaac restart_px4 refuses too)")
            self.error = None
            if self.bridge.sitl["autolaunch"]:
                self.bridge.restart()
            elif not self.bridge.started:
                self.bridge.start()
            else:
                raise ServiceError("autolaunch false: PX4 is managed elsewhere, cannot restart")
            self.cmd.reset_link()
            self.cmd.cli_ready = False
            if self.pose is not None:
                self.cmd.origin = self.pose[0].copy()   # first guess; aligned to the true pose at arm
            self.kin.reset()
            self.thrust.reset()
            self.ref_active, self.hover_freeze_info = False, None
            t0 = self.t
        self.get_logger().info(f"PX4 started: {self.bridge.current_run_dir}")
        self._wait(lambda: self.bridge.status == "lockstep" and self.cmd.connected, float(self.cfg["connect_timeout"]),
                   "PX4 connect (lockstep + offboard link)")
        with self.lock:
            t1 = self.t
        wall1 = time.monotonic()
        ekf = float(self.cfg["ekf_wait"])
        self._wait(lambda: self.t - t1 >= ekf, ekf + 10.0, "ekf_wait")
        with self.lock:
            t2 = self.t
        out = {"run_dir": self.bridge.current_run_dir,
               "phases": [{"name": "px4_connect", "sim_s": None if t0 is None else round(t1 - t0, 3),
                           "wall_s": round(wall1 - wall0, 2)},
                          {"name": "ekf_wait", "sim_s": round(t2 - t1, 3), "wall_s": round(time.monotonic() - wall1, 2)}],
               **self.state()}
        self.get_logger().info(f"PX4 ready (lockstep, ekf_wait done) at sim {t2:.2f} s")
        return out

    def _arm(self):
        with self.lock:
            if self.error is not None:
                raise ServiceError(self.error)
            if not (self.bridge.status == "lockstep" and self.cmd.connected):
                raise ServiceError(f"PX4 not ready (px4 {self.bridge.status}, offboard link {self.cmd.connected}): restart first")
            if self.pose is None:
                raise ServiceError("no drone pose from the plugin yet")
            if self.cmd.estimate_enu is None:
                raise ServiceError("no PX4 position estimate yet")
            p, _, R = self.pose
            yaw = math.atan2(R[1, 0], R[0, 0])
            # PX4 local origin -> world: shift by (true position - PX4 estimate) while still (Isaac DroneFlight.arm;
            #   real drone: register the pose before take-off)
            if not self.bridge.armed and self.cmd.estimate_enu is not None:
                self.cmd.origin = self.cmd.origin + (p - self.cmd.estimate_enu)
                self.cmd.estimate_enu = p.copy()
            self.ref.set_p0(p)
            self.ref_active = False                          # reference starts with /drone/px4/reference_cmd or target
            self.cmd.fly_to(p.copy(), yaw)
            t0 = self.t
        self._wait(lambda: self.bridge.armed and self.cmd.armed and self.cmd.mode == "OFFBOARD",
                   float(self.cfg["arm_timeout"]), "PX4 arm (Offboard)")
        with self.lock:
            self.get_logger().info(f"PX4 armed in OFFBOARD at sim {self.t:.2f} s ({self.t - t0:.2f} s after the request), "
                                   f"target {_r(self.cmd.target)}, yaw {math.degrees(self.cmd.yaw):.1f} deg")
            return {"arm_sim_s": round(self.t - t0, 3), **self.state()}

    def _kill(self):
        with self.lock:
            if not (self.bridge.started and self.bridge.armed):
                raise ServiceError("PX4 not armed (already off)")
            self.hover_freeze("kill")
            self.cmd.kill()
            self.get_logger().info(f"PX4 kill at sim {self.t:.3f} s")
            return {"t": round(self.t, 4), "hover_freeze": self.hover_freeze_info, **self.state()}

    # -- reference (Isaac DroneFlight hover_reseed / hover_freeze / _on_cli) --
    def _make_reference(self, req, p0_default):
        mode = req.get("mode", self.get_parameter("hover_mode").value)
        preset = req.get("preset", self.get_parameter("hover_preset").value) if mode == "hover" else None
        seed = int(req.get("seed", self.get_parameter("hover_seed").value))
        p0 = req.get("p0", p0_default)
        if p0 is None or len(p0) != 3:
            raise ValueError(f"p0 must be [x, y, z]: {p0}")
        return Reference(mode, [float(x) for x in p0], self.presets, seed, preset=preset)

    def _activate_reference(self, req):
        with self.lock:
            p_def = self.ref.p0 if self.ref_active else (self.pose[0] if self.pose is not None else None)
            if "p0" not in req and p_def is None:
                raise ValueError("no p0 and no drone pose yet")
            self.ref = self._make_reference(req, None if p_def is None else p_def.tolist())
            self.t_ref0 = self.t if self.t is not None else 0.0
            self.ref_active = True
            self.hover_freeze_info = None
            self.cmd.cli_ready = bool(req.get("ready", True))   # CLI / transmitter accepted from now (Isaac: after take-off)
            if self.cmd.target is not None and not self.cmd.external:
                self.cmd.goto(self.ref(0.0)[0])
            self.reference_error = None
            self.get_logger().info(f"reference started at sim {self.t_ref0:.3f} s: {json.dumps(self.ref.params)}")

    def _on_reference_cmd(self, msg):
        try:
            req = json.loads(msg.data) if msg.data.strip() else {}
            if not isinstance(req, dict):
                raise ValueError("JSON object expected")
            unknown = set(req) - {"mode", "preset", "seed", "p0", "ready"}
            if unknown:
                raise ValueError(f"unknown keys {sorted(unknown)} (mode, preset, seed, p0, ready)")
            self._activate_reference(req)
        except (ValueError, KeyError) as e:
            with self.lock:
                self.reference_error = str(e)
            self.get_logger().error(f"/drone/px4/reference_cmd rejected: {e}")

    def _on_target(self, msg):
        if msg.header.frame_id not in ("", "world"):
            self.get_logger().error(f"/drone/px4/target frame '{msg.header.frame_id}' rejected (world expected)")
            return
        p0 = [msg.point.x, msg.point.y, msg.point.z]
        with self.lock:
            if self.ref_active:
                self.ref.set_p0(p0)                 # sine continues; static: the target follows in the next exchange
                self.get_logger().info(f"reference p0 -> {_r(self.ref.p0)}")
                return
        try:
            self._activate_reference({"p0": p0})
        except (ValueError, KeyError) as e:
            with self.lock:
                self.reference_error = str(e)
            self.get_logger().error(f"/drone/px4/target rejected: {e}")

    def hover_freeze(self, reason):
        if not self.ref_active:
            return False
        ok = self.ref.freeze(self.t - self.t_ref0, reason)
        if ok:
            self.hover_freeze_info = {"reason": reason, "t": round(self.t, 3), "target": _r(self.ref.p_freeze)}
            self.get_logger().info(f"hover sine frozen ({reason}) at sim {self.t:.2f} s, target {_r(self.ref.p_freeze)}")
        return ok

    def _on_cli(self, cmd, pos=None, yaw=None):
        """drone_cmd.py CLI vs hover (PX4Commander.hover_hook, called in the lockstep thread under the lock)."""
        if self.ref.mode != "hover":
            return False
        if cmd == "goto":
            if yaw is not None:
                raise ValueError("hover mode: goto takes no yaw (moves the reference position only)")
            self.ref.set_p0(pos)
            self.get_logger().info(f"hover reference -> {_r(self.ref.p0)} (CLI goto, sine continues)")
            return True
        if cmd == "hold":
            self.hover_freeze("cli_hold")
            return True
        return False

    def shutdown(self):
        self.running = False
        try:
            self.srv_sock.close()
        except OSError:
            pass
        self.thread.join(timeout=2.0)
        with self.lock:
            self.bridge.stop()
            self.cmd.close()
            if self.drone_rc is not None:
                self.drone_rc.close()


def main():
    rclpy.init()
    node = PX4BridgeNode()
    # EventsExecutor (one thread): the MultiThreadedExecutor busy-looped while a service waited (restart / arm block for
    # seconds) and took ~90 % of a core and the GIL, so the lockstep socket thread got 10x slower (RTF 0.07, 2026-11-03).
    # A blocking service only holds this executor; the lockstep runs in its own thread, waits use the plugin sim time.
    ex = EventsExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        print("[px4_bridge] PX4 stopped, bridge closed", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
