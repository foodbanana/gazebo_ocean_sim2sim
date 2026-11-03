"""Rotor thrust model of the Isaac Sim drone (data_collection_ur5_gripper isaacsim/scripts/drone_flight.py DroneFlight._step,
from Pegasus Simulator QuadraticThrustCurve + LinearDrag + PX4 Gazebo motor_model lag). Same order and equations:

  1. the omega command of the PREVIOUS step is applied now (one step delay), through the first-order motor lag
         omega += (omega_cmd - omega) (1 - exp(-dt / tau)),  tau = tau_up if omega_cmd > omega else tau_down
  2. thrust F_i = k omega_i^2 along the rotor axis at the rotor position
     reaction torque sum(c omega_i^2 dir_i) about the body z axis, linear drag -R D R^T v at the body origin
  3. the new command (from PX4) is stored for the next step

Values come from isaac_flight.yaml (exported from Isaac Sim: rotor positions / axes / directions, k, c, drag, PX4
motor mapping omega = scaling u + zero_position_armed). Gazebo applies the result as one wrench at the body origin
(world frame) = the same forces as Isaac Sim's per-rotor forces + body torque.
"""
import numpy as np
import yaml


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def load_flight(path):
    with open(path) as f:
        return yaml.safe_load(f)


class ThrustModel:
    def __init__(self, fl):
        self.pos = np.array([r['position'] for r in fl['rotors']], dtype=float)       # body frame [m]
        self.axis = np.array([r['thrust_axis'] for r in fl['rotors']], dtype=float)   # body frame
        self.dir = np.array([r['direction'] for r in fl['rotors']], dtype=float)
        self.k = float(fl['rotor_constant'])
        self.c = float(fl['rolling_moment_coefficient'])
        self.w_max = float(fl['max_rotor_velocity'])
        self.drag = np.array(fl['linear_drag'], dtype=float)
        md = (fl.get('px4') or {}).get('motor_dynamics') or {}
        self.tau = (float(md['tau_up']), float(md['tau_down'])) if md.get('enabled') else None
        self.omega = np.zeros(len(self.dir))
        self.omega_cmd = np.zeros(len(self.dir))

    def reset(self):
        self.omega[:] = 0.0
        self.omega_cmd[:] = 0.0

    def step(self, dt, R, v):
        """Apply the stored command (lag) and return the wrench (force, torque about the body origin), world frame."""
        if self.tau is None:
            self.omega = self.omega_cmd.copy()
        else:
            tau = np.where(self.omega_cmd > self.omega, self.tau[0], self.tau[1])
            self.omega = self.omega + (self.omega_cmd - self.omega) * (1.0 - np.exp(-dt / tau))
        F = self.k * self.omega ** 2
        f_rot = (R @ self.axis.T).T * F[:, None]                     # world, per rotor
        r_rot = (R @ self.pos.T).T                                   # world offset from the body origin
        force = f_rot.sum(axis=0) + R @ (-self.drag * (R.T @ v))
        torque = np.cross(r_rot, f_rot).sum(axis=0) + float((self.c * self.omega ** 2 * self.dir).sum()) * R[:, 2]
        return force, torque

    def set_command(self, omega_cmd):
        self.omega_cmd = np.clip(np.asarray(omega_cmd, dtype=float), 0.0, self.w_max)

    def hover_omega(self, mass, g=9.81):
        return float(np.sqrt(mass * g / (len(self.dir) * self.k)))
