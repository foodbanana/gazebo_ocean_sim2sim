# =============================================================
# sensors.py - virtual sensors sent to PX4 SITL (IMU, barometer, magnetometer, motion capture)
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/px4_sensors.py (Isaac Sim PX4 link), which itself is from
#     livealive7/PegasusSimulator b1256ca logic/sensors/{imu,barometer,magnetometer}.py
#     (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto, license: LICENSE.PegasusSimulator next to this file).
#     Equations follow the PX4 sitl_gazebo plugins (Pegasus original comment).
#   Input state (Isaac Sim / Gazebo convention): p, v = body position / velocity (world ENU),
#     R = body (FLU) -> world rotation, w = angular velocity (body FLU).
#   Output (PX4 convention): body FRD, world NED. IMU acceleration = velocity difference - g (contact forces show up too).
#
#   pegasus_compat: true = Pegasus code behaviour including its bugs (Isaac P-2 baseline). false (used) = PX4 sitl_gazebo
#     equations:
#     - gyro bias correlation time: Pegasus uses the accelerometer value (300 s) -> gyro value (1000 s)
#     - accelerometer bias: Pegasus updates it with rand (uniform 0..1) and never adds it -> randn, added
#     - barometer noise: Pegasus feeds randn into Box-Muller -> standard normal 1 Pa (randn), drift = accumulated value
#   MotionCapture (Isaac default position source "mocap", not in Pegasus): world body position / attitude + noise,
#     rate, latency -> MAVLink VISION_POSITION_ESTIMATE (NED position, FRD body roll / pitch / yaw).
#     world = mocap frame = PX4 local frame (same origin).
#   Differences to Pegasus (also in compat): random numbers from a seeded np.random.Generator (reproducible, Pegasus uses
#     the global np.random), sensor rates counted in integer physics steps (Pegasus compares accumulated time).
#
#   Changes from the Isaac Sim file: GPS, RangeFinder and OpticalFlow removed (only position_source mocap is ported;
#     they were disabled in that mode so the random number stream is unchanged: the generator is shared and drawn in the
#     order IMU, barometer, magnetometer, mocap exactly as before). Comments translated to English. Everything else
#     (equations, defaults, rates, latency, noise, bias, seeding) is identical.
# =============================================================

import functools
import math

import numpy as np

from .pegasus_geo_mag import GRAVITY_VECTOR, get_mag_declination, get_mag_inclination, get_mag_strength, reprojection


def enu_to_ned(x):
    return np.array([x[1], x[0], -x[2]])


def flu_to_frd(x):
    return np.array([x[0], -x[1], -x[2]])


def _bias_sigma(random_walk, tau, dt):
    """Std of the discrete-time bias process [Maybeck 4-114] (same as Pegasus / sitl_gazebo)."""
    return math.sqrt(-random_walk * random_walk * tau / 2.0 * (math.exp(-2.0 * dt / tau) - 1.0))


class RateGate:
    """Counts physics steps, True every update_rate. The first call is always True (Pegasus _first_update)."""

    def __init__(self, rate, dt):
        self.n = max(1, int(round(1.0 / (rate * dt))))
        self.period = self.n * dt
        self.k = 0

    def __call__(self):
        due = self.k % self.n == 0
        self.k += 1
        return due


class IMU:
    def __init__(self, c, dt, rng, compat):
        g, a = c["gyroscope"], c["accelerometer"]
        self.g_nd, self.g_rw, self.g_tau = g["noise_density"], g["random_walk"], g["bias_correlation_time"]
        self.a_nd, self.a_rw, self.a_tau = a["noise_density"], a["random_walk"], a["bias_correlation_time"]
        self.gate = RateGate(c["update_rate"], dt)
        self.rng, self.compat = rng, compat
        self.g_bias, self.a_bias = np.zeros(3), np.zeros(3)
        self.v_prev = None

    def update(self, p, v, R, w):
        if not self.gate():
            return None
        dt = self.gate.period
        rng = self.rng
        tau_g = self.a_tau if self.compat else self.g_tau
        s_g = self.g_nd / math.sqrt(dt)
        sb_g = _bias_sigma(self.g_rw, tau_g, dt)
        self.g_bias = math.exp(-dt / tau_g) * self.g_bias + sb_g * rng.standard_normal(3)
        gyro = w + s_g * rng.standard_normal(3) + self.g_bias

        s_a = self.a_nd / math.sqrt(dt)
        sb_a = _bias_sigma(self.a_rw, self.a_tau, dt)
        v_prev = np.zeros(3) if self.v_prev is None else self.v_prev   # Pegasus: previous velocity starts at 0
        acc_world = (v - v_prev) / dt - GRAVITY_VECTOR
        self.v_prev = v.copy()
        acc = R.T @ acc_world
        if self.compat:
            self.a_bias = math.exp(-dt / self.a_tau) * self.a_bias + sb_a * rng.random(3)
            acc = acc + s_a * rng.standard_normal(3)
        else:
            self.a_bias = math.exp(-dt / self.a_tau) * self.a_bias + sb_a * rng.standard_normal(3)
            acc = acc + s_a * rng.standard_normal(3) + self.a_bias
        return {"gyro": flu_to_frd(gyro), "acc": flu_to_frd(acc)}


class Barometer:
    T_MSL, P_MSL, LAPSE, RHO_MSL, ABS_ZERO = 288.15, 101325.0, 0.0065, 1.225, -273.15

    def __init__(self, c, dt, rng, compat, origin_alt):
        self.gate = RateGate(c["update_rate"], dt)
        self.drift_rate = c["drift_pa_per_sec"]
        self.rng, self.compat, self.origin_alt = rng, compat, origin_alt
        self.z0, self.drift, self._y2 = None, 0.0, None

    def _noise(self):
        if not self.compat:
            return float(self.rng.standard_normal())
        # Pegasus as is: polar Box-Muller fed with randn instead of uniform numbers
        if self._y2 is not None:
            y, self._y2 = self._y2, None
            return y
        w = 1.0
        while w >= 1.0:
            x1 = 2.0 * self.rng.standard_normal() - 1.0
            x2 = 2.0 * self.rng.standard_normal() - 1.0
            w = x1 * x1 + x2 * x2
        w = math.sqrt(-2.0 * math.log(w) / w)
        self._y2 = x2 * w
        return x1 * w

    def update(self, p):
        if not self.gate():
            return None
        dt = self.gate.period
        if self.z0 is None:
            self.z0 = p[2]
        alt = self.origin_alt + (p[2] - self.z0)
        T = self.T_MSL - self.LAPSE * alt
        pressure = self.P_MSL / (self.T_MSL / T) ** 5.2561
        n = self._noise()
        self.drift += self.drift_rate * dt
        # Pegasus as is (compat): adds the drift rate (Pa/s) instead of the accumulated drift (default 0: no effect)
        p_noisy = pressure + n + (self.drift_rate if self.compat else self.drift)
        rho = self.RHO_MSL / (self.T_MSL / T) ** 4.256
        p_alt = alt - (n + self.drift) / (np.linalg.norm(GRAVITY_VECTOR) * rho)
        return {"abs_pressure_hpa": p_noisy * 0.01, "pressure_alt": p_alt, "temperature": T + self.ABS_ZERO}


@functools.lru_cache(maxsize=4096)
def _mag_field_enu(lat, lon):
    """Earth magnetic field (ENU, gauss). Cached on a 1e-5 deg (about 1.1 m) lat / lon grid (table interpolation is slow
    in Python); the field difference within 1 m is far below the sensor noise."""
    dec, inc = math.radians(get_mag_declination(lat, lon)), math.radians(get_mag_inclination(lat, lon))
    s = 0.01 * get_mag_strength(lat, lon)                        # gauss
    H = s * math.cos(inc)
    field_ned = np.array([H * math.cos(dec), H * math.sin(dec), math.tan(inc) * H])
    field_enu = enu_to_ned(field_ned)                            # NED <-> ENU is the same swap
    field_enu.setflags(write=False)                              # cached value must not change
    return field_enu


class Magnetometer:
    def __init__(self, c, dt, rng, origin):
        self.nd, self.rw, self.tau = c["noise_density"], c["random_walk"], c["bias_correlation_time"]
        self.gate = RateGate(c["update_rate"], dt)
        self.rng, self.origin = rng, origin
        self.bias = np.zeros(3)

    def update(self, p, R):
        if not self.gate():
            return None
        dt = self.gate.period
        lat, lon = reprojection(p, math.radians(self.origin[0]), math.radians(self.origin[1]))
        body = flu_to_frd(R.T @ _mag_field_enu(round(math.degrees(lat), 5), round(math.degrees(lon), 5)))
        sd = self.nd / math.sqrt(dt)
        sb = _bias_sigma(self.rw, self.tau, dt)
        self.bias = math.exp(-dt / self.tau) * self.bias + sb * self.rng.standard_normal(3)
        return {"mag": body + sd * self.rng.standard_normal(3) + self.bias}


class MotionCapture:
    """Every period: the true pose `latency` ago plus noise (latency rounded to physics steps)."""

    def __init__(self, c, dt, rng):
        self.gate = RateGate(c["update_rate"], dt)
        self.lag = int(round(float(c["latency"]) / dt))
        self.p_std, self.a_std = float(c["position_noise_std"]), float(c["angle_noise_std"])
        self.rng, self.hist = rng, []

    def update(self, p, R):
        self.hist.append((p.copy(), R.copy()))
        if len(self.hist) > self.lag + 1:
            self.hist.pop(0)
        if not self.gate():
            return None
        p0, R0 = self.hist[0]                                   # lag steps ago (at the start: as many as there are)
        T = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])      # ENU -> NED
        Rn = T @ R0 @ np.diag([1.0, -1.0, -1.0])                # FRD body -> NED
        euler = np.array([math.atan2(Rn[2, 1], Rn[2, 2]), -math.asin(max(-1.0, min(1.0, Rn[2, 0]))), math.atan2(Rn[1, 0], Rn[0, 0])])
        return {"pos_ned": T @ p0 + self.p_std * self.rng.standard_normal(3),
                "euler": euler + self.a_std * self.rng.standard_normal(3)}


class SensorSuite:
    """Sensor config (drone_px4.yaml sensors, mocap flag from the position source) -> sensors. update() is called every
    physics step and returns only the values that are new in this step."""

    def __init__(self, c, dt, seed):
        self.compat = bool(c["pegasus_compat"])
        for k in ("gps", "rangefinder", "optical_flow"):
            if (c.get(k) or {}).get("enabled"):
                raise ValueError(f"sensor '{k}' is not ported (only position_source mocap): disable it")
        rng = np.random.default_rng(seed)
        origin = tuple(float(x) for x in c["origin_lat_lon_alt"])
        self.imu = IMU(c["imu"], dt, rng, self.compat)
        self.baro = Barometer(c["barometer"], dt, rng, self.compat, origin[2])
        self.mag = Magnetometer(c["magnetometer"], dt, rng, origin)
        self.mocap = MotionCapture(c["mocap"], dt, rng) if c["mocap"]["enabled"] else None

    def update(self, p, v, R, w):
        out = {"imu": self.imu.update(p, v, R, w), "baro": self.baro.update(p), "mag": self.mag.update(p, R)}
        out["mocap"] = self.mocap.update(p, R) if self.mocap is not None else None
        return out
