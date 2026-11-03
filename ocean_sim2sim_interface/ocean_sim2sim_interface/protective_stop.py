"""UR-style protective stop decision (no ROS here, unit-testable).

Same rules and thresholds as the Isaac Sim reference (data_collection_ur5_gripper isaacsim/scripts/protective_stop.py,
isaacsim/config/ros2_iface.yaml protective_stop, docs/PLAN.md 3-7). Checked on every physics step, any of
  force : contact force on a robot link [N] > contact_force_n
            non-finger links: all contacts (total_N: environment + robot itself + drone)
            finger links    : environment only (env_N), so squeezing the drone or the fingers touching each other
                              never trips it; a finger hitting the boat does
  error : |joint target - measured| [deg] of an arm joint > tracking_error_deg (target = interpolated drive target)
  speed : arm joint speed [deg/s] from joint angle differences > joint_speed_dps
mode "on": the first sample over a limit stops (latched until clear()). mode "measure": never stops, only keeps the
maxima (and reports once per kind that it would have stopped), to check the margins in normal operation.

Each check is fed separately (contacts, controller state, joint states arrive on different topics, each once per
physics step): update_*() return True when this sample started the stop.
"""
import math

MODES = ('on', 'measure')
KINDS = ('force', 'error', 'speed')
UNITS = {'force': 'N', 'error': 'deg', 'speed': 'deg/s'}


class ProtectiveStop:
    def __init__(self, mode, contact_force_n, tracking_error_deg, joint_speed_dps, finger_links, robot_links,
                 arm_joints):
        if mode not in MODES:
            raise ValueError(f'protective_stop.mode must be one of {MODES}: {mode!r}')
        self.mode = mode
        self.lim = {'force': float(contact_force_n), 'error': float(tracking_error_deg),
                    'speed': float(joint_speed_dps)}
        for k, v in self.lim.items():
            if not v > 0:
                raise ValueError(f'protective_stop limit {k} must be > 0: {v}')
        self.finger_links = set(finger_links)
        self.robot_links = set(robot_links)
        missing = self.finger_links - self.robot_links
        if missing:
            raise ValueError(f'protective_stop.finger_links not in robot_links: {sorted(missing)}')
        self.arm_joints = list(arm_joints)
        self.latest = {k: (0.0, None, None) for k in KINDS}    # (value, link / joint, t) of the last sample
        self.max = {k: (0.0, None, None) for k in KINDS}       # maxima while not stopped
        self.would_stop = set()                                # measure mode: kinds that went over once
        self.stopped, self.reason, self.t_stop = False, None, None
        self._q_prev = None                                    # (t, [q]) for the joint speed

    # ------------------------------------------------------------- samples
    def update_contacts(self, t, links):
        """links = {sdf link name: {'total_N', 'env_N', ...}} of one physics step (ContactMonitor JSON)."""
        unknown = set(links) - self.robot_links
        if unknown:
            raise ValueError(f'contact data for links not in protective_stop.robot_links: {sorted(unknown)}')
        value, where = 0.0, None
        for name, f in links.items():
            v = float(f['env_N'] if name in self.finger_links else f['total_N'])
            if v > value:
                value, where = v, name
        return self._sample('force', value, where, t)

    def update_tracking(self, t, errors):
        """errors = {arm joint: target - measured [rad]} (other joints ignored)."""
        value, where = 0.0, None
        for j in self.arm_joints:
            if j in errors:
                v = abs(math.degrees(errors[j]))
                if v > value:
                    value, where = v, j
        return self._sample('error', value, where, t)

    def update_positions(self, t, q):
        """q = {joint: measured position [rad]} at sim time t [s]; speed from the difference to the previous sample."""
        qa = [q[j] for j in self.arm_joints]
        prev, self._q_prev = self._q_prev, (t, qa)
        if prev is None or not t > prev[0]:
            return False
        dt = t - prev[0]
        value, where = 0.0, None
        for j, a, b in zip(self.arm_joints, qa, prev[1]):
            v = abs(math.degrees(a - b)) / dt
            if v > value:
                value, where = v, j
        return self._sample('speed', value, where, t)

    def resync(self):
        """Forget the previous joint sample (after a pause in the checks, e.g. the episode reset)."""
        self._q_prev = None

    def clear(self):
        """Release the stop (episode reset)."""
        self.stopped, self.reason, self.t_stop = False, None, None
        self.resync()

    # ------------------------------------------------------------- decision
    def _sample(self, kind, value, where, t):
        self.latest[kind] = (value, where, t)
        if self.stopped:                       # values after the stop (impact) are not normal-operation maxima
            return False
        if value > self.max[kind][0]:
            self.max[kind] = (value, where, t)
        if not value > self.lim[kind]:
            return False
        if self.mode == 'measure':
            self.would_stop.add(kind)
            return False
        # reason = every kind whose latest sample is over its limit (the others came in this or the previous step)
        self.stopped, self.t_stop = True, t
        self.reason = '; '.join(f'{k} {x:.1f} {UNITS[k]} ({w}) > {self.lim[k]:g}'
                                for k, (x, w, _) in self.latest.items() if x > self.lim[k])
        return True

    def would_stop_reason(self, kind):
        x, w, t = self.latest[kind]
        return f'{kind} {x:.1f} {UNITS[kind]} ({w}) > {self.lim[kind]:g} at t {t:.3f} s'

    def summary(self):
        parts = [f'{k} max {x:.1f} {UNITS[k]} ({w}, t {t:.2f}) / limit {self.lim[k]:g}' if w else f'{k} max 0'
                 for k, (x, w, t) in self.max.items()]
        st = f'STOPPED at t {self.t_stop:.3f}: {self.reason}' if self.stopped else 'not stopped'
        return f'protective stop [{self.mode}] {st} | ' + ', '.join(parts)
