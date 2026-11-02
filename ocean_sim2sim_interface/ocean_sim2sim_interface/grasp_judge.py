"""Drone grasp judge, ported from Isaac Sim (data_collection_ur5_gripper isaacsim/scripts/grasp_judge.py, same rules
and thresholds) so Gazebo and Isaac Sim results are judged the same way. docs/grasp_demo.md

Real scenario: grab the hovering drone -> drone motors off -> a person takes it. Success = held (no putting down).
States: approach -> grasped -> held (success), or failed + reason. Judged at every observation (update).
  grasped: while the close command is on, the gripper stopped (position change < stop_vel * stop_time over stop_time),
           less closed than the empty close angle, contact force > contact_force on both fingers
  held:    hold_time after the motors stop: drone settle (in the TCP frame) <= settle_max, change in the last still_window
           <= still_mm, rotation <= hold_rot. Grabbed from below, the drone settles a few mm onto the fingers and stops.
  failure reasons: no_grasp_empty (closed fully, empty), no_grasp_contact (stopped, not both fingers in contact),
           slip, dropped (moved > drop_mm from the TCP after grasping), opened_early, timeout:<state>
Observation dict: t, grip_q, grip_closing, armed, f_right, f_left, drone_p, drone_R, tcp_p, tcp_R (world, numpy).
"""
import math
from collections import deque

import numpy as np

JUDGE_KEYS = ('stop_vel', 'stop_time', 'empty_close_deg', 'contact_force', 'hold_time', 'settle_max_mm', 'still_window',
              'still_mm', 'hold_rot_deg', 'drop_mm')


def rot_angle(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class GraspJudge:
    def __init__(self, cfg, q_pred=None):
        for k in JUDGE_KEYS:
            if k not in cfg:
                raise KeyError(f'judge.{k} missing')
        self.c = cfg
        self.q_pred = q_pred
        self.state, self.reason = 'approach', None
        self.events = []                    # (t, state, info)
        self.hist = deque()                 # (t, grip_q) while closing
        self.rel0 = None                    # drone in the TCP frame when grasped (R, p)
        self.t_release = None
        self.max_slip_mm, self.max_rot_deg = 0.0, 0.0
        self.settle_mm = None
        self.recent = deque()               # (t, drone position in the TCP frame) after the motors stop
        self.grasp_info = {}

    def _rel(self, o):
        Rt = o['tcp_R']
        return Rt.T @ o['drone_R'], Rt.T @ (o['drone_p'] - o['tcp_p'])

    def _drift(self, o):
        R, p = self._rel(o)
        return float(np.linalg.norm(p - self.rel0[1]) * 1000), rot_angle(self.rel0[0].T @ R)

    def _go(self, t, state, **info):
        self.state = state
        self.events.append((t, state, info))

    def fail(self, t, reason, **info):
        if self.done:
            return
        self.reason = reason
        self._go(t, 'failed', reason=reason, **info)

    @property
    def done(self):
        return self.state in ('failed', 'held')

    def update(self, o):
        if self.done:
            return
        c, t = self.c, o['t']
        if self.state == 'approach':
            if not o['grip_closing']:
                self.hist.clear()
                return
            self.hist.append((t, o['grip_q']))
            while self.hist and self.hist[0][0] < t - c['stop_time']:
                self.hist.popleft()
            if t - self.hist[0][0] < 0.9 * c['stop_time'] or len(self.hist) < 3:
                return
            if abs(self.hist[-1][1] - self.hist[0][1]) >= c['stop_vel'] * c['stop_time']:
                return
            q = o['grip_q']
            info = dict(q_deg=math.degrees(q), f_right=o['f_right'], f_left=o['f_left'])
            if self.q_pred is not None:
                info['q_pred_deg'] = math.degrees(self.q_pred)
            if math.degrees(q) >= c['empty_close_deg']:
                self.fail(t, 'no_grasp_empty', **info)
            elif min(o['f_right'], o['f_left']) < c['contact_force']:
                self.fail(t, 'no_grasp_contact', **info)
            else:
                self.rel0 = self._rel(o)
                self.grasp_info = info
                self._go(t, 'grasped', **info)
            return

        # grasped: wait for the motors to stop, then see if it holds
        if not o['grip_closing']:
            self.fail(t, 'opened_early')
            return
        slip, rot = self._drift(o)
        self.max_slip_mm, self.max_rot_deg = max(self.max_slip_mm, slip), max(self.max_rot_deg, rot)
        if slip > c['drop_mm']:
            self.fail(t, 'dropped', slip_mm=slip, rot_deg=rot)
            return
        if o['armed']:
            return
        if self.t_release is None:
            self.t_release = t
        self.recent.append((t, self._rel(o)[1].copy()))
        while self.recent and self.recent[0][0] < t - c['still_window']:
            self.recent.popleft()
        if slip > c['settle_max_mm'] or rot > c['hold_rot_deg']:
            self.fail(t, 'slip', phase='settle', slip_mm=slip, rot_deg=rot)
            return
        if t - self.t_release >= c['hold_time']:
            moved = float(np.linalg.norm(self.recent[-1][1] - self.recent[0][1]) * 1000)
            if moved > c['still_mm']:
                self.fail(t, 'slip', phase='still', moved_mm=moved, slip_mm=slip)
                return
            self.settle_mm = slip
            self._go(t, 'held', settle_mm=slip, still_mm=moved, rot_deg=rot)

    def finish(self, t):
        if not self.done:
            self.fail(t, f'timeout:{self.state}')
        return self.result()

    def result(self):
        return {'success': self.state == 'held', 'state': self.state, 'reason': self.reason,
                'events': [(round(t, 3), s, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in i.items()})
                           for t, s, i in self.events],
                'max_slip_mm': round(self.max_slip_mm, 3), 'max_rot_deg': round(self.max_rot_deg, 3),
                'settle_mm': None if self.settle_mm is None else round(self.settle_mm, 3),
                'grasp': {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.grasp_info.items()}}
