#!/usr/bin/env python3
"""Drive tests for the arm and gripper in Gazebo through the Isaac Sim / real-robot interface
(/joint_command, /gripper/command). Sim time. See docs/gain_tuning.md and docs/interface.md.

Run while `ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py` is up:

    ros2 run ocean_sim2sim_bringup drive_test.py                 # all tests
    ros2 run ocean_sim2sim_bringup drive_test.py --tests a c     # only test A and C
    ros2 run ocean_sim2sim_bringup drive_test.py --tests step --step 0.2 --joint shoulder_lift_joint

Tests (A, C, D are the same as the Isaac Sim drive tuning tests):
    hold    : hold the home pose for 10 s, max |q - home|
    interp  : one 0.03 rad command on wrist_3 -> reference ramp, start / 50 % times (Isaac Sim check_ros2 #4)
    a       : one joint at a time, 0.05 rad step -> overshoot, settling time (2 %), steady-state error
    c       : one joint at a time, 50 Hz staircase of a 0.5 Hz sine (0.3 rad) -> latency, RMS error
    step    : one step (--step rad) on --joint (or all joints) -> time series
    gripper : test D: close (raw 1150) and open (raw 0) -> time, reaction, mimic error, /gripper/joint_states
"""
import argparse
import math

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
FINGERS = ['rh_r1_joint', 'rh_r2', 'rh_l1', 'rh_l2']
RAW_MAX, UPPER = 1150.0, 1.13514578304


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.log = []       # [(t, {joint: position})]  all joints, every physics step
        self.ref = []       # [(t, [arm reference])]    arm_controller reference
        self.grip_raw = []  # [(t, present raw)]        /gripper/joint_states
        self.create_subscription(JointState, '/joint_state_broadcaster/joint_states',
                                 lambda m: self.log.append((stamp(m), dict(zip(m.name, m.position)))), 200)
        self.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state',
                                 lambda m: self.ref.append((stamp(m), list(m.reference.positions))), 200)
        self.create_subscription(JointState, '/gripper/joint_states',
                                 lambda m: self.grip_raw.append((stamp(m), m.position[0])), 50)
        self.arm_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.grip_pub = self.create_publisher(Float64, '/gripper/command', 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_sec):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sim_sec:
            rclpy.spin_once(self, timeout_sec=0.002)

    def arm(self, q):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ARM
        m.position = [float(x) for x in q]
        self.arm_pub.publish(m)

    def gripper(self, raw):
        self.grip_pub.publish(Float64(data=float(raw)))

    def go_home(self, sec=1.5):
        self.arm(HOME)
        self.gripper(0.0)
        self.spin_for(sec)

    def since(self, t0, log=None):
        return [(t - t0, d) for t, d in (self.log if log is None else log) if t >= t0]


def test_hold(n):
    n.go_home(2.0)
    n.log.clear()
    n.spin_for(10.0)
    print('HOLD 10 s: max |q - home| [deg]')
    for j, h in zip(ARM, HOME):
        print('  %-20s %.4f' % (j, math.degrees(max(abs(d[j] - h) for _, d in n.log))))


def test_interp(n, size=0.03, trials=8):
    """Isaac Sim check_ros2 #4 (0.03 rad step through /joint_command): start moving 33-50 ms, 50 % at 50-67 ms.
    Repeated with the command sent at different times (the wait for the 30 Hz command loop varies)."""
    k = ARM.index('wrist_3_joint')
    print('INTERP: %d x one %.3f rad command on wrist_3_joint' % (trials, size))
    starts, t50s, peaks = [], [], []
    for i in range(trials):
        n.go_home(1.0 + i * 0.0047)       # shift the send time by ~1 physics step each trial
        tgt = list(HOME)
        tgt[k] += size
        n.ref.clear()
        n.log.clear()
        t0 = n.now()
        n.arm(tgt)
        n.spin_for(0.4)
        if i == 0:
            print('  first trial:  %7s %12s %12s' % ('t [ms]', 'reference', 'measured'))
            for t, r in n.since(t0, n.ref):
                if t > 0.09:
                    break
                meas = min(n.since(t0), key=lambda x: abs(x[0] - t))[1]['wrist_3_joint']
                print('                %7.1f %12.4f %12.4f' % (t * 1000, r[k] - HOME[k], meas - HOME[k]))
        rel = [(t, (d['wrist_3_joint'] - HOME[k]) / size) for t, d in n.since(t0)]
        starts.append(next((t for t, r in rel if r > 0.01), float('nan')) * 1000)
        t50s.append(next((t for t, r in rel if r >= 0.5), float('nan')) * 1000)
        peaks.append((max(r for _, r in rel) - 1) * 100)
    print('  start moving %.1f ~ %.1f ms, 50 %% at %.1f ~ %.1f ms, overshoot max %.1f %%'
          % (min(starts), max(starts), min(t50s), max(t50s), max(peaks)))
    print('  (Isaac Sim: start 33 ~ 50 ms, 50 % at 50 ~ 67 ms, overshoot 0 %)')
    n.go_home()


def test_a(n, size=0.05):
    print('TEST A: %.3f rad step, one joint at a time' % size)
    print('  %-20s %10s %10s %12s %12s' % ('joint', 'overshoot', 'settle', 'ss error', 'other jnts'))
    for k, j in enumerate(ARM):
        n.go_home()
        tgt = list(HOME)
        tgt[k] += size
        n.log.clear()
        t0 = n.now()
        n.arm(tgt)
        n.spin_for(1.0)
        rel = [(t, (d[j] - HOME[k]) / size) for t, d in n.since(t0)]
        peak = max(r for _, r in rel)
        settle = next((t for i, (t, _) in enumerate(rel) if all(abs(r - 1) < 0.02 for _, r in rel[i:])),
                      float('nan'))
        others = max(abs(d[o] - HOME[m]) for _, d in n.since(t0) for m, o in enumerate(ARM) if o != j)
        ss = abs(n.log[-1][1][j] - tgt[k])
        print('  %-20s %9.1f%% %9.3fs %10.4f deg %10.4f deg'
              % (j, (peak - 1) * 100, settle, math.degrees(ss), math.degrees(others)))
    n.go_home()


def test_c(n, amp=0.3, freq=0.5, cmd_hz=50.0, dur=4.0):
    print('TEST C: %.0f Hz staircase of a %.1f Hz sine, amplitude %.2f rad' % (cmd_hz, freq, amp))
    for k, j in enumerate(ARM):
        n.go_home()
        cmds = []
        n.log.clear()
        t0 = n.now()
        last = -1
        while rclpy.ok() and n.now() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.002)
            step = int((n.now() - t0) * cmd_hz)
            if step != last:
                last = step
                ts = step / cmd_hz
                v = amp * math.sin(2 * math.pi * freq * ts)
                tgt = list(HOME)
                tgt[k] += v
                n.arm(tgt)
                cmds.append((ts, v))

        def cmd_at(t):
            c = 0.0
            for ts, v in cmds:
                if ts > t:
                    break
                c = v
            return c

        samples = [(t, d[j] - HOME[k]) for t, d in n.since(t0) if t > 1.0]
        # latency = time shift (0..120 ms) that best aligns the command with the response
        lag = min(range(0, 121, 2),
                  key=lambda ms: sum((q - cmd_at(t - ms / 1000)) ** 2 for t, q in samples))
        rms = math.sqrt(sum((q - cmd_at(t)) ** 2 for t, q in samples) / len(samples))
        print('  %-20s latency %3d ms   RMS %.2f deg' % (j, lag, math.degrees(rms)))
    n.go_home()


def test_step(n, size, joint):
    joints = ARM if joint == 'all' else [joint]
    n.go_home(2.0)
    tgt = [h + (size if j in joints else 0.0) for j, h in zip(ARM, HOME)]
    n.log.clear()
    t0 = n.now()
    n.arm(tgt)
    n.spin_for(1.0)
    print('STEP %+.3f rad on %s: q - home [rad] vs time' % (size, joint))
    print('  %7s ' % 't [s]' + ' '.join('%14s' % j[:14] for j in ARM))
    series = n.since(t0)
    for dt in (0.004, 0.012, 0.02, 0.032, 0.048, 0.064, 0.08, 0.1, 0.12, 0.16, 0.2, 0.3, 0.5, 0.9):
        t, d = min(series, key=lambda x: abs(x[0] - dt))
        print('  %7.3f ' % t + ' '.join('%+14.4f' % (d[j] - h) for j, h in zip(ARM, HOME)))
    for j, h in zip(ARM, HOME):
        if j in joints:
            peak = max((d[j] - h) / size for _, d in series)
            print('  %-20s overshoot %.1f %%' % (j, (peak - 1) * 100))
    n.go_home()


def test_gripper(n):
    """Isaac Sim test D: time from the command until the position stops changing (within 2 % of the final value)."""
    n.go_home(3.0)
    print('TEST D (gripper): Isaac Sim 2.15 s close / 2.15 s open, reaction 0.033 s, criterion 1.98 ~ 2.42 s')
    for raw in (RAW_MAX, 0.0):
        n.log.clear()
        n.grip_raw.clear()
        t0 = n.now()
        n.gripper(raw)
        n.spin_for(3.5)
        s = n.since(t0)
        q0, q1 = s[0][1]['rh_r1_joint'], s[-1][1]['rh_r1_joint']
        span = abs(q1 - q0)
        done = next((t for i, (t, _) in enumerate(s)
                     if all(abs(d['rh_r1_joint'] - q1) <= 0.02 * span for _, d in s[i:])), float('nan'))
        react = next((t for t, d in s if abs(d['rh_r1_joint'] - q0) > 0.001), float('nan'))
        mimic = max(abs(d[f] - d['rh_r1_joint']) for _, d in s for f in FINGERS[1:])
        raws = n.since(t0, n.grip_raw)
        print('  raw %6.0f: %.3f -> %.3f rad, done %.3f s, reaction %.3f s, max mimic error %.2f deg, '
              '/gripper/joint_states last %.1f'
              % (raw, q0, q1, done, react, math.degrees(mimic), raws[-1][1] if raws else float('nan')))
    n.go_home()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    names = ['hold', 'interp', 'a', 'c', 'step', 'gripper']
    p.add_argument('--tests', nargs='+', default=names, choices=names)
    p.add_argument('--step', type=float, default=0.2, help='step size for the "step" test [rad]')
    p.add_argument('--joint', default='all', help='joint for the "step" test, or "all"')
    a = p.parse_args()

    rclpy.init()
    n = DriveTest()
    # wait for sim time, states and for the interface node to subscribe to our commands
    # (a message published before the subscription is matched is lost)
    while rclpy.ok() and (not n.log or n.now() == 0.0 or n.arm_pub.get_subscription_count() == 0
                          or n.grip_pub.get_subscription_count() == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    for t in a.tests:
        {'hold': lambda: test_hold(n),
         'interp': lambda: test_interp(n),
         'a': lambda: test_a(n),
         'c': lambda: test_c(n),
         'step': lambda: test_step(n, a.step, a.joint),
         'gripper': lambda: test_gripper(n)}[t]()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
