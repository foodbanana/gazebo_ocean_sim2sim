#!/usr/bin/env python3
"""Drive tests for the arm and gripper in Gazebo (sim time). See docs/gain_tuning.md.

Run while `ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py` is up:

    ros2 run ocean_sim2sim_bringup drive_test.py                 # all tests
    ros2 run ocean_sim2sim_bringup drive_test.py --tests a c     # only test A and C
    ros2 run ocean_sim2sim_bringup drive_test.py --tests step --step 0.2 --joint shoulder_lift_joint

Tests (A and C are the same as the Isaac Sim drive tuning tests):
    hold    : hold the home pose for 10 s, max |q - home|
    a       : one joint at a time, 0.05 rad step -> overshoot, settling time (2 %), steady-state error
    c       : one joint at a time, 50 Hz staircase of a 0.5 Hz sine (0.3 rad) -> latency, RMS error
    step    : one step (--step rad) on --joint (or all joints) -> time series
    gripper : rh_r1_joint 0 -> 1.0 rad -> finger positions, max mimic error
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
FINGERS = ['rh_r1_joint', 'rh_r2', 'rh_l1', 'rh_l2']


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.log = []  # [(stamp [s], {joint: position})]
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 100)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def _on_joint_states(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.log.append((t, dict(zip(msg.name, msg.position))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_sec):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sim_sec:
            rclpy.spin_once(self, timeout_sec=0.002)

    def arm(self, q):
        self.arm_pub.publish(Float64MultiArray(data=[float(x) for x in q]))

    def gripper(self, q):
        self.grip_pub.publish(Float64MultiArray(data=[float(q)]))

    def go_home(self, sec=1.5):
        self.arm(HOME)
        self.gripper(0.0)
        self.spin_for(sec)

    def since(self, t0):
        return [(t - t0, d) for t, d in self.log if t >= t0]


def test_hold(n):
    n.go_home(2.0)
    n.log.clear()
    n.spin_for(10.0)
    err = [max(abs(d[j] - h) for _, d in n.log) for j, h in zip(ARM, HOME)]
    print('HOLD 10 s: max |q - home| [deg]')
    for j, e in zip(ARM, err):
        print('  %-20s %.4f' % (j, math.degrees(e)))


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
        # latency = time shift (0..100 ms) that best aligns the command with the response
        lag = min(range(0, 101, 2),
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


def test_gripper(n, target=1.0):
    n.go_home()
    n.log.clear()
    t0 = n.now()
    n.gripper(target)
    n.spin_for(3.0)
    series = n.since(t0)
    print('GRIPPER rh_r1_joint 0 -> %.2f rad' % target)
    for dt in (0.05, 0.1, 0.2, 0.5, 1.0, 2.9):
        t, d = min(series, key=lambda x: abs(x[0] - dt))
        print('  t=%4.2fs  ' % t + '  '.join('%s %.3f' % (f, d[f]) for f in FINGERS))
    mimic = max(abs(d[f] - d['rh_r1_joint']) for _, d in series for f in FINGERS[1:])
    print('  max mimic error |finger - rh_r1_joint| = %.2f deg' % math.degrees(mimic))
    n.go_home()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tests', nargs='+', default=['hold', 'a', 'c', 'step', 'gripper'],
                   choices=['hold', 'a', 'c', 'step', 'gripper'])
    p.add_argument('--step', type=float, default=0.2, help='step size for the "step" test [rad]')
    p.add_argument('--joint', default='all', help='joint for the "step" test, or "all"')
    a = p.parse_args()

    rclpy.init()
    n = DriveTest()
    while rclpy.ok() and (not n.log or n.now() == 0.0):
        rclpy.spin_once(n, timeout_sec=0.1)
    for t in a.tests:
        {'hold': lambda: test_hold(n),
         'a': lambda: test_a(n),
         'c': lambda: test_c(n),
         'step': lambda: test_step(n, a.step, a.joint),
         'gripper': lambda: test_gripper(n)}[t]()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
