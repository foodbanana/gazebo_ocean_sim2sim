"""Isaac Sim / real-robot ROS 2 interface on top of Gazebo ros2_control.

Same topics and behaviour as the Isaac Sim interface (data_collection_ur5_gripper
isaacsim/scripts/ros2_iface.py ArmBridge / GripperBridge, robot_drive.GripperProfile),
so a policy or teleop node cannot tell Isaac Sim, Gazebo and the real robot apart.

  /joint_command    sensor_msgs/JointState  arm joint targets [rad] (names, count, finite, joint limits are
                    checked: an invalid command is an error and stops this node -> the launch shuts down).
                    Each command is linearly interpolated over command_interp_time (33 ms) from the target
                    currently being interpolated (arm_controller: joint_trajectory_controller with
                    interpolate_from_desired_state).
  /gripper/command  std_msgs/Float64        raw 0 (open) ~ raw_max (closed). Outside the range = error.
                    When the goal changes the gripper moves at profile_velocity [rad/s] from the ACTUAL
                    finger position (real Dynamixel profile velocity).
                    Gazebo only: the joint target is kept command_margin [rad] inside the joint limits (both ends).
                    A joint driven exactly onto its limit gets stuck in DART and does not move away again
                    (measured: raw 1149 opens, raw 1150 = exact upper limit stays closed; opened exactly to 0 it
                    does not close). Real / Isaac Sim fingers stop on each other at about 1.121 rad, and 0.001 rad
                    open (0.05 mm at the pad) changes nothing.
  /gripper/joint_states, /gripper/target  sensor_msgs/JointState, name [gripper.name], position = raw.
                    present = rh_r1 [rad] * raw_max / upper (clamped), target = last goal raw.
                    Published once per loop tick with the tick stamp (below).

Loop (= Isaac Sim app loop, one render per loop, 30 Hz). On every loop tick T:
  1. /gripper/joint_states and /gripper/target with stamp T (finger position at T)
  2. if commands_on_loop: apply the last /joint_command and /gripper/command received since the last tick
     (Isaac Sim handles ROS commands once per loop: a command waits up to one loop and only the last one
     of a loop takes effect, a 60 Hz command stream is applied at 30 Hz). false = apply on arrival.
Loop ticks:
  loop_tick_topic set (camera_info of the wrist camera): every camera frame -> camera images and gripper
    states have the same stamps, as in Isaac Sim. Gazebo schedules camera frames on its own (frames come
    every 8 or sometimes 7 physics steps, about 30.3 Hz for update_rate 30), so the loop follows the camera.
  loop_tick_topic empty (no camera): every physics_hz / loop_hz physics steps (240 / 30 = 8). Counted in
    steps, not by sim time: Gazebo keeps time in integer ns (1/240 s -> 4166666 ns) and drifts off 1/30 s.

Hold (/ocean_sim2sim/hold, std_srvs/SetBool, used by scene_node during /sim/reset = Isaac Sim bridge.hold):
  true  -> commands are still checked but not applied (kept commands are dropped)
  false -> commands are applied again; the gripper goal is open (0), as the reset left the gripper open.

/joint_states (arm only, 120 Hz) is published by arm_state_broadcaster, /clock by the VRX bridge.
"""
import math
import xml.etree.ElementTree as ET
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CommandError(RuntimeError):
    """Invalid command from outside: stop loudly (no silent fallback)."""


def duration(sec):
    s = int(sec)
    return Duration(sec=s, nanosec=int(round((sec - s) * 1e9)))


class InterfaceNode(Node):
    def __init__(self):
        super().__init__('ocean_sim2sim_interface')
        p = self.declare_parameter
        self.arm_joints = list(p('arm.joints', rclpy.Parameter.Type.STRING_ARRAY).value or [])
        if not self.arm_joints:
            raise CommandError('parameter arm.joints is empty')
        self.interp_time = float(p('arm.command_interp_time', 0.0).value)
        self.grip_joint = str(p('gripper.joint', '').value)
        self.grip_name = str(p('gripper.name', '').value)
        self.raw_max = float(p('gripper.raw_max', 0.0).value)
        self.upper = float(p('gripper.upper', 0.0).value)
        self.profile_velocity = float(p('gripper.profile_velocity', 0.0).value)
        self.command_margin = float(p('gripper.command_margin', 0.0).value)
        self.physics_hz = float(p('physics_hz', 0.0).value)
        self.loop_hz = float(p('loop_hz', 0.0).value)
        self.loop_tick_topic = str(p('loop_tick_topic', '').value)
        self.commands_on_loop = bool(p('commands_on_loop', True).value)
        queue_sec = float(p('publish_queue_sec', 3.0).value)
        for name, v in (('arm.command_interp_time', self.interp_time), ('gripper.raw_max', self.raw_max),
                        ('gripper.upper', self.upper), ('gripper.profile_velocity', self.profile_velocity),
                        ('loop_hz', self.loop_hz), ('physics_hz', self.physics_hz)):
            if not v > 0:
                raise CommandError(f'parameter {name} must be > 0: {v}')
        self.every = self.physics_hz / self.loop_hz      # loop period in physics steps (no camera)
        if abs(self.every - round(self.every)) > 1e-9:
            raise CommandError(f'physics_hz {self.physics_hz} is not a multiple of loop_hz {self.loop_hz}')
        self.every = int(round(self.every))
        if not 0.0 <= self.command_margin < self.upper:
            raise CommandError(f'gripper.command_margin must be 0 ~ upper: {self.command_margin}')
        topic = lambda key, default: str(p(key, default).value)  # noqa: E731

        self.error = None
        self.limits = None            # {joint: (lower, upper)} from /robot_description
        self.q = {}                   # latest measured position of every joint
        self.goal_raw = 0.0           # real gripper node starts with goal 0 (open)
        self.goal_rad = 0.0
        self._last_step = None
        self.ticks = deque()          # loop ticks (stamp msgs) waiting for the finger state at that time
        self.grip_hist = deque(maxlen=int(2 * self.physics_hz))   # (stamp ns, rh_r1 [rad]), last 2 s
        self.pending_arm = None       # last checked /joint_command waiting for the next loop
        self.pending_grip = None      # last checked /gripper/command raw waiting for the next loop
        self.hold = False             # True during /sim/reset: commands are checked but ignored
        self.n_ignored = 0

        reliable = lambda depth: QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)  # noqa: E731
        grip_depth = max(10, int(queue_sec * self.loop_hz))
        self.pub_arm = self.create_publisher(JointTrajectory, topic('arm.trajectory_topic', '/arm_controller/joint_trajectory'), 10)
        self.pub_grip = self.create_publisher(JointTrajectory, topic('gripper.trajectory_topic', '/gripper_controller/joint_trajectory'), 10)
        self.pub_present = self.create_publisher(JointState, topic('gripper.joint_states_topic', '/gripper/joint_states'), reliable(grip_depth))
        self.pub_target = self.create_publisher(JointState, topic('gripper.target_topic', '/gripper/target'), reliable(grip_depth))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(JointState, topic('state_topic', '/joint_state_broadcaster/joint_states'), self._on_state, 100)
        self.create_subscription(JointState, topic('arm.joint_command_topic', '/joint_command'), self._guard(self._on_arm_command), 10)
        self.create_subscription(Float64, topic('gripper.command_topic', '/gripper/command'), self._guard(self._on_gripper_command), 10)
        self.create_service(SetBool, '/ocean_sim2sim/hold', self._on_hold)
        if self.loop_tick_topic:
            self.create_subscription(CameraInfo, self.loop_tick_topic, lambda m: self._tick(m.header.stamp), reliable(10))
        self.get_logger().info(
            f'arm {len(self.arm_joints)} joints, interp {self.interp_time * 1000:.1f} ms | '
            f'gripper {self.grip_joint} raw 0~{self.raw_max:g} <-> 0~{self.upper:.4f} rad, {self.profile_velocity} rad/s | '
            'loop ' + (f'= frames of {self.loop_tick_topic}' if self.loop_tick_topic else f'every {self.every} physics steps')
            + ', commands applied ' + ('on the loop' if self.commands_on_loop else 'on arrival'))

    # ---------------------------------------------------------------- inputs
    def _guard(self, fn):
        def cb(msg):
            if self.error is None:
                try:
                    fn(msg)
                except CommandError as e:
                    self.error = e
        return cb

    def _on_description(self, msg):
        root = ET.fromstring(msg.data)
        limits = {}
        for j in root.findall('joint'):
            lim = j.find('limit')
            if lim is not None and j.get('type') in ('revolute', 'prismatic'):
                limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        missing = [n for n in self.arm_joints if n not in limits]
        if missing:
            self.error = CommandError(f'no joint limits in robot_description for {missing}')
            return
        self.limits = limits

    def _on_state(self, msg):
        self.q.update(zip(msg.name, msg.position))
        if self.grip_joint in self.q:
            self.grip_hist.append((msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, self.q[self.grip_joint]))
        if not self.loop_tick_topic:
            step = round((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) * self.physics_hz)
            if step % self.every == 0 and step != self._last_step:
                self._last_step = step
                self.ticks.append(msg.header.stamp)
        self._run_loop()

    def _tick(self, stamp):
        self.ticks.append(stamp)
        self._run_loop()

    def _run_loop(self):
        """Handle the loop ticks whose finger state has arrived (state and tick come on different topics)."""
        while self.ticks and self.grip_hist:
            stamp = self.ticks[0]
            t = stamp.sec * 1_000_000_000 + stamp.nanosec
            if self.grip_hist[-1][0] < t:
                return                                   # state at t not received yet
            self.ticks.popleft()
            q = next((q for ts, q in reversed(self.grip_hist) if ts <= t), self.grip_hist[0][1])
            # 1. gripper states with the tick stamp
            present = min(max(q * self.raw_max / self.upper, 0.0), self.raw_max)
            for pub, val in ((self.pub_present, present), (self.pub_target, self.goal_raw)):
                m = JointState()
                m.header.stamp = stamp
                m.name = [self.grip_name]
                m.position = [float(val)]
                pub.publish(m)
            # 2. commands kept since the last tick (Isaac Sim: after the states, last one wins)
            if self.pending_arm is not None:
                q_arm, self.pending_arm = self.pending_arm, None
                self._apply_arm(q_arm)
            if self.pending_grip is not None:
                v, self.pending_grip = self.pending_grip, None
                self._apply_gripper(v)

    def _on_hold(self, req, res):
        self.hold = bool(req.data)
        self.pending_arm = self.pending_grip = None
        if not self.hold:
            self.goal_raw = self.goal_rad = 0.0          # the reset opened the gripper
        res.success = True
        res.message = f'hold {self.hold}, ignored commands so far {self.n_ignored}'
        return res

    def _on_arm_command(self, msg):
        names, pos = list(msg.name), list(msg.position)
        if len(names) != len(pos):
            raise CommandError(f'/joint_command has {len(names)} names but {len(pos)} positions')
        if sorted(names) != sorted(self.arm_joints) or len(set(names)) != len(names):
            raise CommandError(f'/joint_command joint names {names} != arm joints {self.arm_joints}')
        by = dict(zip(names, pos))
        q = [float(by[n]) for n in self.arm_joints]
        if not all(math.isfinite(v) for v in q):
            raise CommandError(f'/joint_command is not finite: {q}')
        if self.limits is None:
            raise CommandError('/joint_command before /robot_description (joint limits unknown)')
        out = [f'{n} {v:+.4f} (limit {self.limits[n][0]:+.4f} ~ {self.limits[n][1]:+.4f})'
               for n, v in zip(self.arm_joints, q) if not self.limits[n][0] <= v <= self.limits[n][1]]
        if out:
            raise CommandError('/joint_command outside the joint limits: ' + ', '.join(out))
        if self.hold:
            self.n_ignored += 1               # reset in progress: checked but not applied
        elif self.commands_on_loop:
            self.pending_arm = q              # only the last command of a loop takes effect
        else:
            self._apply_arm(q)

    def _apply_arm(self, q):
        traj = JointTrajectory()          # stamp 0 = start now
        traj.joint_names = self.arm_joints
        traj.points = [JointTrajectoryPoint(positions=q, time_from_start=duration(self.interp_time))]
        self.pub_arm.publish(traj)

    def _on_gripper_command(self, msg):
        v = float(msg.data)
        if not math.isfinite(v) or not 0.0 <= v <= self.raw_max:
            raise CommandError(f'/gripper/command must be 0 ~ {self.raw_max:g} (raw): {v}')
        if self.grip_joint not in self.q:
            raise CommandError(f'/gripper/command before the first state of {self.grip_joint}')
        if self.hold:
            self.n_ignored += 1
        elif self.commands_on_loop:
            self.pending_grip = v
        else:
            self._apply_gripper(v)

    def _apply_gripper(self, v):
        self.goal_raw = v
        goal = min(max(v * self.upper / self.raw_max, self.command_margin), self.upper - self.command_margin)
        if goal == self.goal_rad:
            return                        # same goal repeated: keep moving as before (GripperProfile.set_goal)
        self.goal_rad = goal
        start = self.q[self.grip_joint]   # new motion starts at the ACTUAL finger position
        t_end = max(abs(goal - start) / self.profile_velocity, 1e-3)
        traj = JointTrajectory()
        traj.joint_names = [self.grip_joint]
        traj.points = [JointTrajectoryPoint(positions=[start], time_from_start=duration(0.0)),
                       JointTrajectoryPoint(positions=[goal], time_from_start=duration(t_end))]
        self.pub_grip.publish(traj)


def main():
    rclpy.init()
    node = InterfaceNode()
    try:
        while rclpy.ok() and node.error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    err = node.error
    if err is not None:
        node.get_logger().fatal(f'{err}  -> stopping (invalid command, no silent fallback)')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if err is not None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
