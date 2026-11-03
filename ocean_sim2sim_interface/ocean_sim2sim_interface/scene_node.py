"""Episode scene in Gazebo: the drone to grasp (Isaac Sim drone scene services).

  - After the boat has settled (spawn_delay sim seconds) spawn
      drone_anchor : model welded to the world, no collision (the point the drone is held at)
      drone        : the drone (ocean_sim2sim_gazebo/models/iris_075, Iris 0.75x as in Isaac Sim), held on the anchor by a
                     DetachableJoint = world-fixed hover (stand-in until the PX4 SITL drone)
    at  robot base position + Rz(base yaw) * drone_pos_base, level (yaw = base yaw).
    The robot base pose comes from the boat model pose (Gazebo OdometryPublisher on the boat) and the fixed joints
    boat_link -> robot_base_link in /robot_description, AVERAGED over base_average seconds (about one wave
    period): the drone stays fixed in the world while the boat moves on the waves, so it is placed relative
    to where the base is on average, not where a wave happened to put it (one instant was 3 cm too high).
  - /sim/drone_kill (std_srvs/Trigger): "motors off" = detach from the anchor, the drone falls
    (onto the gripper if it is held). Response message = JSON. Already off -> success False.
  - /sim/reset (std_srvs/Trigger): back to the episode start (Isaac Sim sim_reset.EpisodeReset):
      1. interface hold on (commands are checked but ignored)
      2. remove the drone and its anchor (also a drone held in the gripper)
      3. arm to init_pose and gripper open, smoothly in reset_arm_time (Gazebo has no joint teleport from ROS)
      4. new hover position: next point of drone_targets_base (shuffled, one full round at a time)
         + uniform +-drone_offset per axis, from seed + reset count (same seed = same sequence);
         use_drone_targets false -> drone_pos_base + offset
      5. spawn anchor + drone there (mean robot base, as at the start), wait for "attached"
      6. hold off (gripper goal = open)
    Response message = JSON (seed, target, offsets, positions, sim seconds per phase, wall time).
    On failure success False and the hold stays on (commands ignored) until a reset succeeds.
"""
import json
import math
import os
import random
import time
from collections import deque
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry

# Anchor = a non-static model welded to the world (as VRX's dock "platform"). The DetachableJoint does not
# attach to a <static> model (tested: the drone was not held).
ANCHOR_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="drone_anchor"><static>false</static>
  <link name="link"/>
  <joint name="world_joint" type="fixed"><parent>world</parent><child>link</child></joint>
</model></sdf>"""


# ---------------------------------------------------------------- small 3D helpers (no numpy needed)
def quat_to_mat(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_vec(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def fixed_chain(urdf_xml, parent, child):
    """Transform parent -> child through fixed joints of a URDF: (R, t)."""
    root = ET.fromstring(urdf_xml)
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    R, t = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    link = child
    while link != parent:
        j = by_child.get(link)
        if j is None:
            raise RuntimeError(f'{child} is not below {parent} in robot_description')
        if j.get('type') != 'fixed':
            raise RuntimeError(f'joint {j.get("name")} between {parent} and {child} is not fixed')
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        Rj = rpy_to_mat(*rpy)
        t = [a + b for a, b in zip(mat_vec(Rj, t), xyz)]        # T = Tj * T
        R = mat_mul(Rj, R)
        link = j.find('parent').get('link')
    return R, t


class SceneNode(Node):
    def __init__(self):
        super().__init__('ocean_sim2sim_scene')
        p = self.declare_parameter
        self.world = str(p('world', 'empty_ocean').value)
        self.boat_model = str(p('boat_model', 'wamv').value)
        self.boat_link = str(p('boat_link', 'wamv/base_link').value)
        self.base_link = str(p('robot_base_link', 'robot_mount').value)
        self.drone_pos_base = [float(v) for v in p('drone_pos_base', [0.60, 0.0, 0.738]).value]
        self.spawn_delay = float(p('spawn_delay', 3.0).value)
        self.base_average = float(p('base_average', 5.0).value)
        default_sdf = os.path.join(get_package_share_directory('ocean_sim2sim_gazebo'), 'models', 'iris_075', 'model.sdf')
        self.drone_sdf_path = str(p('drone_sdf', default_sdf).value)
        with open(self.drone_sdf_path) as f:
            self.drone_sdf = f.read()
        # reset
        self.arm_joints = list(p('arm_joints', rclpy.Parameter.Type.STRING_ARRAY).value)
        self.init_pose = [float(v) for v in p('init_pose', rclpy.Parameter.Type.DOUBLE_ARRAY).value]
        if len(self.init_pose) != len(self.arm_joints):
            raise RuntimeError(f'init_pose has {len(self.init_pose)} values for {len(self.arm_joints)} arm joints')
        self.gripper_joint = str(p('gripper_joint', 'rh_r1_joint').value)
        self.reset_arm_time = float(p('reset_arm_time', 2.0).value)
        # After a protective stop the arm may be pressed against the boat: moving straight to init_pose can drag it
        # along the deck and stop short (Isaac Sim teleports the arm, Gazebo cannot). Then go via this pose first.
        self.escape_pose = [float(v) for v in p('reset_escape_pose', [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]).value]
        self.reset_arrive_tol = float(p('reset_arrive_tol', 0.02).value)     # [rad] arm must reach init_pose
        # open = this far inside the lower limit: a joint driven exactly onto its limit gets stuck in DART (interface
        # gripper.command_margin)
        self.gripper_open = float(p('gripper_open', 0.001).value)
        self.use_targets = bool(p('use_drone_targets', True).value)
        flat = [float(v) for v in p('drone_targets_base', rclpy.Parameter.Type.DOUBLE_ARRAY).value]
        names = list(p('drone_target_names', rclpy.Parameter.Type.STRING_ARRAY).value)
        if len(flat) != 3 * len(names) or not names:
            raise RuntimeError(f'drone_targets_base needs 3 values per name: {len(flat)} values, {len(names)} names')
        self.targets = [(n, flat[3 * i:3 * i + 3]) for i, n in enumerate(names)]
        self.drone_offset = float(p('drone_offset', 0.03).value)
        # drone: fixed = held on the world anchor (no flight); px4 = PX4 SITL flies it (ocean_sim2sim_drone px4_bridge
        # node). PX4 air start (docs/drone_px4.md): anchored -> PX4 restart (+ EKF wait) -> arm (Offboard) -> target
        # lifted by px4_release_lift -> release the anchor when the rotor thrust reaches px4_release_thrust -> target
        # back to the hover point -> settle_time -> drone motion (static / hover sine, Isaac Sim presets)
        self.drone_mode = str(p('drone_mode', 'fixed').value)
        if self.drone_mode not in ('fixed', 'px4'):
            raise RuntimeError(f'drone_mode must be fixed or px4: {self.drone_mode}')
        self.px4_lift = float(p('px4_release_lift', 0.10).value)
        self.px4_release_thrust = float(p('px4_release_thrust', 0.8 * 9.81).value)
        self.px4_release_timeout = float(p('px4_release_timeout', 20.0).value)
        self.settle_time = float(p('settle_time', 5.0).value)
        self.drone_motion = str(p('drone_motion', 'static').value)
        self.hover_preset = str(p('hover_preset', 'mid').value)
        self.px4_state = None
        seed = int(p('seed', -1).value)
        self.seed = seed if seed >= 0 else int(time.time())   # -1: start time (different every session, as Isaac Sim)
        self.reset_count = 0
        self.order = []               # target indices left in this round
        self.round = 0
        self.busy = False

        self.cb = ReentrantCallbackGroup()
        self.base_hist = deque(maxlen=5000)   # (sim t, robot base position, base yaw) from the boat odometry
        self.base_offset = None        # (R, t) boat_link -> robot base
        self.attached = None           # from /drone/state
        self.spawned = False
        self.start_time = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        # arm state for the reset: protective stop (go via the escape pose first) and arrival check
        self.q_arm, self.protective_stop = {}, False
        self.create_subscription(JointState, '/joint_states', lambda m: self.q_arm.update(zip(m.name, m.position)), 10,
                                 callback_group=self.cb)
        self.create_subscription(Bool, '/protective_stop', lambda m: setattr(self, 'protective_stop', m.data), 10,
                                 callback_group=self.cb)
        self.create_subscription(Odometry, '/sim/boat_odometry', self._on_boat_odometry, 10)
        self.create_subscription(String, '/drone/state', self._on_drone_state, 10)
        self.pub_detach = self.create_publisher(Empty, '/drone/detach', 10)
        self.pub_attach = self.create_publisher(Empty, '/drone/attach', 10)
        self.spawn_cli = self.create_client(SpawnEntity, f'/world/{self.world}/create', callback_group=self.cb)
        self.delete_cli = self.create_client(DeleteEntity, f'/world/{self.world}/remove', callback_group=self.cb)
        self.hold_cli = self.create_client(SetBool, '/ocean_sim2sim/hold', callback_group=self.cb)
        self.pub_arm = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_grip = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_service(Trigger, '/sim/drone_kill', self._on_kill, callback_group=self.cb)
        if self.drone_mode == 'px4':
            self.create_subscription(String, '/drone/px4/state', self._on_px4_state, 10, callback_group=self.cb)
            self.px4_cli = {n: self.create_client(Trigger, f'/drone/px4/{n}', callback_group=self.cb)
                            for n in ('restart', 'arm', 'kill')}
            self.pub_ref = self.create_publisher(String, '/drone/px4/reference_cmd', 10)
        self.create_service(Trigger, '/sim/reset', self._on_reset, callback_group=self.cb)
        self.get_logger().info(f'seed {self.seed}, {len(self.targets)} drone targets'
                               + ('' if self.use_targets else ' (not used: drone_pos_base)')
                               + f', offset +-{self.drone_offset} m, init_pose {self.init_pose}')
        self.timer = self.create_timer(0.5, self._spawn_when_ready, callback_group=self.cb)

    # ---------------------------------------------------------------- inputs
    def _on_description(self, msg):
        self.base_offset = fixed_chain(msg.data, self.boat_link, self.base_link)

    def _on_boat_odometry(self, msg):
        if self.base_offset is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        R_boat = quat_to_mat([q.x, q.y, q.z, q.w])
        R_off, t_off = self.base_offset
        base = [a + b for a, b in zip([p.x, p.y, p.z], mat_vec(R_boat, t_off))]
        R_base = mat_mul(R_boat, R_off)
        yaw = math.atan2(R_base[1][0], R_base[0][0])        # level frame: only the base heading
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.base_hist.append((t, base, yaw))

    def _on_drone_state(self, msg):
        self.attached = (msg.data == 'attached')

    def _on_px4_state(self, msg):
        self.px4_state = json.loads(msg.data)

    # ---------------------------------------------------------------- PX4 drone
    def _px4_call(self, name, timeout=180.0):
        cli = self.px4_cli[name]
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'/drone/px4/{name} not available (px4_bridge node, drone_mode px4)')
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise RuntimeError(f'/drone/px4/{name}: no answer in {timeout:g} s')
            time.sleep(0.01)
        res = fut.result()
        if not res.success:
            raise RuntimeError(f'/drone/px4/{name} failed: {res.message[:300]}')
        return json.loads(res.message) if res.message.startswith('{') else res.message

    def _px4_reference(self, mode, p0, preset=None, seed=None):
        cmd = {'mode': mode, 'p0': [float(v) for v in p0]}
        if preset is not None:
            cmd['preset'] = preset
        if seed is not None:
            cmd['seed'] = int(seed)
        self.pub_ref.publish(String(data=json.dumps(cmd)))
        return cmd

    def _px4_start(self, pos, motion_seed, phases):
        """Air start of the anchored drone at pos (world). Returns info for the response."""
        t = self.now()
        r = self._px4_call('restart')
        phases['px4_restart'], t = self.now() - t, self.now()
        self._px4_call('arm')
        phases['px4_arm'], t = self.now() - t, self.now()
        p = list(pos)
        self._px4_reference('static', [p[0], p[1], p[2] + self.px4_lift])
        if not self.wait_sim(self.px4_release_timeout,
                             until=lambda: (self.px4_state or {}).get('thrust_sum', 0.0) >= self.px4_release_thrust):
            raise RuntimeError(f'PX4 thrust did not reach {self.px4_release_thrust:.2f} N in {self.px4_release_timeout:g} s '
                               f'(state {self.px4_state and self.px4_state.get("thrust_sum")})')
        thrust = self.px4_state['thrust_sum']
        self.pub_detach.publish(Empty())
        self._px4_reference('static', p)
        if not self.wait_sim(2.0, until=lambda: self.attached is False):
            raise RuntimeError('drone did not report "detached" after the release')
        phases['px4_release'], t = self.now() - t, self.now()
        self.wait_sim(self.settle_time)
        phases['settle'], t = self.now() - t, self.now()
        st = self.px4_state or {}
        err = [round(a - b, 4) for a, b in zip(st.get('true_position', p), p)]
        if self.drone_motion == 'hover':
            ref = self._px4_reference('hover', p, self.hover_preset, motion_seed)
        else:
            ref = self._px4_reference('static', p)
        return {'px4_run_dir': os.path.basename(r.get('run_dir', '')) if isinstance(r, dict) else None,
                'release_thrust_n': round(thrust, 3), 'settle_error': err, 'drone_ref': ref}

    # ---------------------------------------------------------------- scene
    def mean_base(self):
        """Robot base position and yaw averaged over the last base_average seconds."""
        t_end = self.base_hist[-1][0]
        win = [h for h in self.base_hist if h[0] >= t_end - self.base_average]
        base = [sum(h[1][k] for h in win) / len(win) for k in range(3)]
        yaw = math.atan2(sum(math.sin(h[2]) for h in win), sum(math.cos(h[2]) for h in win))
        return base, yaw

    def drone_world_pose(self, pos_base):
        """World pose for a drone position given in the (level, averaged) robot base frame."""
        base, yaw = self.mean_base()
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = pos_base
        return [base[0] + c * x - s * y, base[1] + s * x + c * y, base[2] + z], yaw, base

    def _spawn(self, name, sdf, pos, yaw):
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.allow_renaming = False
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req.entity_factory.pose = pose
        res = self.spawn_cli.call(req)
        if not res.success:
            raise RuntimeError(f'spawning {name} failed')

    def _spawn_when_ready(self):
        if self.spawned or not self.base_hist:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        # let the boat settle, then average the base over base_average seconds
        if now - self.start_time < self.spawn_delay + self.base_average or not self.spawn_cli.service_is_ready():
            return
        if self.busy:
            return
        self.busy = True                  # a /sim/reset now would race with this spawn: it answers "not ready"
        self.timer.cancel()
        try:
            pos, yaw, base = self.drone_world_pose(self.drone_pos_base)
            self._spawn('drone_anchor', ANCHOR_SDF, pos, yaw)
            self._spawn('drone', self.drone_sdf, pos, yaw)
            self.wait_sim(2.0, until=lambda: self.attached is True)
            if self.drone_mode == 'px4':
                ph = {}
                px4 = self._px4_start(pos, self.seed + 1_000_000, ph)
                self.get_logger().info(f'PX4 drone flying: {px4}, phases {ph}')
            self.spawned = True           # only now: /sim/reset and /sim/drone_kill are accepted
        finally:
            self.busy = False
        self.get_logger().info(
            f'drone at world ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) yaw {math.degrees(yaw):.1f} deg = '
            f'mean robot base over {self.base_average:g} s ({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}) '
            f'+ {self.drone_pos_base} (base frame, level)')

    # ---------------------------------------------------------------- reset
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, sec, until=None):
        """Wait sec sim seconds (or until until() is true). Returns True if until() became true."""
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sec:
            if until is not None and until():
                return True
            time.sleep(0.005)
        return until is not None and until()

    def _hold(self, on):
        if not self.hold_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/ocean_sim2sim/hold not available (interface node)')
        if not self.hold_cli.call(SetBool.Request(data=on)).success:
            raise RuntimeError(f'interface hold {on} failed')

    def _delete(self, name):
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        return self.delete_cli.call(req).success     # False if it does not exist

    def _move_joints(self, pub, joints, q, sec):
        traj = JointTrajectory()          # stamp 0 = start now, from the current desired state
        traj.joint_names = joints
        s = int(sec)
        traj.points = [JointTrajectoryPoint(positions=[float(v) for v in q],
                                            time_from_start=Duration(sec=s, nanosec=int((sec - s) * 1e9)))]
        pub.publish(traj)

    def next_target(self, k):
        """Drone position (base frame) for reset number k: (name, target, offset)."""
        # read now: the grasp demo switches these at run time (ros2 param set), e.g. targets off + offset 0 = drone_pos_base
        self.use_targets = bool(self.get_parameter('use_drone_targets').value)
        self.drone_offset = float(self.get_parameter('drone_offset').value)
        self.drone_pos_base = [float(v) for v in self.get_parameter('drone_pos_base').value]
        if self.use_targets:
            if not self.order:
                self.order = list(range(len(self.targets)))
                random.Random(self.seed * 1000 + self.round).shuffle(self.order)
                self.round += 1
            name, target = self.targets[self.order.pop(0)]
        else:
            name, target = 'drone_pos_base', list(self.drone_pos_base)
        rng = random.Random(self.seed + k)
        offset = [rng.uniform(-self.drone_offset, self.drone_offset) for _ in range(3)]
        return name, target, offset

    def _on_reset(self, req, res):
        if self.busy or not self.spawned:
            res.success = False
            res.message = json.dumps({'error': 'scene not ready yet (drone not spawned)' if not self.spawned
                                      else 'reset already running'})
            return res
        self.busy = True
        t0, w0 = self.now(), time.time()
        phases, t = {}, t0
        k = self.reset_count
        try:
            was_stopped = self.protective_stop     # read before the hold: the hold releases the protective stop
            self._hold(True)
            # 2. remove the drone (and anchor). Detach FIRST and wait until Gazebo has done it: removing the
            #    drone while its DetachableJoint still holds it crashed Gazebo (DART WeldJointConstraint on a
            #    deleted body, segfault).
            if self.drone_mode == 'px4' and (self.px4_state or {}).get('armed'):
                self._px4_call('kill')         # motors off first (Isaac Sim: kill -> teleport -> PX4 restart)
                if not self.wait_sim(2.0, until=lambda: not (self.px4_state or {}).get('armed')):
                    raise RuntimeError('PX4 still armed 2 s after kill')
            if self.attached is not False:
                self.pub_detach.publish(Empty())
                if not self.wait_sim(2.0, until=lambda: self.attached is False):
                    raise RuntimeError('drone did not report "detached" within 2 s')
            self.wait_sim(0.05)
            self._delete('drone')
            self.wait_sim(0.05)
            self._delete('drone_anchor')
            self.wait_sim(0.05)
            self.attached = None
            phases['remove_drone'], t = self.now() - t, self.now()
            # 3. arm to the start pose, gripper open
            if was_stopped:
                self._move_joints(self.pub_arm, self.arm_joints, self.escape_pose, self.reset_arm_time)
                self.wait_sim(self.reset_arm_time + 0.3)
            self._move_joints(self.pub_arm, self.arm_joints, self.init_pose, self.reset_arm_time)
            self._move_joints(self.pub_grip, [self.gripper_joint], [self.gripper_open], self.reset_arm_time)
            self.wait_sim(self.reset_arm_time + 0.3)
            off = {j: round(self.q_arm[j] - q, 4) for j, q in zip(self.arm_joints, self.init_pose)
                   if j in self.q_arm and abs(self.q_arm[j] - q) > self.reset_arrive_tol}
            if off or not self.q_arm:
                raise RuntimeError(f'arm did not reach init_pose (rad off: {off or "no /joint_states"})')
            phases['arm_to_start'], t = self.now() - t, self.now()
            # 4. new hover position, 5. spawn
            name, target, offset = self.next_target(k)
            pos_base = [a + b for a, b in zip(target, offset)]
            pos, yaw, base = self.drone_world_pose(pos_base)
            self._spawn('drone_anchor', ANCHOR_SDF, pos, yaw)
            self._spawn('drone', self.drone_sdf, pos, yaw)
            attached = self.wait_sim(2.0, until=lambda: self.attached is True)
            phases['spawn_drone'], t = self.now() - t, self.now()
            px4 = None
            if self.drone_mode == 'px4':
                # hover sine seed: reset seed + 1,000,000 (Isaac Sim sim_reset)
                px4 = self._px4_start(pos, self.seed + k + 1_000_000, phases)
                t = self.now()
            # 6. commands again
            self._hold(False)
            self.reset_count += 1
            info = {'reset_count': self.reset_count, 'seed': self.seed, 'offset_seed': self.seed + k,
                    'drone_target': name, 'drone_target_base': target,
                    'drone_offset': [round(v, 5) for v in offset],
                    'drone_pos_base': [round(v, 5) for v in pos_base],
                    'drone_world': [round(v, 5) for v in pos], 'drone_yaw': round(yaw, 5),
                    'mean_robot_base': [round(v, 5) for v in base],
                    'init_pose': self.init_pose, 'drone_attached': attached,
                    'phases': {k2: round(v, 3) for k2, v in phases.items()},
                    'drone_mode': self.drone_mode, 'px4': px4,
                    'sim_time': round(self.now(), 4), 'sim_s': round(self.now() - t0, 3), 'wall_s': round(time.time() - w0, 3)}
            res.success = True
            res.message = json.dumps(info)
            self.get_logger().info(f'reset {self.reset_count}: drone {name} + {info["drone_offset"]} -> '
                                   f'base {info["drone_pos_base"]}, {info["sim_s"]} s sim')
        except Exception as e:  # noqa: BLE001  report, keep the hold on (no commands until a reset succeeds)
            res.success = False
            res.message = json.dumps({'error': str(e), 'phases': phases})
            self.get_logger().error(f'reset failed: {e}')
        finally:
            self.busy = False
        return res

    def _on_kill(self, req, res):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.drone_mode == 'px4':
            try:
                info = self._px4_call('kill', timeout=10.0)
                res.success = True
                res.message = json.dumps({'sim_time': t, 'px4': 'kill', 'hover_freeze': (info or {}).get('reference', {})
                                          .get('freeze') if isinstance(info, dict) else None})
                self.get_logger().info(f'drone motors off (PX4 kill) at sim time {t:.3f} s')
            except Exception as e:  # noqa: BLE001
                res.success = False
                res.message = json.dumps({'sim_time': t, 'error': str(e)})
            return res
        if not self.spawned or self.attached is False:
            res.success = False
            res.message = json.dumps({'sim_time': t, 'error': 'drone is not held (not spawned or already off)'})
            return res
        self.pub_detach.publish(Empty())
        self.wait_sim(1.0, until=lambda: self.attached is False)
        res.success = True
        res.message = json.dumps({'sim_time': t})
        self.get_logger().info(f'drone motors off at sim time {t:.3f} s')
        return res


def main():
    rclpy.init()
    node = SceneNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001  the executor may fail on a context that the shutdown signal already closed
        if rclpy.ok():
            raise
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
