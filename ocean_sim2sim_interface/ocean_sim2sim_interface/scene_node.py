"""Episode scene in Gazebo: the drone to grasp (Isaac Sim drone scene services).

  - After the boat has settled (spawn_delay sim seconds) spawn
      drone_anchor : model welded to the world, no collision (the point the drone is held at)
      simple_drone : the drone (ocean_sim2sim_gazebo/models/simple_drone), held on the anchor by a
                     DetachableJoint = world-fixed hover (stand-in until the PX4 SITL drone)
    at  robot base position + Rz(base yaw) * drone_pos_base, level (yaw = base yaw).
    The robot base pose comes from the boat model pose (Gazebo OdometryPublisher on the boat) and the fixed joints
    boat_link -> robot_base_link in /robot_description, AVERAGED over base_average seconds (about one wave
    period): the drone stays fixed in the world while the boat moves on the waves, so it is placed relative
    to where the base is on average, not where a wave happened to put it (one instant was 3 cm too high).
  - /sim/drone_kill (std_srvs/Trigger): "motors off" = detach from the anchor, the drone falls
    (onto the gripper if it is held). Response message = JSON. Already off -> success False.
"""
import json
import math
import os
from collections import deque
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
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
        default_sdf = os.path.join(get_package_share_directory('ocean_sim2sim_gazebo'), 'models', 'simple_drone', 'model.sdf')
        self.drone_sdf_path = str(p('drone_sdf', default_sdf).value)
        with open(self.drone_sdf_path) as f:
            self.drone_sdf = f.read()

        self.cb = ReentrantCallbackGroup()
        self.base_hist = deque(maxlen=5000)   # (sim t, robot base position, base yaw) from the boat odometry
        self.base_offset = None        # (R, t) boat_link -> robot base
        self.attached = None           # from /drone/state
        self.spawned = False
        self.start_time = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(Odometry, '/sim/boat_odometry', self._on_boat_odometry, 10)
        self.create_subscription(String, '/drone/state', self._on_drone_state, 10)
        self.pub_detach = self.create_publisher(Empty, '/drone/detach', 10)
        self.pub_attach = self.create_publisher(Empty, '/drone/attach', 10)
        self.spawn_cli = self.create_client(SpawnEntity, f'/world/{self.world}/create', callback_group=self.cb)
        self.create_service(Trigger, '/sim/drone_kill', self._on_kill, callback_group=self.cb)
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
        self.spawned = True
        self.timer.cancel()
        pos, yaw, base = self.drone_world_pose(self.drone_pos_base)
        self._spawn('drone_anchor', ANCHOR_SDF, pos, yaw)
        self._spawn('simple_drone', self.drone_sdf, pos, yaw)
        self.get_logger().info(
            f'drone at world ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) yaw {math.degrees(yaw):.1f} deg = '
            f'mean robot base over {self.base_average:g} s ({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}) '
            f'+ {self.drone_pos_base} (base frame, level)')

    def _on_kill(self, req, res):
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.spawned or self.attached is False:
            res.success = False
            res.message = json.dumps({'sim_time': t, 'error': 'drone is not held (not spawned or already off)'})
            return res
        self.pub_detach.publish(Empty())
        self.attached = False
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
