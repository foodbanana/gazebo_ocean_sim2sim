#!/usr/bin/env python3
"""Gripper force / friction tests in Gazebo, after the Isaac Sim grasp tests
(data_collection_ur5_gripper isaacsim/scripts/grasp_tests.py, docs/PLAN.md 1-6). See docs/grasp.md.

Run with the calm test world (no waves, ApplyLinkWrench):
    ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm
    ros2 run ocean_sim2sim_bringup grasp_test.py --tests E G3

Tests (arm in the home pose, tool pointing down):
    E   empty close: close the gripper with nothing between the fingers -> where it stops, finger joint torques
    G3  pull: a box 60 (grasp direction) x 160 x 100 mm is held between the fingers by a temporary anchor, the gripper
        closes (squeeze torque = sum of the 4 finger joint torques, contact normal force if Gazebo reports it), the anchor
        lets go, then a force along the tool axis out of the fingers (down) grows at --rate N/s until the box slips 1 mm
        -> slip force = pulling force + box weight (Isaac Sim: 19.1 N, 0.8 kg box, maxForce 2.28 Nm, friction 0.6/0.5)
    G1  small box 40 mm, 0.2 kg: hold 3 s -> slip, jitter
    G4  box 0.5 kg: hold 5 s -> finger / arm joint jitter, box jitter

Isaac Sim virtual work: sum of finger normal forces x 50.8 mm/rad = transmitted joint torque (2.28 Nm -> 44.9 N, ~22 N per finger).
"""
import argparse
import math
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts, Entity, EntityWrench
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from tf2_ros import Buffer, TransformListener

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
FINGERS = ['rh_r1_joint', 'rh_r2', 'rh_l1', 'rh_l2']
FINGER_LINKS = ['rh_p12_rn_r1', 'rh_p12_rn_r2', 'rh_p12_rn_l1', 'rh_p12_rn_l2']
PAD_RATIO = 0.0508   # [m/rad] Isaac Sim: finger pad travel per rad of rh_r1 (virtual work)

ANCHOR_SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="test_box_anchor"><static>false</static>
<link name="link"/><joint name="world_joint" type="fixed"><parent>world</parent><child>link</child></joint></model></sdf>"""


BOX_MESH = None     # --box-mesh: use this STL (same size) as the box collision instead of a <box> (mesh-vs-mesh check)


def box_sdf(size, mass, mu):
    sx, sy, sz = size
    coll_geom = (f'<mesh><uri>file://{BOX_MESH}</uri></mesh>' if BOX_MESH else f'<box><size>{sx} {sy} {sz}</size></box>')
    ixx, iyy, izz = mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12, mass * (sx * sx + sy * sy) / 12
    return f"""<?xml version="1.0"?><sdf version="1.9"><model name="test_box">
<link name="link">
  <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
  <collision name="box"><geometry>{coll_geom}</geometry>
    <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>
  <visual name="box"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
    <material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material></visual>
  <sensor name="contacts" type="contact"><always_on>true</always_on><update_rate>240</update_rate>
    <topic>/test_box/contacts</topic><contact><collision>box</collision><topic>/test_box/contacts</topic></contact></sensor>
</link>
<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
  <parent_link>link</parent_link><child_model>test_box_anchor</child_model><child_link>link</child_link>
  <attach_topic>/test_box/attach</attach_topic><detach_topic>/test_box/detach</detach_topic><output_topic>/test_box/state</output_topic>
</plugin>
<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
  <odom_frame>world</odom_frame><robot_base_frame>test_box</robot_base_frame><odom_publish_frequency>240</odom_publish_frequency><dimensions>3</dimensions>
</plugin>
</model></sdf>"""


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def quat_rot(q, v):
    x, y, z, w = q
    p = quat_mul(quat_mul(q, [v[0], v[1], v[2], 0.0]), [-x, -y, -z, w])
    return p[:3]


class GraspTest(Node):
    def __init__(self, world):
        super().__init__('grasp_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.world = world
        self.q, self.eff = {}, {}
        self.boat = None
        self.box = None                  # (t, position, quaternion)
        self.box_log = []
        self.box_state = None
        self.contacts = None
        self.debug = False
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(JointState, '/joint_state_broadcaster/joint_states', self._on_js, 50)
        self.create_subscription(Odometry, '/sim/boat_odometry', lambda m: setattr(self, 'boat', m.pose.pose), 10)
        self.create_subscription(Odometry, '/model/test_box/odometry', self._on_box, 100)
        self.create_subscription(String, '/test_box/state', lambda m: setattr(self, 'box_state', m.data), 10)
        self.create_subscription(Contacts, '/test_box/contacts', lambda m: setattr(self, 'contacts', m), 10)
        self.arm_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.grip_pub = self.create_publisher(Float64, '/gripper/command', 10)
        self.detach_pub = self.create_publisher(Empty, '/test_box/detach', 10)
        self.wrench_pub = self.create_publisher(EntityWrench, f'/world/{world}/wrench/persistent', 10)
        self.clear_pub = self.create_publisher(Entity, f'/world/{world}/wrench/clear', 10)
        self.spawn_cli = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.delete_cli = self.create_client(DeleteEntity, f'/world/{world}/remove')

    def _on_js(self, m):
        self.q.update(zip(m.name, m.position))
        self.eff.update(zip(m.name, m.effort))

    def _on_box(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.box = (t, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        self.box_log.append(self.box)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, sec):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sec:
            time.sleep(0.002)

    def arm(self, q):
        m = JointState()
        m.name, m.position = ARM, [float(v) for v in q]
        self.arm_pub.publish(m)

    def gripper(self, raw):
        self.grip_pub.publish(Float64(data=float(raw)))

    def tcp_world(self):
        """TCP pose in the world: boat odometry x TF wamv/base_link -> rh_p12_rn_tcp."""
        tr = self.tf.lookup_transform('wamv/base_link', 'rh_p12_rn_tcp', rclpy.time.Time())
        t, r = tr.transform.translation, tr.transform.rotation
        b = self.boat
        qb = [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w]
        p = [a + c for a, c in zip([b.position.x, b.position.y, b.position.z], quat_rot(qb, [t.x, t.y, t.z]))]
        return p, quat_mul(qb, [r.x, r.y, r.z, r.w])

    def spawn(self, name, sdf, pos, quat):
        req = SpawnEntity.Request()
        req.entity_factory.name, req.entity_factory.sdf, req.entity_factory.allow_renaming = name, sdf, False
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.entity_factory.pose = pose
        if not self.spawn_cli.call(req).success:
            raise RuntimeError(f'spawn {name} failed')

    def delete(self, name):
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        self.delete_cli.call(req)

    def finger_torque(self):
        return sum(abs(self.eff.get(j, 0.0)) for j in FINGERS)

    def contact_force(self):
        """Sum of |normal force| per finger link from the box contact sensor (None if Gazebo gives no forces)."""
        c = self.contacts
        if c is None or not c.contacts:
            return None
        per, any_force = {}, False
        for ct in c.contacts:
            other = ct.collision2.name if 'test_box' in ct.collision1.name else ct.collision1.name
            for w, n in zip(ct.wrenches, ct.normals):
                f = w.body_1_wrench.force
                fn = abs(f.x * n.x + f.y * n.y + f.z * n.z)
                any_force = any_force or math.hypot(f.x, f.y, f.z) > 1e-9
                key = next((l for l in FINGER_LINKS if l in other), other.split('::')[0])
                per[key] = per.get(key, 0.0) + fn
        return per if any_force else {}


def go_home(n, sec=2.0):
    n.arm(HOME)
    n.gripper(0.0)
    n.wait(sec)


def test_empty(n):
    go_home(n, 3.0)
    n.gripper(1150.0)
    n.wait(3.5)
    print('E empty close (Isaac Sim: fingers touch each other and stop at 1.121 rad = 64.2 deg):')
    print('  ' + '  '.join(f'{j} {n.q[j]:.4f} rad' for j in FINGERS))
    print('  finger joint torques [Nm]: ' + '  '.join(f'{j} {n.eff[j]:+.3f}' for j in FINGERS)
          + f'   sum |tau| {n.finger_torque():.3f}')
    n.gripper(0.0)
    n.wait(3.0)


def grasp_box(n, size, mass, mu, top_above_tcp=0.03):
    """Box held by an anchor between the open fingers (top face top_above_tcp towards the gripper), then close."""
    go_home(n, 3.0)
    p, q = n.tcp_world()
    centre = [a + b for a, b in zip(p, quat_rot(q, [0.0, 0.0, size[2] / 2 - top_above_tcp]))]
    n.box_state = None
    n.spawn('test_box_anchor', ANCHOR_SDF, centre, q)
    n.spawn('test_box', box_sdf(size, mass, mu), centre, q)
    n.wait(0.5)
    n.gripper(1150.0)
    n.wait(3.0)                       # closes in ~2.2 s, then squeezes
    squeeze = n.finger_torque()
    forces = n.contact_force()
    n.detach_pub.publish(Empty())
    n.wait(0.5)
    return p, q, squeeze, forces


def remove_box(n):
    n.clear_pub.publish(Entity(name='test_box::link', type=Entity.LINK))
    n.delete('test_box')
    n.wait(0.1)
    n.delete('test_box_anchor')
    n.gripper(0.0)
    n.wait(2.5)


class Slip:
    """Slip as in Isaac Sim: the box point that was at the grasp point (TCP) when start() was called,
    expressed in the CURRENT TCP frame (the boat drifts slowly even in calm water, the TCP moves with it)."""

    def __init__(self, n):
        self.n = n
        self.g = None

    def start(self):
        p_t, _ = self.n.tcp_world()
        _, pb, qb = self.n.box
        x, y, z, w = qb
        self.g = quat_rot([-x, -y, -z, w], [a - b for a, b in zip(p_t, pb)])   # grasp point in the box frame

    def vec(self, box=None):
        p_t, q_t = self.n.tcp_world()
        _, pb, qb = box or self.n.box
        pt = [a + b for a, b in zip(pb, quat_rot(qb, self.g))]
        x, y, z, w = q_t
        return quat_rot([-x, -y, -z, w], [a - b for a, b in zip(pt, p_t)])

    def mm(self, box=None):
        return math.hypot(*self.vec(box)) * 1000


def test_g3(n, mass, mu, rate, fmax=80.0, top=0.03):
    size = (0.16, 0.06, 0.10)
    p, q, squeeze, forces = grasp_box(n, size, mass, mu, top_above_tcp=top)
    slip = Slip(n)
    slip.start()                                   # grasp point = TCP at the moment the anchor let go
    print(f'G3 pull: box 60x160x100 mm {mass} kg, mu {mu}, box top {top * 1000:+.0f} mm from TCP: squeeze sum |tau| {squeeze:.3f} Nm '
          f'(-> normal force sum {squeeze / PAD_RATIO:.1f} N by Isaac Sim virtual work; Isaac Sim 2.28 Nm / 44.9 N)')
    if forces is None:
        print('  contact sensor: no contacts reported')
    elif not forces:
        print('  contact sensor: contacts but no forces (Gazebo does not fill contact wrenches)')
    else:
        print('  contact normal force per finger link [N]: ' + '  '.join(f'{k} {v:.1f}' for k, v in sorted(forces.items())))
    print('  fingers [rad]: ' + '  '.join(f'{j} {n.q[j]:.4f}' for j in FINGERS))
    n.wait(1.0)
    print(f'  slip while holding 1 s (no pull): {slip.mm():.3f} mm')
    if n.debug:
        for k in range(4):
            n.wait(0.5)
            print(f'    +{0.5 * (k + 1):.1f} s: grasp point in TCP frame {[round(v * 1000, 3) for v in slip.vec()]} mm, '
                  'fingers ' + ' '.join(f'{n.q[j]:.4f}' for j in FINGERS) + ', tau ' + ' '.join(f'{n.eff[j]:+.3f}' for j in FINGERS))
    # pull along the tool axis out of the fingers (+z of the TCP frame = down here) at `rate` N/s
    down = quat_rot(q, [0.0, 0.0, 1.0])
    slip.start()
    t0 = n.now()
    slip_f = slip_dir = None
    applied = 0.0
    while rclpy.ok():
        # ApplyLinkWrench ADDS every persistent wrench message to the ones before (it does not replace them),
        # so send only the increase: the sum of all messages = the pulling force f
        f = rate * (n.now() - t0)
        w = EntityWrench()
        w.entity.name, w.entity.type = 'test_box::link', Entity.LINK
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = [(f - applied) * c for c in down]
        n.wrench_pub.publish(w)
        applied = f
        if slip.mm() >= 1.0:
            slip_f = f
            slip_dir = [round(v * 1000, 2) for v in slip.vec()]
            break
        if f > fmax:
            break
        n.wait(0.02)
    if slip_f is None:
        print(f'  no slip up to {fmax} N')
    else:
        weight = mass * 9.81 * max(0.0, -down[2])
        print(f'  slips (1 mm) at pull {slip_f:.1f} N + weight {weight:.1f} N = slip force {slip_f + weight:.1f} N'
              f'   (Isaac Sim 19.1 N with 0.8 kg)')
        print(f'  grasp point moved (TCP frame x / y = grasp direction / z = tool axis) {slip_dir} mm, '
              'fingers ' + ' '.join(f'{n.q[j]:.4f}' for j in FINGERS))
    remove_box(n)
    return slip_f


def test_hold(n, name, size, mass, mu, sec, top=-0.015):
    p, q, squeeze, forces = grasp_box(n, size, mass, mu, top_above_tcp=top)   # pad contact, as G3
    if n.debug:
        print(f'  {name} contacts [N]: ' + ('  '.join(f'{k} {v:.1f}' for k, v in sorted(forces.items())) if forces else str(forces))
              + '   fingers ' + ' '.join(f'{n.q[j]:.4f}' for j in FINGERS) + '   tau ' + ' '.join(f'{n.eff[j]:+.3f}' for j in FINGERS))
    sl = Slip(n)
    sl.start()
    n.box_log.clear()
    qs = []
    t0 = n.now()
    while n.now() - t0 < sec:
        qs.append([n.q[j] for j in ARM + FINGERS])
        n.wait(0.01)
    rel = [sl.vec(b) for b in n.box_log[::10]]
    if n.debug:
        step = max(1, len(rel) // 6)
        print('  grasp point in TCP frame over time [mm]: ' + '  '.join(str([round(v * 1000, 2) for v in r]) for r in rel[::step]))
    slip = max(math.hypot(*r) for r in rel) * 1000
    jit_box = max(max(r[k] for r in rel) - min(r[k] for r in rel) for k in range(3)) * 1000
    jit_q = max(max(s[k] for s in qs) - min(s[k] for s in qs) for k in range(len(qs[0])))
    print(f'{name}: box {[round(v * 1000) for v in size]} mm {mass} kg, mu {mu}, {sec:g} s: slip {slip:.2f} mm, '
          f'box jitter {jit_box:.3f} mm, joint jitter {math.degrees(jit_q):.4f} deg, squeeze {squeeze:.3f} Nm')
    remove_box(n)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--world', default='empty_ocean_calm')
    a.add_argument('--tests', nargs='+', default=['E', 'G3'], choices=['E', 'G3', 'G1', 'G4', 'H'])
    a.add_argument('--mu', type=float, default=0.6, help='box friction coefficient (fingers: gripper_friction in the robot model, 0.6)')
    a.add_argument('--mass', type=float, default=0.8, help='G3 box mass [kg] (Isaac Sim 19.1 N was with 0.8 kg)')
    a.add_argument('--rate', type=float, default=5.0, help='G3 pull force rate [N/s] (Isaac Sim 5)')
    a.add_argument('--debug', action='store_true', help='print box / TCP drift before pulling')
    a.add_argument('--box-mesh', default=None, help='box collision as this STL (mesh-vs-mesh contact check)')
    a.add_argument('--top', type=float, default=-0.015, help='G3: box top face this far from the TCP towards the gripper [m]')
    args = a.parse_args()

    rclpy.init()
    n = GraspTest(args.world)
    n.debug = args.debug
    global BOX_MESH
    BOX_MESH = args.box_mesh
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    w = args.world
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/model/test_box/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
         '/test_box/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
         '/test_box/detach@std_msgs/msg/Empty]gz.msgs.Empty',
         '/test_box/state@std_msgs/msg/String[gz.msgs.StringMsg',
         f'/world/{w}/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
         f'/world/{w}/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
         '--ros-args', '-p', 'use_sim_time:=true'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        while rclpy.ok() and (not n.q or n.boat is None or n.arm_pub.get_subscription_count() == 0
                              or not n.spawn_cli.service_is_ready() or n.wrench_pub.get_subscription_count() == 0):
            time.sleep(0.1)
        for t in args.tests:
            if t == 'E':
                test_empty(n)
            elif t == 'G3':
                test_g3(n, args.mass, args.mu, args.rate, top=args.top)
            elif t == 'G1':
                test_hold(n, 'G1', (0.04, 0.04, 0.04), 0.2, args.mu, 3.0, top=args.top)
            elif t == 'G4':
                test_hold(n, 'G4', (0.16, 0.06, 0.10), 0.5, args.mu, 5.0, top=args.top)
            elif t == 'H':                 # hold a G3-size box of --mass without pulling
                test_hold(n, 'H', (0.16, 0.06, 0.10), args.mass, args.mu, 3.0, top=args.top)
    finally:
        bridge.terminate()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
