"""Grasp judge observations from Gazebo: what the judge sees, not what a policy gets. docs/grasp_demo.md, docs/policy.md

Shared by ocean_sim2sim_bringup grasp_demo.py (scripted demo) and ocean_sim2sim_policy policy_eval.py (policy evaluation),
so both judge from the same sources in the same way (factored out of grasp_demo.py, behaviour unchanged):
  /robot_description (latched)  TCP chain base_link -> tcp_link and boat_link -> base_link (any arm, read from the URDF)
  /joint_states (arm, 120 Hz)    arm joints for the TCP forward kinematics
  /gripper/joint_states (raw)    gripper angle = raw / gripper_raw_max * gripper_upper [rad]
  /sim/boat_odometry             boat pose (world) -> robot base pose -> TCP pose (world)
  /drone/odometry                drone body pose (world)
  /drone/contacts                contact sensor of each drone link (body, rotors) on one topic: merged per sim step
  /drone/state, /drone/px4/state fixed drone attached / detached; PX4 drone armed flag (= motors on, as Isaac Sim judge)

Use as a mixin of an rclpy Node (use_sim_time true): call init_grasp_observation() in __init__, setup_kinematics() once
observation_missing() is empty, then judge_observation() at every judge tick (input of grasp_judge.GraspJudge.update).
"""
import json

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ocean_sim2sim_interface.kinematics import Chain, quat_matrix

FINGER_LINKS = ('rh_p12_rn_r1', 'rh_p12_rn_r2', 'rh_p12_rn_l1', 'rh_p12_rn_l2')


def pose_matrix(pose):
    p, o = pose.position, pose.orientation
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = quat_matrix([o.x, o.y, o.z, o.w]), [p.x, p.y, p.z]
    return T


class GraspObservation:
    """Mixin for rclpy.node.Node: subscriptions + world poses / forces for the grasp judge."""

    def init_grasp_observation(self, gripper_upper, gripper_raw_max):
        self.gripper_upper, self.gripper_raw_max = float(gripper_upper), float(gripper_raw_max)
        self.q = {}
        self.boat = self.drone = self.contacts = self.urdf = None
        self.drone_state = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data), latched)
        self.js_t = -1.0
        # what a policy gets: /joint_states (arm, 120 Hz) and /gripper/joint_states (raw, camera loop ~30 Hz).
        # (the 240 Hz all-joint topic made this Python node fall up to 200 ms behind)
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.create_subscription(JointState, '/gripper/joint_states', self._on_grip, 20)
        self.create_subscription(Odometry, '/sim/boat_odometry', lambda m: setattr(self, 'boat', m), 50)
        self.create_subscription(Odometry, '/drone/odometry', lambda m: setattr(self, 'drone', m), 50)
        self.create_subscription(Contacts, '/drone/contacts', self._on_contacts, 50)
        self.create_subscription(String, '/drone/state', lambda m: setattr(self, 'drone_state', m.data), 10)
        # PX4 drone (drone_mode px4): motors on = PX4 armed (Isaac Sim judge 'armed' = PX4 armed flag)
        self.px4 = None
        self.create_subscription(String, '/drone/px4/state', lambda m: setattr(self, 'px4', json.loads(m.data)), 10)
        self.contact_t = -1.0

    def _on_js(self, m):
        self.q.update(zip(m.name, m.position))
        self.js_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _on_grip(self, m):
        self.q['rh_r1_joint'] = m.position[0] * self.gripper_upper / self.gripper_raw_max

    def _on_contacts(self, m):
        # one contact sensor per drone link (body, rotor0..3) on the same topic: merge the messages of one sim step
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.contacts is not None and t == self.contact_t:
            self.contacts.contacts.extend(m.contacts)
        else:
            self.contacts = m
            self.contact_t = t

    def observation_missing(self):
        """{source: still missing} of the judge inputs (all False = ready)."""
        return {'robot_description': self.urdf is None, 'gripper state': 'rh_r1_joint' not in self.q,
                'joint states': self.js_t < 0, 'boat odometry': self.boat is None, 'drone odometry': self.drone is None}

    def setup_kinematics(self, base_link, tcp_link, boat_link):
        self.chain = Chain(self.urdf, base_link, tcp_link)
        self.arm = self.chain.joints
        self.boat_to_base = Chain(self.urdf, boat_link, base_link).fk(np.zeros(0))

    # ------------------------------------------------------------------ state
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def q_arm(self):
        return np.array([self.q[j] for j in self.arm])

    def base_world(self):
        return pose_matrix(self.boat.pose.pose) @ self.boat_to_base

    def tcp_world(self):
        T = self.base_world() @ self.chain.fk(self.q_arm())
        return T[:3, 3], T[:3, :3]

    def drone_world(self):
        T = pose_matrix(self.drone.pose.pose)
        return T[:3, 3], T[:3, :3]

    def finger_forces(self):
        """Net force [N] per robot link from the drone contact sensor (as Isaac Sim: norm of the summed force per link)."""
        if self.contacts is None or self.now() - self.contact_t > 0.05:
            return {}
        f = {}
        for ct in self.contacts.contacts:
            c1, c2 = ct.collision1.name, ct.collision2.name
            drone_first = c1.startswith('drone::')
            other = c2 if drone_first else c1
            key = next((ln for ln in FINGER_LINKS if ln in other), other.split('::')[1] if '::' in other else other)
            for w in ct.wrenches:
                v = w.body_1_wrench.force
                vec = np.array([v.x, v.y, v.z]) * (-1.0 if drone_first else 1.0)     # force on the robot link
                f[key] = f.get(key, np.zeros(3)) + vec
        return {k: float(np.linalg.norm(v)) for k, v in f.items()}

    def drone_armed(self, armed, kill_requested):
        """Motors on. PX4 drone: the PX4 armed flag. Fixed drone: off (and stays off) once it is detached after a kill."""
        if self.px4 is not None:
            return bool(self.px4.get('armed'))
        if self.drone_state == 'detached' and armed and kill_requested:
            return False
        return armed

    def judge_observation(self, grip_closing, armed):
        """-> (observation dict for GraspJudge.update, finger_forces() of this tick)."""
        f = self.finger_forces()
        right = f.get('rh_p12_rn_r1', 0.0) + f.get('rh_p12_rn_r2', 0.0)
        left = f.get('rh_p12_rn_l1', 0.0) + f.get('rh_p12_rn_l2', 0.0)
        other = float(sum(v for k, v in f.items() if k not in FINGER_LINKS))
        tp, tR = self.tcp_world()
        dp, dR = self.drone_world()
        o = dict(t=self.now(), grip_q=float(self.q['rh_r1_joint']), grip_closing=grip_closing, armed=armed,
                 f_right=right, f_left=left, f_other=other, drone_p=dp, drone_R=dR, tcp_p=tp, tcp_R=tR)
        return o, f
