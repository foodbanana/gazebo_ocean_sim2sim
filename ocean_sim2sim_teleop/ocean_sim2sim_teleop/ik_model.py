# =============================================================
# ik_model.py - arm kinematics from the URDF + one differential IK step (numpy and the standard library only)
#
#   Ported from: data_collection_ur5_gripper teleop/ik_model.py (ArmModel, ik_step) - same equations and limits.
#   ArmModel(urdf_xml, base, tip, arm_joints)
#     fk(q)            -> 4x4, tip pose in the base frame. q = 6 arm joints [rad] (arm_joints order)
#     jacobian(q)      -> 6 x 6 [v; w], base frame, point = tip origin (geometric Jacobian)
#     limits           -> (lo, hi) [rad]
#     joint_origins(q) -> 6 x 3, arm joint origins (base frame)
#   ik_step(model, q_cmd, q_meas, p_target, R_target, ...) -> next joint target (IkStep)
#     e = [p* - p, log(R* R^T)],  dq = J^T (J J^T + lambda^2 I)^-1 e,  |dq| <= max_dq (per joint, ratio kept)
#     joint target = previous target + dq
#     The error is taken from the FK of the COMMANDED joints (the node runs outside the sim: command -> arm takes
#       50 ~ 80 ms, adding measured errors meanwhile overshoots). The measurement is only used against wind-up
#       (target at most max_lead ahead of the measurement).
#     Joint limits: clipped to limit - limit_margin and reported (the interface rejects /joint_command outside the limits)
#
#   Changes from the Isaac Sim file: the URDF comes as XML text (the node reads /robot_description, no file path; the
#     Gazebo URDF is the boat + arm, the chain robot_mount -> rh_p12_rn_tcp is the same arm), comments in English.
#   Self check (chain summary, home FK, Jacobian vs finite differences, ik_step convergence):
#       python3 -m ocean_sim2sim_teleop.ik_model --urdf <file>    (or the node's --self-check with /robot_description)
# =============================================================

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
BASE_LINK = "robot_mount"           # robot base (dataset "robot base frame")
TIP_LINK = "rh_p12_rn_tcp"          # between the fingers (z 0.11 from rh_p12_rn_base)
HOME = (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)


# ---------------------------------------------------------------- rotations
def rpy_to_R(r, p, y):
    """URDF rpy (fixed axes x -> y -> z): R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_R(axis, q):
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def R_to_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w), w >= 0. Shepperd: divide by the largest component (stable everywhere)."""
    t = np.trace(R)
    d = np.array([t, R[0, 0], R[1, 1], R[2, 2]])
    k = int(np.argmax(d))
    if k == 0:
        w = math.sqrt(1 + t) / 2
        q = np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    else:
        i, j, l = k - 1, k % 3, (k + 1) % 3
        s = math.sqrt(max(1 + 2 * R[i, i] - t, 0.0)) / 2
        q = np.zeros(4)
        q[i] = s
        q[j] = (R[j, i] + R[i, j]) / (4 * s)
        q[l] = (R[l, i] + R[i, l]) / (4 * s)
        q[3] = (R[l, j] - R[j, l]) / (4 * s)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def R_to_rotvec(R):
    """Rotation matrix -> rotation vector (axis x angle, 0 ~ pi), via the quaternion and atan2 (precise near 180 deg)."""
    q = R_to_quat(R)
    s = float(np.linalg.norm(q[:3]))
    if s < 1e-12:
        return np.zeros(3)
    return (2 * math.atan2(s, q[3]) / s) * q[:3]


# ---------------------------------------------------------------- model
class ArmModel:
    """Serial chain base -> tip of the URDF. Fixed joints = origin transform only; moving joints must be in arm_joints
    (an unknown moving joint is an error)."""

    def __init__(self, urdf_xml, base=BASE_LINK, tip=TIP_LINK, arm_joints=ARM_JOINTS):
        self.base, self.tip, self.arm_joints = base, tip, tuple(arm_joints)
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            lim = j.find("limit")
            by_child[j.find("child").get("link")] = {
                "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                "T": make_T(rpy_to_R(*rpy), np.array(xyz)),
                "axis": np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()]),
                "lo": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                "hi": float(lim.get("upper")) if lim is not None and lim.get("upper") else None}
        chain, link = [], tip                      # from the tip back to the base
        while link != base:
            if link not in by_child:
                raise ValueError(f"no chain from {tip} to {base} (no parent joint of {link})")
            chain.append(by_child[link])
            link = chain[-1]["parent"]
        chain.reverse()
        self.chain, names = [], []
        for j in chain:
            if j["type"] == "fixed":
                self.chain.append((j["T"], None, None))
            elif j["type"] in ("revolute", "continuous"):
                if j["name"] not in self.arm_joints:
                    raise ValueError(f"moving joint outside arm_joints in the chain: {j['name']}")
                n = np.linalg.norm(j["axis"])
                self.chain.append((j["T"], self.arm_joints.index(j["name"]), j["axis"] / n))
                names.append(j["name"])
            else:
                raise ValueError(f"unsupported joint type: {j['name']} ({j['type']})")
        if sorted(names) != sorted(self.arm_joints):
            raise ValueError(f"moving joints of the chain {names} != arm_joints {list(self.arm_joints)}")
        js = {by_child[c]["name"]: by_child[c] for c in by_child}
        self.lo = np.array([js[n]["lo"] for n in self.arm_joints], dtype=float)
        self.hi = np.array([js[n]["hi"] for n in self.arm_joints], dtype=float)

    @property
    def limits(self):
        return self.lo.copy(), self.hi.copy()

    def _frames(self, q):
        """(base-frame axis, joint origin) of each moving joint and the tip pose."""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.arm_joints),) or not np.all(np.isfinite(q)):
            raise ValueError(f"joint angles must be {len(self.arm_joints)} finite values: {q}")
        T = np.eye(4)
        axes = []
        for To, idx, ax in self.chain:
            T = T @ To
            if idx is not None:
                axes.append((idx, T[:3, :3] @ ax, T[:3, 3].copy()))
                T = T @ make_T(axis_angle_R(ax, q[idx]), np.zeros(3))
        return axes, T

    def fk(self, q):
        return self._frames(q)[1]

    def joint_origins(self, q):
        """Arm joint origins (6 x 3, base frame, arm_joints order). Used for the wrist height check."""
        P = np.zeros((len(self.arm_joints), 3))
        for idx, _, o in self._frames(q)[0]:
            P[idx] = o
        return P

    def jacobian(self, q):
        axes, T = self._frames(q)
        p = T[:3, 3]
        J = np.zeros((6, len(self.arm_joints)))
        for idx, z, o in axes:
            J[:3, idx] = np.cross(z, p - o)
            J[3:, idx] = z
        return J


# ---------------------------------------------------------------- one differential IK step
class IkStep:
    __slots__ = ("q", "pos_err", "rot_err", "limited", "manip")

    def __init__(self, q, pos_err, rot_err, limited, manip):
        self.q, self.pos_err, self.rot_err, self.limited, self.manip = q, pos_err, rot_err, limited, manip

    def __repr__(self):
        return (f"IkStep(q={np.round(self.q, 4).tolist()}, pos_err={self.pos_err * 1000:.2f} mm, "
                f"rot_err={math.degrees(self.rot_err):.2f} deg, limited={self.limited}, manip={self.manip:.4f})")


def ik_step(model, q_cmd, q_meas, p_target, R_target, max_dq=0.05, damping=0.02, max_lead=0.05, limit_margin=0.05):
    """Next joint target towards the TCP target pose (base frame).
    q_cmd: current joint target (last step), q_meas: measured joints (anti wind-up).
    max_dq [rad]: max change per step (per joint, ratio kept). max_lead [rad]: max target lead over the measurement.
    limit_margin [rad]: margin inside the joint limits. limited = joint names clipped by a limit or the lead.
    manip = sqrt(det(J J^T)) (near 0 = singular)."""
    q_cmd, q_meas = np.asarray(q_cmd, dtype=float), np.asarray(q_meas, dtype=float)
    p_target, R_target = np.asarray(p_target, dtype=float), np.asarray(R_target, dtype=float)
    if not (np.all(np.isfinite(p_target)) and np.all(np.isfinite(R_target))):
        raise ValueError("target pose is not finite")
    T = model.fk(q_cmd)
    e = np.concatenate([p_target - T[:3, 3], R_to_rotvec(R_target @ T[:3, :3].T)])
    J = model.jacobian(q_cmd)
    JJt = J @ J.T
    dq = J.T @ np.linalg.solve(JJt + damping ** 2 * np.eye(6), e)
    m = float(np.max(np.abs(dq)) / max_dq)
    if m > 1:
        dq /= m
    q_new = q_cmd + dq
    limited = []
    lo_lead, hi_lead = q_meas - max_lead, q_meas + max_lead
    lo_lim, hi_lim = model.lo + limit_margin, model.hi - limit_margin
    for i, n in enumerate(model.arm_joints):
        if q_new[i] < lo_lead[i] or q_new[i] > hi_lead[i]:
            q_new[i] = min(max(q_new[i], lo_lead[i]), hi_lead[i])
            limited.append(f"{n}:lead")
        if q_new[i] < lo_lim[i] or q_new[i] > hi_lim[i]:
            q_new[i] = min(max(q_new[i], lo_lim[i]), hi_lim[i])
            limited.append(f"{n}:limit")
    manip = float(math.sqrt(max(np.linalg.det(JJt), 0.0)))
    return IkStep(q_new, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:])), limited, manip)


# ---------------------------------------------------------------- self check
def self_check(urdf_xml, log=print):
    m = ArmModel(urdf_xml)
    log(f"chain {BASE_LINK} -> {TIP_LINK}: {len(m.chain)} transforms, moving joints {list(m.arm_joints)}")
    log(f"joint limits [rad]: {np.round(m.lo, 4).tolist()} ~ {np.round(m.hi, 4).tolist()}")
    T = m.fk(HOME)
    log(f"home TCP (base) [m]: {np.round(T[:3, 3], 4).tolist()}, tool axis (z) {np.round(T[:3, 2], 3).tolist()}")
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        J = m.jacobian(q)
        Jn = np.zeros((6, 6))
        T0 = m.fk(q)
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            T1 = m.fk(q + dq)
            Jn[:3, i] = (T1[:3, 3] - T0[:3, 3]) / 1e-6
            Jn[3:, i] = R_to_rotvec(T1[:3, :3] @ T0[:3, :3].T) / 1e-6
        worst = max(worst, float(np.max(np.abs(J - Jn))))
    log(f"Jacobian vs finite differences (200 random poses): max difference {worst:.2e}")
    ok = 0
    for _ in range(50):
        q0 = np.array(HOME) + rng.uniform(-0.4, 0.4, 6)
        Tt = m.fk(q0 + rng.uniform(-0.15, 0.15, 6))
        q = q0.copy()
        for _ in range(200):
            s = ik_step(m, q, q, Tt[:3, 3], Tt[:3, :3], max_lead=10.0)
            q = s.q
            if s.pos_err < 1e-5 and s.rot_err < 1e-4:
                ok += 1
                break
    log(f"ik_step convergence (50 random targets near home, 0.01 mm / 0.006 deg within 200 steps): {ok} / 50")
    return worst < 1e-4 and ok >= 48


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="URDF arm kinematics self check")
    ap.add_argument("--urdf", required=True, help="URDF file (e.g. xacro output of wamv_with_arm.urdf.xacro)")
    a = ap.parse_args()
    with open(a.urdf, encoding="utf-8") as f:
        sys.exit(0 if self_check(f.read()) else 1)
