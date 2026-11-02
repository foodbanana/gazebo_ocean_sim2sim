"""Serial chain kinematics from a URDF string (no KDL): forward kinematics, Jacobian, damped least squares IK step.

Works for any arm (the chain is read from robot_description between two links), so the arm stays replaceable.
Revolute / continuous / prismatic / fixed joints. numpy only.
"""
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def quat_matrix(q):
    """(x, y, z, w) -> 3x3."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_log(R):
    """Rotation vector of R."""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.pi - a < 1e-6:                      # 180 deg: axis from the symmetric part
        k = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0.0))
        i = int(np.argmax(k))
        k[i] = k[i] if k[i] > 0 else 1.0
        for j in range(3):
            if j != i:
                k[j] = np.copysign(k[j], R[i, j] + R[j, i])
        return a * k / np.linalg.norm(k)
    return a * v / (2 * np.sin(a))


class Chain:
    """Chain root_link -> tip_link of a URDF. FK in the root link frame."""

    def __init__(self, urdf_xml, root_link, tip_link):
        joints = {j.find('child').get('link'): j for j in ET.fromstring(urdf_xml).findall('joint')}
        seq, link = [], tip_link
        while link != root_link:
            j = joints.get(link)
            if j is None:
                raise ValueError(f'{tip_link} is not below {root_link} in the URDF')
            seq.append(j)
            link = j.find('parent').get('link')
        self.elems = []                       # (name or None, type, origin T, axis)
        for j in reversed(seq):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = rpy_matrix(*rpy), xyz
            a = j.find('axis')
            axis = [float(v) for v in (a.get('xyz') if a is not None else '1 0 0').split()]
            typ = j.get('type')
            self.elems.append((None if typ == 'fixed' else j.get('name'), typ, T, np.asarray(axis)))
        self.joints = [e[0] for e in self.elems if e[0] is not None]

    def fk(self, q):
        """q: joint values in self.joints order -> 4x4 tip pose in the root frame."""
        T, k = np.eye(4), 0
        for name, typ, To, axis in self.elems:
            T = T @ To
            if name is None:
                continue
            M = np.eye(4)
            if typ == 'prismatic':
                M[:3, 3] = axis * q[k]
            else:
                M[:3, :3] = axis_angle(axis, q[k])
            T = T @ M
            k += 1
        return T

    def jacobian(self, q):
        """Geometric Jacobian (6 x n) of the tip, root frame, [v; w]."""
        T, k = np.eye(4), 0
        frames = []
        for name, typ, To, axis in self.elems:
            T = T @ To
            if name is None:
                continue
            frames.append((typ, T[:3, 3].copy(), T[:3, :3] @ axis))
            M = np.eye(4)
            if typ == 'prismatic':
                M[:3, 3] = axis * q[k]
            else:
                M[:3, :3] = axis_angle(axis, q[k])
            T = T @ M
            k += 1
        p = T[:3, 3]
        J = np.zeros((6, len(frames)))
        for i, (typ, o, z) in enumerate(frames):
            if typ == 'prismatic':
                J[:3, i] = z
            else:
                J[:3, i], J[3:, i] = np.cross(z, p - o), z
        return J


def pose_error(T, p_t, R_t):
    """[position error; rotation vector of R_t R^T] from pose T to the target."""
    return np.concatenate([np.asarray(p_t) - T[:3, 3], rot_log(R_t @ T[:3, :3].T)])


def dls_step(chain, q, p_t, R_t, damping, max_dq):
    """One damped least squares step from q towards the target: dq (largest |dq_i| <= max_dq, ratio kept), error."""
    e = pose_error(chain.fk(q), p_t, R_t)
    J = chain.jacobian(q)
    dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
    m = np.max(np.abs(dq)) / max_dq
    if m > 1:
        dq /= m
    return dq, e
