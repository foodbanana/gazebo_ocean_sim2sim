"""Run metadata (spec C) and the one-time facts E3 (mounting) and E6 (wave model) for the boat motion recordings.

Everything is read from the running / generated sources (robot_description, the wavefield parameter message, the
URDF IMU sensor block, versions), not typed in, so a changed model or world shows up in meta.json.
"""
import json
import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from fractions import Fraction

import numpy as np

from ocean_sim2sim_interface.kinematics import Chain

G = 9.81
# VRX Wavefield (vrx_gz/src/Wavefield.cc) defaults for the keys a world does not set
VRX_WAVE_DEFAULTS = {'model': 'PMS', 'number': 3, 'scale': 1.1, 'angle': 0.4, 'steepness': 0.0, 'amplitude': 0.0,
                     'period': 5.0, 'direction': 0.0, 'gain': 1.0, 'tau': 2.0}


# ------------------------------------------------------------------------------------------------ repository / versions
def repo_root():
    """Source repository of this package (symlink install: the module is a link into src), or None."""
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(4):
        if os.path.isdir(os.path.join(d, '.git')):
            return d
        d = os.path.dirname(d)
    return None


def run_text(cmd, cwd=None, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd)
        return (r.stdout or r.stderr).strip()
    except (OSError, subprocess.TimeoutExpired):
        return None


def versions():
    root = repo_root()
    gz = run_text(['gz', 'sim', '--version'])
    out = {'gazebo': gz.splitlines()[0] if gz else None, 'ros_distro': os.environ.get('ROS_DISTRO')}
    if root:
        out['gazebo_ocean_sim2sim_commit'] = run_text(['git', 'describe', '--always', '--dirty'], cwd=root)
        vrx = os.path.join(os.path.dirname(root), 'vrx')
        if os.path.isdir(os.path.join(vrx, '.git')):
            out['vrx_commit'] = run_text(['git', 'describe', '--always', '--dirty'], cwd=vrx)
    return out


# ------------------------------------------------------------------------------------------------ waves (E6)
def read_wave_params(timeout=15):
    """Wavefield parameters as Gazebo uses them (one /vrx/wavefield/parameters message, sent every 2 s by the world's
    vrx::PublisherPlugin). Keys the world does not send keep the VRX defaults."""
    text = run_text(['gz', 'topic', '-e', '-t', '/vrx/wavefield/parameters', '-n', '1'], timeout=timeout) or ''
    sent = {}
    for key, body in re.findall(r'key:\s*"(\w+)"\s*value\s*\{([^}]*)\}', text):
        m = re.search(r'(double_value|int_value|string_value):\s*"?([^"\s]+)"?', body)
        if m:
            sent[key] = float(m.group(2)) if m.group(1) == 'double_value' else (
                int(m.group(2)) if m.group(1) == 'int_value' else m.group(2))
    params = dict(VRX_WAVE_DEFAULTS, **sent)
    return params, sorted(sent)


def _pm(omega, omega_p):
    return 0.0081 * G ** 2 / omega ** 5 * math.exp(-1.25 * (omega_p / omega) ** 4)


def wave_components(p):
    """Component waves exactly as VRX Wavefield::RecalculatePms (3 components; the spacing table has 3 entries, so
    number > 3 is not supported by VRX). Height eta(x, t) = sum a_i cos(k_i (d_i . x) - w_i t) (all phases 0) times
    (1 - exp(-t / tau)); steepness q_i > 0 makes them Gerstner (trochoidal) waves."""
    if p['model'] != 'PMS' or int(p['number']) != 3:
        return None
    wp, s, n_c = 2 * math.pi / p['period'], p['scale'], 3
    spacing = [wp * (1 - 1 / s), wp * (s - 1 / s) / 2, wp * (s - 1)]
    comps = []
    for i in range(n_c):
        n = i - 1
        w = wp * s ** n
        a = p['gain'] * math.sqrt(2 * _pm(w, wp) * spacing[i])
        k = w * w / G
        q = min(1.0, p['steepness'] / (a * k * n_c)) if a > 0 else 0.0
        d = n * p['angle'] + p['direction']
        comps.append({'amplitude_m': round(a, 5), 'period_s': round(2 * math.pi / w, 4), 'omega_rad_s': round(w, 5),
                      'wavelength_m': round(2 * math.pi / k, 3), 'wavenumber_rad_m': round(k, 5),
                      'direction_rad': round(d, 5), 'direction_deg': round(math.degrees(d), 2),
                      'steepness_q': round(q, 4), 'phase_rad': 0.0})
    return comps


def wave_summary(p):
    comps = wave_components(p)
    out = {'model': f"VRX Wavefield {p['model']} (Pierson-Moskowitz spectrum sampled at {int(p['number'])} frequencies)",
           'params': p, 'components': comps,
           'random_seed': None,
           'notes': ['all component phases are 0 and there is no random seed: the same parameters give exactly the '
                     'same waves in every run (pairs arm fixed / arm moving share the same waves)',
                     'number > 3 is not possible with the VRX PMS model (spacing table of 3), the wave visual shader '
                     'also has 3 waves',
                     'wave height ramps up with 1 - exp(-t / tau) from the start of the simulation']}
    if comps:
        # exact repeat period of the sum (frequency ratios scale^n as fractions)
        r = Fraction(p['scale']).limit_denominator(1000)
        ratios = [r ** (i - 1) for i in range(3)]
        lcm_den = math.lcm(*[x.denominator for x in ratios])
        nums = [x.numerator * (lcm_den // x.denominator) for x in ratios]
        g = math.gcd(*nums)
        f0 = Fraction(g, lcm_den) / Fraction(p['period']).limit_denominator(1000)
        out['repeat_period_s'] = round(float(1 / f0), 3)
        w = sorted(c['omega_rad_s'] for c in comps)
        out['beat_periods_s'] = [round(2 * math.pi / (w[1] - w[0]), 2), round(2 * math.pi / (w[2] - w[1]), 2)]
        var = sum(c['amplitude_m'] ** 2 / 2 for c in comps)
        out['significant_height_hs_m'] = round(4 * math.sqrt(var), 4)
        out['max_crest_to_trough_m'] = round(2 * sum(c['amplitude_m'] for c in comps), 4)
        out['direction_meaning'] = ('direction = where the waves travel to, angle from world +x (ENU, counter-clockwise '
                                    'from above); the boat starts with its bow on +x, so 0 = following sea (waves from '
                                    'astern), pi = head sea, pi/2 = waves travel to the boat\'s port (+y) side')
    return out


# ------------------------------------------------------------------------------------------------ mounting (E3)
def _pose(chain, q=None):
    T = chain.fk(np.zeros(len(chain.joints)) if q is None else np.asarray(q))
    from scipy.spatial.transform import Rotation
    return {'xyz_m': [round(float(v), 5) for v in T[:3, 3]],
            'quat_xyzw': [round(float(v), 6) for v in Rotation.from_matrix(T[:3, :3]).as_quat()],
            'rpy_rad': [round(float(v), 6) for v in Rotation.from_matrix(T[:3, :3]).as_euler('xyz')]}


def mounting(urdf, init_pose, boat_link='wamv/base_link', imu_link='wamv/imu_wamv_link', base_link='robot_mount',
             flange_link='tool0', tcp_link='rh_p12_rn_tcp'):
    """E3: IMU and robot base in the boat body frame, flange -> TCP, TCP at the init pose."""
    out = {'frame': f'{boat_link} (WAM-V body frame, FLU: x bow, y port, z up; origin = VRX hull origin)'}
    out['imu_in_body'] = _pose(Chain(urdf, boat_link, imu_link))      # IMU joint is a locked revolute (VRX), q = 0
    out['robot_base_in_body'] = _pose(Chain(urdf, boat_link, base_link))
    fl = Chain(urdf, base_link, flange_link)
    tcp = Chain(urdf, base_link, tcp_link)
    out['flange'] = flange_link
    out['tcp'] = tcp_link
    Tf, Tt = fl.fk(np.asarray(init_pose)), tcp.fk(np.asarray(init_pose))
    out['flange_to_tcp_m'] = round(float(np.linalg.norm(Tt[:3, 3] - Tf[:3, 3])), 5)
    out['flange_to_tcp_note'] = 'Gazebo / Isaac Sim gripper model; the real gripper is about 0.151 m (spec E3)'
    out['init_pose_rad'] = list(init_pose)
    out['tcp_at_init_pose_in_robot_base'] = _pose(tcp, init_pose)
    full = Chain(urdf, boat_link, tcp_link)
    out['tcp_at_init_pose_in_body'] = _pose(full, list(init_pose))
    out['tcp_distance_from_body_origin_m'] = round(float(np.linalg.norm(full.fk(np.asarray(init_pose))[:3, 3])), 4)
    root = ET.fromstring(urdf)
    for link in root.findall('link'):
        if link.get('name') == boat_link:
            inertial = link.find('inertial')
            if inertial is not None:
                out['body_link_mass_kg_urdf'] = float(inertial.find('mass').get('value'))
    return out


def imu_sensor(urdf, sensor_name='imu_wamv_sensor'):
    """IMU sensor settings from the URDF <gazebo> block: rate, noise per axis, orientation reference."""
    root = ET.fromstring(urdf)
    for s in root.iter('sensor'):
        if s.get('name') != sensor_name:
            continue
        out = {'name': sensor_name, 'type': s.get('type'),
               'update_rate_hz': float(s.findtext('update_rate', 'nan'))}
        imu = s.find('imu')
        for group in ('angular_velocity', 'linear_acceleration'):
            g = imu.find(group) if imu is not None else None
            if g is None:
                continue
            out[group] = {ax: {c.tag: (float(c.text) if c.text and re.match(r'^[-0-9.eE+]+$', c.text.strip()) else c.text)
                               for c in g.find(ax).find('noise')} | {'type': g.find(ax).find('noise').get('type')}
                          for ax in ('x', 'y', 'z') if g.find(ax) is not None}
        ref = imu.find('orientation_reference_frame') if imu is not None else None
        out['orientation_reference'] = ref.findtext('localization') if ref is not None else None
        out['orientation_noise'] = None
        out['notes'] = ['orientation: no noise option in the gz IMU (A6 is noise free, world ENU -> IMU)',
                        'units: angular_velocity rad/s, linear_acceleration m/s^2 (specific force: +9.81 on z at rest)']
        return out
    return None


def write_json(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
        f.write('\n')
