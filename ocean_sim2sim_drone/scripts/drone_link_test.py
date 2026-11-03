#!/usr/bin/env python3
"""Check of the Gazebo drone plugin (DroneLockstep) without PX4: stands in for the bridge and answers every exchange
with fixed rotor speeds through the Isaac Sim thrust model (ocean_sim2sim_drone.flight), plus the prop joint speeds
(Isaac prop spin rule, --no-prop-spin: 0). docs/drone_px4.md

Run with the scene up (drone hanging on its anchor), then:
    ros2 run ocean_sim2sim_drone drone_link_test.py --test hover      # hover omega for 0.8 kg, drone released after 1 s
    ros2 run ocean_sim2sim_drone drone_link_test.py --test roll       # rotor speeds that should roll the drone left (+x)
Prints: exchange rate in sim time, RTF while in lockstep, vertical acceleration and tilt after release.
"""
import argparse
import os
import socket
import struct
import subprocess
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from ocean_sim2sim_drone.flight import ThrustModel, load_flight, prop_speeds, quat_wxyz_to_R

IN = struct.Struct('<15d')
OUT = struct.Struct('<10d')                         # wrench (6) + prop joint speeds (4)


def recv_exact(conn, n):
    b = b''
    while len(b) < n:
        k = conn.recv(n - len(b))
        if not k:
            raise ConnectionError('plugin closed the connection')
        b += k
    return b


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--test', choices=('hover', 'roll'), default='hover')
    a.add_argument('--port', type=int, default=4570)
    a.add_argument('--release-after', type=float, default=1.0, help='s sim after connecting: /drone/detach')
    a.add_argument('--no-prop-spin', action='store_true', help='reply prop joint speeds 0 (props held still)')
    a.add_argument('--duration', type=float, default=2.0, help='s sim after release')
    a.add_argument('--flight', default=os.path.join(get_package_share_directory('ocean_sim2sim_gazebo'), 'models', 'iris_075',
                                                    'meshes', 'collision_isaac', 'isaac_flight.yaml'))
    args = a.parse_args()
    fl = load_flight(args.flight)
    tm = ThrustModel(fl)
    w0 = tm.hover_omega(fl['mass'])
    if args.test == 'hover':
        cmd = np.full(4, w0)
    else:
        # more thrust on the rotors with y < 0 (right side) -> roll left = positive rotation about +x
        cmd = np.array([w0 * (1.05 if p[1] < 0 else 0.95) for p in tm.pos])
    tm.set_command(cmd)
    srv = socket.create_server(('127.0.0.1', args.port), reuse_port=False)
    print(f'[link_test] waiting for the plugin on 127.0.0.1:{args.port}, command omega {np.round(cmd, 1).tolist()} '
          f'(hover {w0:.1f} rad/s)', flush=True)
    conn, _ = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    rows, t0, released, wall0 = [], None, None, time.time()
    while True:
        d = IN.unpack(recv_exact(conn, IN.size))
        t, p, q, v, w, dt = d[0], np.array(d[1:4]), np.array(d[4:8]), np.array(d[8:11]), np.array(d[11:14]), d[14]
        R = quat_wxyz_to_R(q)
        force, torque = tm.step(1.0 / 120.0, R, v)      # one exchange = one Isaac Sim physics step (1/120 s)
        props = np.zeros(4) if args.no_prop_spin else prop_speeds(tm.forces, tm.dir)
        conn.sendall(OUT.pack(*force, *torque, *props))
        t0 = t if t0 is None else t0
        rows.append((t, time.time(), p, v, R, w))
        if released is None and t - t0 >= args.release_after:
            subprocess.run(['gz', 'topic', '-t', '/drone/detach', '-m', 'gz.msgs.Empty', '-p', ''], check=True)
            released = t
            print(f'[link_test] released at sim {t:.3f} s', flush=True)
        if released is not None and t - released >= args.duration:
            break
    conn.close()
    ts = np.array([r[0] for r in rows])
    wall = np.array([r[1] for r in rows])
    rate = (len(ts) - 1) / (ts[-1] - ts[0])
    rtf = (ts[-1] - ts[0]) / (wall[-1] - wall[0])
    after = [r for r in rows if r[0] >= released + 0.2]
    vz = np.array([r[3][2] for r in after])
    tt = np.array([r[0] for r in after])
    az = np.polyfit(tt, vz, 1)[0]
    roll = [np.degrees(np.arctan2(r[4][2, 1], r[4][2, 2])) for r in after]
    wx = [r[5] @ r[4][:, 0] for r in after]
    print(f'[link_test] exchanges {len(ts)}, rate {rate:.2f} Hz sim, RTF {rtf:.3f} (lockstep, incl. wait for release)')
    print(f'[link_test] after release ({released:.2f} + 0.2 .. {tt[-1]:.2f} s): vertical acceleration {az:+.3f} m/s^2, '
          f'z change {(after[-1][2][2] - after[0][2][2]) * 1000:+.1f} mm, roll {roll[0]:+.2f} -> {roll[-1]:+.2f} deg, '
          f'body roll rate end {np.degrees(wx[-1]):+.1f} deg/s')
    # first 0.1 s after release: body angular acceleration (body frame) vs the model torque / inertia (Euler terms ~0)
    first = [r for r in rows if released < r[0] <= released + 0.1]
    wb = np.array([r[4].T @ r[5] for r in first])
    tb = np.array([r[0] for r in first])
    alpha = np.array([np.polyfit(tb, wb[:, i], 1)[0] for i in range(3)])
    I = np.array(fl['inertia_about_com'])
    F = tm.k * (tm.omega ** 2)
    com = np.array(fl['com'])
    tau_b = sum(np.cross(rp - com, ax * f) for rp, ax, f in zip(tm.pos, tm.axis, F)) \
        + tm.c * (tm.omega ** 2 * tm.dir).sum() * np.array([0, 0, 1.0])
    pred = np.linalg.solve(I, tau_b)
    print(f'[link_test] angular acceleration first 0.1 s (body) {np.round(alpha, 2).tolist()} rad/s^2, '
          f'predicted tau/I {np.round(pred, 2).tolist()} rad/s^2 (tau about COM {np.round(tau_b * 1000, 2).tolist()} mN m)')
    print(f'[link_test] wrench at the end: thrust sum {tm.k * (tm.omega ** 2).sum():.3f} N (weight {fl["mass"] * 9.81:.3f} N)')


if __name__ == '__main__':
    main()
