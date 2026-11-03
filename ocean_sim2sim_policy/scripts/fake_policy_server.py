#!/usr/bin/env python3
"""Fake openpi policy server for plumbing tests (no model, no GPU). docs/policy.md

Speaks the openpi websocket protocol (openpi src/openpi/serving/websocket_policy_server.py: metadata on connect, then
msgpack-numpy obs -> {"actions": (H, 7), "server_timing": {"infer_ms"}}) and answers every observation with a constant
(horizon, 7) chunk:
  hold (default)  arm = observation/state[0:6] of the request (the arm stays where it is), gripper = --gripper
  --constant q1..q6 g   the same 7 values every step
The first request's keys / shapes / dtypes are printed (check what the client sends). --delay simulates inference time.

    ros2 run ocean_sim2sim_policy fake_policy_server.py --port 8000 [--delay 0.18]
"""
import argparse
import time

import numpy as np
import websockets.sync.server
from openpi_client import msgpack_numpy


def describe(obs):
    out = {}
    for k, v in obs.items():
        a = np.asarray(v)
        out[k] = repr(v) if a.dtype.kind in 'US' else f'{a.dtype}{list(a.shape)}'
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--horizon', type=int, default=10)
    ap.add_argument('--delay', type=float, default=0.0, help='[s] wait before answering (inference time)')
    ap.add_argument('--gripper', type=float, default=0.0, help='gripper output in hold mode (> 0.5 = close)')
    ap.add_argument('--constant', type=float, nargs=7, default=None, metavar='V')
    ap.add_argument('--require', nargs='*', default=[], help='observation keys that must be present (error otherwise)')
    args = ap.parse_args()
    packer = msgpack_numpy.Packer()
    count = {'n': 0}

    def handler(ws):
        print(f'[fake_policy] connection from {ws.remote_address}', flush=True)
        ws.send(packer.pack({'fake': True, 'horizon': args.horizon}))
        while True:
            try:
                obs = msgpack_numpy.unpackb(ws.recv())
            except websockets.exceptions.ConnectionClosed:
                print('[fake_policy] connection closed', flush=True)
                return
            t0 = time.monotonic()
            count['n'] += 1
            if count['n'] == 1:
                print(f'[fake_policy] first observation: {describe(obs)}', flush=True)
            missing = [k for k in args.require if k not in obs]
            if missing:
                ws.send(f'fake_policy: observation keys missing: {missing} (got {sorted(obs)})')
                continue
            if args.constant is not None:
                row = np.asarray(args.constant, dtype=float)
            else:
                state = np.asarray(obs['observation/state'], dtype=float)
                row = np.concatenate([state[:6], [args.gripper]])
            if args.delay > 0:
                time.sleep(args.delay)
            ws.send(packer.pack({'actions': np.tile(row, (args.horizon, 1)),
                                 'server_timing': {'infer_ms': (time.monotonic() - t0) * 1000}}))

    with websockets.sync.server.serve(handler, args.host, args.port, compression=None, max_size=None) as server:
        print(f'[fake_policy] listening on {args.host}:{args.port}', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
