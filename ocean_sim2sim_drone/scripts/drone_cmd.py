#!/usr/bin/env python3
# =============================================================
# drone_cmd.py - command the running PX4 drone from another terminal (plain python3, no ROS / pymavlink needed)
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/drone_cmd.py main() (CLI part). The Isaac Sim PX4 link is
#     based on livealive7/PegasusSimulator b1256ca (BSD-3-Clause, Copyright (c) 2023 Marcelo Jacinto,
#     ocean_sim2sim_drone/LICENSE.PegasusSimulator).
#   Sends one JSON command to the PX4Commander inside px4_bridge_node.py (UDP localhost command_port 14600 + instance)
#   and prints the JSON answer. Same JSON as Isaac: {"cmd": "status"|"goto"|"hold"|"land"|"kill", "pos": [x, y, z],
#   "yaw_deg": deg}. The commander keeps streaming the Offboard setpoint; the CLI only changes it.
#     drone_cmd.py status
#     drone_cmd.py goto 0.6 0.1 1.5 [--yaw-deg 0]     # hover mode: moves the sine reference only (sine continues)
#     drone_cmd.py hold | land | kill                 # hold in hover mode: freezes the sine at the current target
#   Move commands are refused until the scene started the reference (status "ready" true); kill is always accepted.
#
#   Changes from Isaac: default port from config/drone_px4.yaml found through ament_index when ROS is sourced
#     (else 14600), messages in English.
# =============================================================
import argparse
import json
import os
import socket
import sys

DEFAULT_PORT = 14600            # config/drone_px4.yaml command_port + instance


def default_port():
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory

        with open(os.path.join(get_package_share_directory("ocean_sim2sim_drone"), "config", "drone_px4.yaml"),
                  encoding="utf-8") as fh:
            c = yaml.safe_load(fh)
        return int(c["command_port"]) + int(c["instance"])
    except Exception:  # noqa: BLE001  (ROS not sourced: fixed default)
        return DEFAULT_PORT


def main():
    ap = argparse.ArgumentParser(description="Command the PX4 drone of the running sim (world frame, m)")
    ap.add_argument("--port", type=int, default=None, help=f"command UDP port (default drone_px4.yaml, {DEFAULT_PORT})")
    ap.add_argument("--timeout", type=float, default=3.0, help="reply wait [s, wall clock]")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("goto", help="position target (x y z, world m). hover mode: moves the sine reference only")
    g.add_argument("x", type=float)
    g.add_argument("y", type=float)
    g.add_argument("z", type=float)
    g.add_argument("--yaw-deg", type=float, default=None, help="yaw [deg, from world +x counter-clockwise] (default: keep)")
    for c, h in (("hold", "stop at the current estimate (hover mode: freeze the sine at the current target)"),
                 ("land", "land (AUTO.LAND)"), ("kill", "forced stop in the air (motors off at once)"),
                 ("status", "mode, armed, estimate, target")):
        sub.add_parser(c, help=h)
    a = ap.parse_args()
    port = default_port() if a.port is None else a.port
    req = {"cmd": a.cmd}
    if a.cmd == "goto":
        req["pos"] = [a.x, a.y, a.z]
        if a.yaw_deg is not None:
            req["yaw_deg"] = a.yaw_deg
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(a.timeout)
    sock.sendto(json.dumps(req).encode(), ("127.0.0.1", port))
    try:
        data, _ = sock.recvfrom(4096)
    except socket.timeout:
        print(f"no answer ({a.timeout} s): is px4_bridge_node.py running? (command port {port})", file=sys.stderr)
        return 2
    res = json.loads(data.decode())
    print(json.dumps(res, ensure_ascii=False, indent=1))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
