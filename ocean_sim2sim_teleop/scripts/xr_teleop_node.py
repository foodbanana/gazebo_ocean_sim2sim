#!/usr/bin/env python3
# =============================================================
# xr_teleop_node.py - PICO 4 Ultra arm teleoperation node (ocean_sim2sim_teleop/xr_teleop.py, docs/teleop.md)
#
#   Ported from: data_collection_ur5_gripper teleop/xr_teleop_node.py main().
#   Needs the XRoboToolkit Python binding (xrobotoolkit_sdk), built into its own venv (Isaac Sim repo
#   teleop/build_xr_binding.sh -> ~/xrobotoolkit/venv, --system-site-packages so ROS 2 rclpy is visible). If the
#   current python cannot import it, the node re-runs itself with $XR_PYTHON or ~/xrobotoolkit/venv/bin/python.
#   Run (sim running, PC service running, headset app WORKING + Send, start before the recorder):
#       ros2 run ocean_sim2sim_teleop xr_teleop_node.py [--config <yaml>] [--stamp sim|wall]
#   Test without the headset: ros2 run ocean_sim2sim_teleop check_xr_teleop.py
#
#   Changes from the Isaac Sim file: URDF from /robot_description instead of a file, re-run with the venv python,
#     messages in English.
# =============================================================
import argparse
import os
import sys

REEXEC_ENV = "OCEAN_SIM2SIM_XR_REEXEC"


def _ensure_binding():
    try:
        import xrobotoolkit_sdk  # noqa: F401
        return
    except ImportError:
        pass
    py = os.environ.get("XR_PYTHON") or os.path.expanduser("~/xrobotoolkit/venv/bin/python")
    if os.environ.get(REEXEC_ENV) or not os.path.isfile(py):
        sys.exit(f"[ERROR] cannot import xrobotoolkit_sdk with {sys.executable} and no binding python at {py} "
                 "(set XR_PYTHON, docs/teleop.md)")
    os.environ[REEXEC_ENV] = "1"
    os.execv(py, [py, os.path.abspath(__file__)] + sys.argv[1:])


def main():
    from ocean_sim2sim_teleop import xr_teleop as tn

    ap = argparse.ArgumentParser(description="PICO 4 Ultra controller arm teleoperation node")
    ap.add_argument("--config", default=tn.default_config())
    ap.add_argument("--stamp", choices=("sim", "wall"), default=None, help="/joint_command stamp (overrides the config)")
    args = ap.parse_args()
    cfg = tn.load_config(args.config)
    if args.stamp:
        cfg["stamp"] = args.stamp
    _ensure_binding()

    import rclpy
    try:
        inp = tn.XrInput()
    except RuntimeError as e:
        sys.exit(f"[ERROR] {e}")
    try:
        tn.wait_xr(inp, float(cfg["xr_connect_timeout"]))
    except RuntimeError as e:
        inp.close()                              # exiting without closing the SDK thread aborts the process
        sys.exit(f"\n[ERROR] {e}")
    rclpy.init()
    try:
        urdf = tn.wait_robot_description(cfg["topics"]["robot_description"])
    except RuntimeError as e:
        inp.close()
        rclpy.shutdown()
        sys.exit(f"[ERROR] {e}")
    left = [f"{r} ({b})" for r, b in [("teleop", cfg["teleop_button"])] + list(cfg["buttons"].items()) if b in ("X", "Y")]
    if left and not inp.left_ok():
        print(f"\a[xr_teleop] warning: no left controller data -> left buttons {', '.join(left)} do not work. Select the "
              "left controller too in the headset app, Send, then restart the node", flush=True)
    node = tn.XrTeleop(cfg, inp, urdf)
    fc = cfg["filter"]
    filt = (f"One Euro filter (position {fc['pos_min_cutoff_hz']:g} Hz + {fc['pos_beta']:g} x speed, orientation "
            f"{fc['rot_min_cutoff_hz']:g} Hz + {fc['rot_beta']:g} x angular speed)" if fc["enabled"] else "filter off")
    print(f"[xr_teleop] ready: {cfg['rate_hz']:g} Hz, stamp {cfg['stamp']}, scale {cfg['scale']:g}, rotation "
          f"{'followed' if cfg['rotation'] else 'off'}, {filt}. Hold the right controller pointing forward in the same start "
          f"pose and press {cfg['teleop_button']} -> teleop start / {cfg['teleop_button']} again -> stop. Trigger = gripper. "
          f"Recorder requests: {', '.join(f'{b} = {r}' for r, b in cfg['buttons'].items()) or 'none'}. Ctrl+C to quit",
          flush=True)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        inp.close()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\n[xr_teleop] quit ({node.n_sent} commands sent)", flush=True)


if __name__ == "__main__":
    main()
