#!/usr/bin/env python3
# =============================================================
# policy_client_node.py - drive the Gazebo ocean sim with an openpi policy server (scripts/serve_policy.py). docs/policy.md
#
#   Ported from: data_collection_ur5_gripper policy/policy_client_node.py main() (Isaac Sim). Same CLI and episode
#   loop; the client itself is ocean_sim2sim_policy/client.py (what changed: see its header). Here: reset answer of the
#   Gazebo scene (drone_pos_base, drone_target name), --prompt, messages in English.
#
#   Takes the teleop node's place: reads /joint_states, /gripper/joint_states, the cameras of config cameras (Gazebo:
#   wrist only), /clock, /protective_stop (+ /base/imu if base_imu.enabled); sends /joint_command (60 Hz, stamp = sim
#   time) and /gripper/command (0 / 1150, on change).
#
# Run (the Gazebo stack and the policy server first; in a terminal with the venv that has openpi-client, README):
#   source /opt/ros/jazzy/setup.bash && source ~/vrx_ws/.venv/bin/activate && source ~/vrx_ws/install/setup.bash
#   ros2 run ocean_sim2sim_policy policy_client_node.py --reset --duration 30
#   --episodes N : reset -> run, N times.  --auto-kill S : S s after the close command, if the gripper is blocked by the
#   object (present < 0.9), drone motors off. Judged evaluation with reports: policy_eval.py
# =============================================================
import argparse
import os

from ament_index_python.packages import get_package_share_directory

from ocean_sim2sim_policy.client import PolicyClient, load_config


def default_config():
    return os.path.join(get_package_share_directory('ocean_sim2sim_policy'), 'config', 'policy_client.yaml')


def main():
    ap = argparse.ArgumentParser(description='Drive the Gazebo ocean sim with an openpi policy server')
    ap.add_argument('--config', default=None, help='policy_client.yaml (default: the installed one)')
    ap.add_argument('--host', default=None, help='policy server address (overrides config)')
    ap.add_argument('--port', type=int, default=None)
    ap.add_argument('--prompt', default=None, help='overrides config prompt')
    ap.add_argument('--duration', type=float, required=True, help='[sim s] episode length (data episodes 12 - 30 s)')
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--reset', action='store_true', help='/sim/reset before every episode')
    ap.add_argument('--auto-kill', type=float, default=None,
                    help='[sim s] after the close command, drone motors off if the gripper is blocked')
    args = ap.parse_args()
    if args.episodes > 1 and not args.reset:
        ap.error('--episodes 2 or more needs --reset (without it the next episode starts from the previous result)')
    cfg = load_config(args.config or default_config())
    if args.prompt is not None:
        cfg['prompt'] = args.prompt
    host = args.host or cfg['server']['host']
    port = args.port or int(cfg['server']['port'])

    import rclpy
    from openpi_client import websocket_client_policy
    print(f'[policy] connecting to the policy server {host}:{port} ...', flush=True)
    policy = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
    print(f'[policy] server metadata: {policy.get_server_metadata()}', flush=True)
    rclpy.init()
    pc = PolicyClient(cfg, policy)
    results = []
    try:
        pc.wait_ready()
        pc.warmup()
        for ep in range(args.episodes):
            if args.reset:
                print(f"[policy] episode {ep + 1}/{args.episodes}: {cfg['topics']['reset_service']} ...", flush=True)
                info = pc.reset()
                print(f"[policy] reset done: drone position (base) {info.get('drone_pos_base')}, target "
                      f"{info.get('drone_target')}, seed {info.get('seed')}", flush=True)
                t_reset = pc.now()
                pc.wait_sim_until(t_reset + 0.2)     # fresh observations after the reset
            results.append(pc.run_episode(args.duration, args.auto_kill))
    except KeyboardInterrupt:
        print('[policy] interrupted (Ctrl+C)', flush=True)
    finally:
        pc.shutdown()
        rclpy.try_shutdown()
    if len(results) > 1:
        print(f"[policy] summary: gripper blocked (grasp signal) "
              f"{sum(r['gripper_closed'] and r['gripper_present_end'] < 0.9 for r in results)}"
              f"/{len(results)}, stop reasons {[r['stop'] for r in results]}", flush=True)


if __name__ == '__main__':
    main()
