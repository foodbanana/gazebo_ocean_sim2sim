#!/usr/bin/env python3
# =============================================================
# record_toggle.py - rosbag2 episode recorder for the Gazebo ocean sim (docs/teleop.md)
#
#   Ported from: data_collection_ur5_gripper record_toggle.py (RosLink, write_qos_overrides, BagRecorder,
#     reset_then_record, request_record, request_reset, key loop) in its --sim --base moving mode. Same keys, same
#     controller requests, same episode.json, same bag layout (one bag = one episode).
#   Keys (focus on this terminal):
#     r (idle)      : /sim/reset, then record <YYYYMMDD_HHMMSS>_<task name>
#     r (recording) : stop the bag safely (SIGINT, wait for metadata.yaml) -> idle
#     k             : drone motors off (/sim/drone_kill; time in episode.json while recording)
#     d (idle)      : move the episode just saved to <bag dir>/_discarded/ (confirm with y; nothing is deleted)
#     q             : quit (stops a running recording safely first)
#   Controller buttons (xr_teleop_node.py, /teleop/request std_msgs/String, not recorded):
#     record (A)     : start / stop without a reset; start only after a reset that has not been recorded yet
#     reset (B)      : episode reset; while recording = failed episode, stopped and moved to _discarded/, then reset
#     drone_kill (X) : = k
#     Requests that arrive during a reset / stop / kill are dropped.
#   Topics: config/recorder.yaml (state topics need a publisher before recording; image topics RELIABLE).
#     /base/imu and /base/odom are recorded (boat motion). Plot the IMU of a bag: plot_bag_imu.py <bag>
#   Bags: --bag-dir (default ~/ocean_sim2sim_bags), never inside the repo. Not deleted automatically (about 1.5 GB per
#     30 s with the wrist camera in Isaac; check df -h).
#
#   Run (sim and teleop node running):
#       ros2 run ocean_sim2sim_teleop record_toggle.py <task name> [--bag-dir DIR]
#   Do not start new ROS nodes (ros2 topic echo, rqt) while recording: discovery can stall delivery for a moment.
#
#   Changes from the Isaac Sim file: sim only (Gazebo), moving base always, topics from config/recorder.yaml (wrist
#     camera only), bag directory option, Gazebo reset / kill replies (drone_world, sim_s, sim_time), messages in English.
# =============================================================

import argparse
import json
import os
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import time
import tty
from datetime import datetime

import yaml

SIM_RESET_SERVICE = '/sim/reset'
SIM_KILL_SERVICE = '/sim/drone_kill'
TELEOP_REQUEST_TOPIC = '/teleop/request'        # = xr_teleop.yaml topics.request
TELEOP_REQUESTS = ('record', 'reset', 'drone_kill')
STOP_WARN_SEC = 10.0

IDLE_MSG = "[IDLE] idle. 'r' reset + record / 'k' drone off / 'd' discard last episode / 'q' quit"


def say(*args, **kwargs):
    # keep the safe bag stop working after the terminal closed (SIGHUP): output errors are ignored
    try:
        print(*args, **kwargs)
    except OSError:
        pass


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ocean_sim2sim_teleop'), 'config', 'recorder.yaml')


def load_config(path):
    with open(path, encoding='utf-8') as f:
        c = yaml.safe_load(f)
    for k in ('state_topics', 'command_topics', 'cameras', 'queue_sec', 'topic_hz', 'reset_timeout'):
        if k not in c:
            raise KeyError(f"{path}: missing '{k}'")
    for role, r in c['cameras'].items():
        for k in ('topic', 'info_topic'):
            if k not in r:
                raise KeyError(f"{path}: missing 'cameras.{role}.{k}'")
    c['path'] = os.path.abspath(path)
    return c


class RosLink:
    """Topic checks before recording and the sim service calls (rclpy). The recording itself is a ros2 bag record process."""

    def __init__(self, cfg):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.rclpy, self.cfg = rclpy, cfg
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # this script handles the signals (safe bag stop)
        self.node = rclpy.create_node('record_toggle')
        self.reset_cli = self.node.create_client(Trigger, SIM_RESET_SERVICE)
        self.kill_cli = self.node.create_client(Trigger, SIM_KILL_SERVICE)
        self.Trigger = Trigger
        self.requests = []
        self.node.create_subscription(String, TELEOP_REQUEST_TOPIC, lambda m: self.requests.append(m.data), 10)
        self.image_topics = [r['topic'] for r in cfg['cameras'].values()]
        self.info_topics = [r['info_topic'] for r in cfg['cameras'].values()]

    @property
    def state_topics(self):
        return list(self.cfg['state_topics']) + self.image_topics + self.info_topics

    @property
    def topics(self):
        return self.state_topics + list(self.cfg['command_topics'])

    def spin(self, sec):
        self.rclpy.spin_once(self.node, timeout_sec=sec)

    def pop_request(self):
        self.spin(0)
        return self.requests.pop(0) if self.requests else None

    def drop_requests(self, sec=0.3):
        """Drop requests that arrived during a long action (like tcflush for keys); also those still in DDS."""
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            self.spin(0.02)
        if self.requests:
            say(f"\n[TELEOP] dropped {len(self.requests)} controller requests that came while busy: {', '.join(self.requests)}")
            self.requests.clear()

    def check_topics(self, wait=2.0):
        """Every state topic has a publisher, camera publishers are RELIABLE. -> list of problems (empty = ok)"""
        from rclpy.qos import ReliabilityPolicy

        t_end = time.monotonic() + wait          # discovery may not be complete right after the node started
        while True:
            problems = []
            for t in self.state_topics:
                pubs = self.node.get_publishers_info_by_topic(t)
                if not pubs:
                    problems.append(f"{t}: no publisher")
                elif t in self.image_topics and any(p.qos_profile.reliability != ReliabilityPolicy.RELIABLE for p in pubs):
                    problems.append(f"{t}: publisher not RELIABLE (best effort drops whole images)")
            if not problems or time.monotonic() > t_end:
                return problems
            self.spin(0.1)

    def _call(self, cli, name, timeout, on_tick=None):
        """Call a Trigger service and wait. -> (success, reply JSON dict or error text)"""
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, f"{name} not available (is the sim running?)"
        fut = cli.call_async(self.Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            self.spin(0.1)
            el = time.monotonic() - t0
            if on_tick is not None:
                on_tick(el)
            if el > timeout:
                return False, f"{name}: no answer within {timeout:g} s"
        res = fut.result()
        try:
            info = json.loads(res.message)
        except json.JSONDecodeError:
            return False, f"{name}: reply is not JSON: {res.message!r}"
        if not res.success:
            return False, f"{name} failed: {info.get('error', res.message)}"
        return True, info

    def sim_reset(self, on_tick=None):
        return self._call(self.reset_cli, SIM_RESET_SERVICE, float(self.cfg['reset_timeout']), on_tick)

    def drone_kill(self):
        return self._call(self.kill_cli, SIM_KILL_SERVICE, 5.0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def write_qos_overrides(link):
    """QoS override file of ros2 bag record (state topics). -> path
    - queue depth: queue_sec worth of messages (no overflow while the recorder starts)
    - camera images RELIABLE (check_topics checks the publisher too). Others: the publisher reliability
      (RELIABLE subscription to a best effort publisher = no connection = silently empty topic)"""
    from rclpy.qos import ReliabilityPolicy

    f = tempfile.NamedTemporaryFile('w', prefix='record_toggle_qos_', suffix='.yaml', delete=False)
    for t in link.state_topics:
        pubs = link.node.get_publishers_info_by_topic(t)
        reliable = t in link.image_topics or (bool(pubs) and all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs))
        depth = int(round(float(link.cfg['queue_sec']) * float(link.cfg['topic_hz'].get(t, 30.0))))
        f.write(f"{t}:\n  history: keep_last\n  depth: {depth}\n  reliability: {'reliable' if reliable else 'best_effort'}\n  durability: volatile\n")
    f.close()
    return f.name


class BagRecorder:
    def __init__(self, task_name, link, bag_dir):
        self.task_name, self.link, self.bag_dir = task_name, link, bag_dir
        self.discard_dir = os.path.join(bag_dir, '_discarded')
        self.qos_path = None
        self.proc = None
        self.out_dir = None
        self.t_start = None
        self.info = None
        self.episode_count = 0
        self.last_saved = None       # episode just saved in this session ('d' target)
        self.last_elapsed = None

    @property
    def recording(self):
        return self.proc is not None

    def start(self, reset_info=None):
        """Start recording. Topic problems -> not started, False. reset_info: /sim/reset reply (into episode.json)"""
        problems = self.link.check_topics()
        if problems:
            say("\n[ERROR] not recording - topic problems:")
            for p in problems:
                say(f"          {p}")
            say(IDLE_MSG)
            return False
        self._remove_qos_file()
        self.qos_path = write_qos_overrides(self.link)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        ep_name = f"{stamp}_{self.task_name}" if self.task_name else stamp
        self.out_dir = os.path.join(self.bag_dir, ep_name)
        self.info = {
            'episode': ep_name, 'mode': 'sim', 'simulator': 'gazebo', 'base': 'moving', 'task_name': self.task_name,
            'recorded_at': datetime.now().isoformat(timespec='seconds'),
            'topics': self.link.topics, 'camera_reliability': 'reliable', 'queue_sec': float(self.link.cfg['queue_sec']),
            'cameras': {role: {'topic': r['topic']} for role, r in self.link.cfg['cameras'].items()},
            'recorder_config': self.link.cfg['path'],
            'sim_reset': reset_info,
            'drone_kill': [],            # drone motors off while recording (kill replies, sim time)
        }
        cmd = ['ros2', 'bag', 'record', '-o', self.out_dir, '--disable-keyboard-controls',
               '--qos-profile-overrides-path', self.qos_path, '--topics', *self.link.topics]
        # own session: Ctrl+C of the terminal does not reach the child; SIGINT goes exactly once (ours)
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, start_new_session=True)
        self.t_start = time.monotonic()
        say(f"\a\n[REC *] recording: {self.out_dir}")
        say("        'r' to stop")
        return True

    def stop(self):
        """Safe stop with SIGINT. Waits for the buffer flush and metadata.yaml (never SIGKILL)."""
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        say("\n[REC #] stopping... (waiting for the bag to finish)")
        while True:
            try:
                self.proc.wait(timeout=STOP_WARN_SEC)
                break
            except subprocess.TimeoutExpired:
                say("        still finishing... (not killed, to keep the mcap intact)")
            except KeyboardInterrupt:
                pass
        self._finish()

    def check_alive(self):
        if self.proc is not None and self.proc.poll() is not None:
            say(f"\n[ERROR] ros2 bag record exited unexpectedly (exit code {self.proc.returncode})")
            self._finish()
            return False
        return True

    def _finish(self):
        elapsed = time.monotonic() - self.t_start
        if os.path.isfile(os.path.join(self.out_dir, 'metadata.yaml')):
            self.info['wall_duration_s'] = round(elapsed, 2)
            with open(os.path.join(self.out_dir, 'episode.json'), 'w') as f:
                json.dump(self.info, f, ensure_ascii=False, indent=2)
            self.episode_count += 1
            self.last_saved = self.out_dir
            self.last_elapsed = elapsed
            say(f"[SAVED] {self.out_dir}  ({elapsed:.1f} s)  - episode {self.episode_count} of this session")
        else:
            say(f"[WARN] no metadata.yaml -> the bag may not have finished properly: {self.out_dir}")
        self.proc = None
        self.out_dir = None
        self.t_start = None
        self.info = None
        say(IDLE_MSG)

    def discard_last(self):
        os.makedirs(self.discard_dir, exist_ok=True)
        dst = os.path.join(self.discard_dir, os.path.basename(self.last_saved))
        try:
            os.rename(self.last_saved, dst)
        except OSError as e:
            say(f"[ERROR] move failed: {e}")
            return
        self.episode_count -= 1
        self.last_saved = None
        self.last_elapsed = None
        say(f"[DISCARDED] -> {dst}")
        say(f"            (to restore, move the folder back to {self.bag_dir}/) - {self.episode_count} episodes this session")

    def kill_drone(self):
        ok, info = self.link.drone_kill()
        if not ok:
            say(f"\n[ERROR] {info}")
            return
        say(f"\n[KILL] drone motors off (sim t {info.get('sim_time')} s). On again with the next reset")
        if self.recording:
            self.info['drone_kill'].append(info)

    def _remove_qos_file(self):
        if self.qos_path is not None:
            try:
                os.unlink(self.qos_path)
            except OSError:
                pass
            self.qos_path = None

    def close(self):
        self._remove_qos_file()


def _reset_done(info):
    return (f"seed {info.get('seed')}, drone {info.get('drone_target')} base {info.get('drone_pos_base')}, "
            f"{info.get('sim_s')} s sim / {info.get('wall_s')} s wall")


def reset_then_record(recorder, link):
    say(f"\n[RESET] {SIM_RESET_SERVICE} (arm to the start pose, gripper open, drone air start). Recording starts right after")
    ok, info = link.sim_reset(on_tick=lambda el: say(f"\r[RESET] {el:5.1f} s ", end='', flush=True))
    if not ok:
        say(f"\n[ERROR] {info}")
        say("        not recording. Check the sim terminal and press 'r' again")
        say(IDLE_MSG)
        return False
    say(f"\n[RESET] done: {_reset_done(info)}")
    return recorder.start(reset_info=info)


def request_record(recorder, link, pending):
    """Controller record: stop if recording, else start without a reset - only after a reset not recorded yet."""
    if recorder.recording:
        recorder.stop()
        return
    if pending['reset'] is None:
        say("\n[TELEOP] record refused: start only after a reset (no reset since the last recording, or it failed). "
            "Controller B (reset) first")
        say(IDLE_MSG)
        return
    if recorder.start(reset_info=pending['reset']):
        pending['reset'] = None              # one reset = one episode


def request_reset(recorder, link, pending):
    """Controller reset: while recording = failed episode (stop, move to _discarded/), then /sim/reset."""
    if recorder.recording:
        say("\n[TELEOP] reset while recording -> this episode failed: stop and move to _discarded/")
        before = recorder.last_saved
        recorder.stop()
        if recorder.last_saved is not None and recorder.last_saved != before:
            recorder.discard_last()
    pending['reset'] = None
    say(f"\n[RESET] {SIM_RESET_SERVICE} (arm to the start pose, gripper open, drone air start). Not recording")
    ok, info = link.sim_reset(on_tick=lambda el: say(f"\r[RESET] {el:5.1f} s ", end='', flush=True))
    if not ok:
        say(f"\n[ERROR] {info}")
        say("        check the sim terminal and reset again (B or r)")
    else:
        pending['reset'] = info
        say(f"\n[RESET] done: {_reset_done(info)}. Teleop start (Y) -> hold still -> A to record")
    say(IDLE_MSG)


def print_help(task_name, link, bag_dir):
    say()
    say("=== rosbag2 episode recorder (Gazebo) ===")
    say(f"  task name : {task_name if task_name else '(none)'}")
    say(f"  bags      : {bag_dir}/<YYYYMMDD_HHMMSS>{'_' + task_name if task_name else ''}/")
    say(f"  topics    : {' '.join(link.topics)}")
    say("  'r' : reset + record / stop (toggle)")
    say("  'k' : drone motors off (after the grasp)")
    say("  'd' : move the episode just saved to _discarded/ (confirm with 'y')")
    say("  'q' : quit (stops a running recording safely first)")
    say(f"  controller buttons ({TELEOP_REQUEST_TOPIC}, xr_teleop_node.py): A = record start / stop (no reset, only after "
        "a reset), B = reset (while recording: failed, to _discarded/), X = drone motors off")
    say("  controller order: B reset -> Y teleop on -> hold -> A record -> grasp -> X motors off -> A stop -> B ...")
    say("=========================================")
    say("(keep the focus on this terminal for the keys)")
    say(IDLE_MSG)


def read_key_nonblocking():
    # os.read directly (sys.stdin.read buffers ahead, tcflush could not drop it); first character only (rest = key repeat)
    fd = sys.stdin.fileno()
    dr, _, _ = select.select([fd], [], [], 0)
    if dr:
        data = os.read(fd, 64).decode('utf-8', errors='ignore')
        if data:
            return data[0]
    return None


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description="rosbag2 episode recorder (r reset + record / stop, k drone off, d discard, q quit)")
    ap.add_argument('task_name', nargs='?', default='', help='appended to the bag name (same settings = same task name)')
    ap.add_argument('--bag-dir', default='~/ocean_sim2sim_bags', help='where the bags go (default ~/ocean_sim2sim_bags)')
    ap.add_argument('--config', default=None, help='topics (default: this package config/recorder.yaml)')
    args = ap.parse_args()
    task_name = args.task_name
    bag_dir = os.path.abspath(os.path.expanduser(args.bag_dir))

    if shutil.which('ros2') is None:
        say("[ERROR] ros2 not found: source ROS 2 and the workspace")
        sys.exit(1)
    if not sys.stdin.isatty():
        say("[ERROR] run it directly in a terminal (keys are read from it)")
        sys.exit(1)

    os.makedirs(bag_dir, exist_ok=True)
    link = RosLink(load_config(args.config or default_config()))
    recorder = BagRecorder(task_name, link, bag_dir)
    print_help(task_name, link, bag_dir)

    # terminal closed (SIGHUP) / kill (SIGTERM): stop the bag safely too
    signal.signal(signal.SIGTERM, _raise_interrupt)
    signal.signal(signal.SIGHUP, _raise_interrupt)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    last_status = 0.0
    confirm_discard = False
    pending = {'reset': None}  # reset reply of controller B not recorded yet (controller A records with it)

    try:
        tty.setcbreak(fd)
        while True:
            ch = read_key_nonblocking()
            if ch is None:
                req = link.pop_request()
                if req is not None:
                    if req not in TELEOP_REQUESTS:
                        say(f"\n[TELEOP] unknown controller request: {req!r} (ignored)")
                    elif confirm_discard:
                        say(f"\n[TELEOP] waiting for the discard confirmation ('y'): controller request {req} ignored")
                    else:
                        say(f"\n[TELEOP] controller request: {req}")
                        if req == 'drone_kill':
                            ch = 'k'
                        else:
                            (request_record if req == 'record' else request_reset)(recorder, link, pending)
                            termios.tcflush(fd, termios.TCIFLUSH)
                            link.drop_requests()
                            last_status = time.monotonic()
            if confirm_discard and ch is not None:
                confirm_discard = False          # the key after 'd': y = move, anything else = cancel (no other action)
                if ch in ('y', 'Y', 'ㅛ'):
                    recorder.discard_last()
                else:
                    say("[CANCEL] discard cancelled - episode kept")
                say(IDLE_MSG)
                termios.tcflush(fd, termios.TCIFLUSH)
            elif ch in ('q', 'Q', 'ㅂ'):           # Korean keyboard layout: r / q / d / y / k = ㄱ / ㅂ / ㅇ / ㅛ / ㅏ
                break
            elif ch in ('d', 'D', 'ㅇ'):
                if recorder.recording:
                    say("\n[INFO] cannot discard while recording: stop with 'r' first")
                elif recorder.last_saved is None:
                    say("\n[INFO] nothing to discard (only the episode just saved in this session)")
                else:
                    confirm_discard = True
                    say(f"\n[DISCARD?] 'y' moves {os.path.basename(recorder.last_saved)} ({recorder.last_elapsed:.1f} s) "
                        "to _discarded/ (other key = cancel)")
                termios.tcflush(fd, termios.TCIFLUSH)
            elif ch in ('r', 'R', 'ㄱ'):
                if recorder.recording:
                    recorder.stop()
                else:
                    pending['reset'] = None        # r records with its own reset
                    reset_then_record(recorder, link)
                termios.tcflush(fd, termios.TCIFLUSH)    # drop key repeats / keys and requests during the stop or reset
                link.drop_requests()
                last_status = time.monotonic()
            elif ch in ('k', 'K', 'ㅏ'):
                recorder.kill_drone()
                termios.tcflush(fd, termios.TCIFLUSH)
                link.drop_requests()
            elif ch is not None:
                say(f"\nunsupported key: {repr(ch)}")

            if recorder.recording and recorder.check_alive():
                now = time.monotonic()
                if now - last_status >= 1.0:
                    last_status = now
                    sec = int(now - recorder.t_start)
                    say(f"\r[REC *] {sec // 60:02d}:{sec % 60:02d} ", end='', flush=True)
            time.sleep(1.0 / 100.0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        if recorder.recording:
            recorder.stop()
        recorder.close()
        link.close()
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        except termios.error:
            pass
        say("\n[EXIT] quit")


if __name__ == '__main__':
    main()
