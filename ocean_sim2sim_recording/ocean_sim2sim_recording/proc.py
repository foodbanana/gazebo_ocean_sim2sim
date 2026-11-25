"""Process helpers: start / stop whole process groups, find leftover simulation processes.

Same rules as ocean_sim2sim_bringup scripts/regression.py (anchored argv matching, so the shell that runs a script is
never matched; SIGINT -> SIGTERM -> SIGKILL per process group).
"""
import os
import re
import signal
import subprocess
import time


def read_argv(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return []


def own_ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def is_sim_process(argv):
    if not argv:
        return False
    a0, a1 = argv[0], argv[1] if len(argv) > 1 else ''
    base = os.path.basename(a0)
    if ' '.join(argv).startswith('gz sim'):
        return True
    if base == 'ruby' and 'gz_tools_vendor' in a1:
        return True
    if a0.endswith('/ros_gz_bridge/parameter_bridge') or a0.endswith('/robot_state_publisher') or a0.endswith('/bin/px4'):
        return True
    if base.startswith('python3') and re.search(
            r'/(px4_bridge_node(\.py)?|interface_node|scene_node|spawner|spawn_controllers\.py|arm_motion_node\.py)$', a1):
        return True
    return False


def gazebo_servers():
    return [int(p) for p in os.listdir('/proc') if p.isdigit() and ' '.join(read_argv(p)).startswith('gz sim')]


def sim_processes():
    skip = own_ancestors()
    return [int(p) for p in os.listdir('/proc') if p.isdigit() and int(p) not in skip and is_sim_process(read_argv(p))]


def group_alive(pgid):
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                if os.getpgid(int(p)) == pgid:
                    return True
            except (ProcessLookupError, PermissionError):
                pass
    return False


def kill_group(proc, first=signal.SIGINT, timeout=20.0):
    """Stop a process started with start_new_session=True: its whole process group."""
    if proc is None:
        return
    for sig, wait in ((first, timeout), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=wait)
            time.sleep(0.5)
            if not group_alive(proc.pid):
                break
        except subprocess.TimeoutExpired:
            continue


def sweep(log):
    """Stop leftover simulation processes: SIGINT, then SIGTERM, then SIGKILL. Returns those still running."""
    for sig, wait in ((signal.SIGINT, 5.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        left = sim_processes()
        if not left:
            return []
        log(f'  sweep {sig.name}: {[(p, " ".join(read_argv(p))[:80]) for p in left]}')
        for p in left:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        t0 = time.time()
        while time.time() - t0 < wait and sim_processes():
            time.sleep(0.2)
    return sim_processes()
