#!/usr/bin/env python3
# =============================================================
# spawn_controllers.py - load, configure and activate ros2_control controllers without the 60 s stall
#
#   Replaces `ros2 run controller_manager spawner` in wamv_arm.launch.py. Why: the stock spawner uses ONE call timeout
#   for every service call and retries each call blindly.
#     - In about 12 % of start-ups (27 of 234 runs, 2026-11-03 / 04) the controller manager inside Gazebo logs
#       "failed to send response to /controller_manager/list_controllers (timeout)": the first answer to a new client
#       is lost (DDS discovery race: the request arrives before the server has matched the client's reply reader).
#       The spawner then waits the whole call timeout (60 s; 10 s before 2026-11-03) and only then asks again.
#     - A short timeout is not possible with the stock spawner: at a busy start-up load_controller answered later than
#       10 s, the spawner sent load again and failed with "already loaded" (controllers stayed inactive).
#   Here the calls are split by kind:
#     - list_controllers (read only, safe to repeat): short timeout (--list-timeout, 3 s), asked again until answered
#     - load / configure / switch (change state): sent ONCE with a long timeout (--change-timeout, 60 s). No answer in
#       time -> the controller list is read again and the wanted state is checked (no blind repeat, no "already loaded")
#   Per controller: not loaded -> load; unconfigured -> configure; then all inactive ones are activated together
#   (strict). Already active controllers are left as they are. Exit 0 = all active, 1 = error (logged).
#
#   Usage: spawn_controllers.py <controller> [...] [--controller-manager /controller_manager] [--wait 120]
#          [--list-timeout 3] [--change-timeout 60]
# =============================================================
import argparse
import sys
import time

import rclpy
from controller_manager.controller_manager_services import service_caller
from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController


class Spawner:
    def __init__(self, node, cm, list_timeout, change_timeout):
        self.node, self.cm = node, cm.rstrip('/')
        self.list_timeout, self.change_timeout = list_timeout, change_timeout
        self.log = node.get_logger()

    def states(self, deadline):
        """{name: state} from list_controllers; asked again every list_timeout until deadline (read only, safe)."""
        n = 0
        while True:
            n += 1
            try:
                res = service_caller(self.node, f'{self.cm}/list_controllers', ListControllers, ListControllers.Request(),
                                     call_timeout=self.list_timeout, max_attempts=1)
                if n > 1:
                    self.log.info(f'list_controllers answered at try {n} (lost answers before: DDS discovery)')
                return {c.name: c.state for c in res.controller}
            except RuntimeError:
                if time.monotonic() > deadline:
                    raise RuntimeError(f'{self.cm}/list_controllers: no answer in {n} tries')

    def change(self, what, srv, req, done, deadline):
        """State-changing call sent once; no answer in change_timeout -> read the list and check done(states)."""
        try:
            res = service_caller(self.node, f'{self.cm}/{what}', srv, req, call_timeout=self.change_timeout, max_attempts=1)
            if not res.ok:
                raise RuntimeError(f'{what} refused by the controller manager')
            return
        except RuntimeError as e:
            if 'refused' in str(e):
                raise
        st = self.states(deadline)
        if not done(st):
            raise RuntimeError(f'{what}: no answer in {self.change_timeout:g} s and the controller list does not show it done: {st}')
        self.log.warn(f'{what}: answer lost, but the controller list shows it done')

    def run(self, names, wait):
        deadline = time.monotonic() + wait
        cli = self.node.create_client(ListControllers, f'{self.cm}/list_controllers')
        if not cli.wait_for_service(timeout_sec=wait):
            raise RuntimeError(f'{self.cm} not available within {wait:g} s')
        self.node.destroy_client(cli)
        t0 = time.monotonic()
        st = self.states(deadline)
        for name in names:
            if name not in st:
                req = LoadController.Request(name=name)
                self.change('load_controller', LoadController, req, lambda s, n=name: n in s, deadline)
                self.log.info(f'Loaded {name}')
                st = self.states(deadline)
            if st.get(name) == 'unconfigured':
                req = ConfigureController.Request(name=name)
                self.change('configure_controller', ConfigureController, req,
                            lambda s, n=name: s.get(n) in ('inactive', 'active'), deadline)
                st = self.states(deadline)
        todo = [n for n in names if st.get(n) == 'inactive']
        bad = [f'{n}: {st.get(n)}' for n in names if st.get(n) not in ('inactive', 'active')]
        if bad:
            raise RuntimeError(f'controllers not ready to activate: {bad}')
        if todo:
            req = SwitchController.Request(activate_controllers=todo, deactivate_controllers=[],
                                           strictness=SwitchController.Request.STRICT, activate_asap=True)
            req.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
            self.change('switch_controller', SwitchController, req,
                        lambda s: all(s.get(n) == 'active' for n in todo), deadline)
        st = self.states(deadline)
        not_active = [n for n in names if st.get(n) != 'active']
        if not_active:
            raise RuntimeError(f'not active after the switch: {[(n, st.get(n)) for n in not_active]}')
        for n in names:
            self.log.info(f'Configured and activated {n}')
        self.log.info(f'all {len(names)} controllers active in {time.monotonic() - t0:.1f} s after the controller manager appeared')


def main():
    ap = argparse.ArgumentParser(description='load + configure + activate controllers (no 60 s stall on a lost answer)')
    ap.add_argument('controllers', nargs='+')
    ap.add_argument('--controller-manager', default='/controller_manager')
    ap.add_argument('--wait', type=float, default=120.0, help='[s] for the controller manager and the whole spawn')
    ap.add_argument('--list-timeout', type=float, default=3.0, help='[s] per list_controllers try (repeated)')
    ap.add_argument('--change-timeout', type=float, default=60.0, help='[s] per load / configure / switch (sent once)')
    a, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = rclpy.create_node('spawn_controllers')
    code = 0
    try:
        Spawner(node, a.controller_manager, a.list_timeout, a.change_timeout).run(a.controllers, a.wait)
    except RuntimeError as e:
        node.get_logger().fatal(str(e))
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
