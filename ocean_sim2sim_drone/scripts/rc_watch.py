#!/usr/bin/env python3
"""Show the transmitter serial values and mapped sticks (no sim). docs/teleop.md.
   ros2 run ocean_sim2sim_drone rc_watch.py [--port /dev/ttyACM0] [--config .../rc_input.yaml]"""
import argparse
import os

from ament_index_python.packages import get_package_share_directory

from ocean_sim2sim_drone import rc_input

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--config", default=os.path.join(get_package_share_directory("ocean_sim2sim_drone"), "config", "rc_input.yaml"))
ap.add_argument("--port", default=None, help="serial device (default: config port)")
a = ap.parse_args()
rc_input.watch(rc_input.load_config(a.config), a.port)
