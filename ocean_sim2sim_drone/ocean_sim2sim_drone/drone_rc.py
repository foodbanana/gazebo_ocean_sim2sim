# =============================================================
# drone_rc.py - fly the PX4 drone with the real transmitter (docs/teleop.md, Isaac Sim docs/drone_teleop.md)
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/drone_rc.py (DroneRc) - same hand-over / take-back rules.
#   DroneRc.update() every bridge loop (before PX4Commander.tick): transmitter input (rc_input.RcInput) -> PX4Commander
#     - hand over: drone hovering in Offboard (scene finished the air start, cmd.cli_ready) + transmitter signal + all 4
#       axes near the centre -> handover_params set at runtime and confirmed -> handover() (PX4 manual mode POSCTL, sticks
#       as MANUAL_CONTROL on the offboard link UDP 14540). Sticks not centred: no hand-over, a notice instead
#     - take back: after the hand-over, signal lost / bad input / no line -> takeback() = Offboard hover where it is +
#       notice. Signal back and sticks centred -> handed over again
#     - /sim/reset restarts PX4 (reset_link clears the hand-over) -> handed over again after the new air start
#     - after /sim/drone_kill (disarmed): nothing until the next reset
#   The transmitter input is not a policy action (the policy only moves the arm and gripper).
#
#   Changes from the Isaac Sim file: takes the PX4Commander and a sim-time function instead of DroneFlight (the
#     Gazebo bridge owns the commander), messages in English, a notice repeats only after NOTICE_SEC also when its
#     numbers change (Isaac printed "no line for 0.21 s", "0.22 s", ... every loop).
# =============================================================

import re

from ocean_sim2sim_drone import rc_input

NOTICE_SEC = 5.0       # repeat interval of the same notice (sim s)


class DroneRc:
    def __init__(self, cmd, cfg, now, port=None, log=print):
        self.cmd, self.cfg, self.now, self.log = cmd, cfg, now, log
        self.rc = rc_input.RcInput(cfg, port)
        self.sticks, self.why = None, "no line received"
        self.n_handover, self.n_takeback = 0, 0
        self._notice = (None, -1e9)

    def _say(self, msg, repeat=True):
        # same notice = same text without its numbers ("no line for 0.21 s" and "0.22 s" are one notice)
        key = re.sub(r"[-+]?\d+(\.\d+)?", "#", msg)
        last, t = self._notice
        if key != last or (repeat and self.now() - t >= NOTICE_SEC):
            self.log(f"[drone_rc] t {self.now():7.1f} s  {msg}")
            self._notice = (key, self.now())

    def update(self):
        c = self.cmd
        self.sticks, self.why = self.rc.poll()
        if c.want_manual:
            if self.sticks is None:
                c.takeback()
                self.n_takeback += 1
                self._say(f"no transmitter input ({self.why}) -> drone back to Offboard hover where it is")
            else:
                c.set_sticks(*self.sticks)
            return
        if not (c.cli_ready and c.connected and c.armed):
            return                                              # air start / reset in progress, or after a kill
        if self.sticks is None:
            self._say(f"no transmitter input ({self.why}) -> not handed over (drone hovers by itself)")
            return
        if c.sticks is None:
            c.set_sticks(0.0, 0.0, 0.0, 0.0)                    # centre values first so PX4 sees a manual input
        if c.mode != "OFFBOARD":
            return                                              # taking back (waiting for Offboard)
        if not self.rc.map.centered(self.sticks):
            names = [n for n, x in zip(rc_input.AXES, self.sticks) if abs(x) > self.cfg["handover_center"]]
            self._say(f"sticks not centred ({', '.join(names)}) -> centre them to hand over to the transmitter")
            return
        hp = self.cfg["handover_params"] or {}
        if not c.params_confirmed(hp):
            if any(c.param_wanted.get(k) != float(v) for k, v in hp.items()):
                c.set_params(hp)
            self._say("setting the transmitter PX4 parameters (hand-over once confirmed)")
            return
        c.set_sticks(*self.sticks)
        c.handover()
        self.n_handover += 1
        self._say("handed over to the transmitter (PX4 manual mode requested)", repeat=False)

    def status(self):
        return {"manual": self.cmd.want_manual, "handovers": self.n_handover, "takebacks": self.n_takeback,
                "sticks": None if self.sticks is None else [round(x, 3) for x in self.sticks], "why": self.why}

    def close(self):
        self.rc.close()
