# =============================================================
# rc_input.py - real transmitter input (Radiolink -> receiver -> Raspberry Pi Pico -> USB serial)
#
#   Ported from: data_collection_ur5_gripper isaacsim/scripts/rc_input.py (parse_line, RcReader, load_config, RcMap,
#     RcInput) - same line format, checks, normalisation and dead zone. docs/drone_teleop.md.
#   The Pico sends one line per sample (142.9 Hz):
#       RC,0,0,1006,1002,995,996,200,200,200,1800
#     header "RC" + 10 integers: values 1, 2 = signal status (0 = ok, 1 = transmitter off), 3 ~ 6 = the 4 stick axes,
#     7 ~ 10 unused (config/rc_input.yaml).
#   parse_line: one line -> 10 ints. Other formats = ValueError (never silently skipped)
#   RcReader  : reads the serial device in a thread, keeps (receive time, values); bad lines kept apart
#   RcMap     : 10 values -> sticks (pitch, roll, throttle, yaw) -1 ~ 1. Signal lost / out of range -> None + reason
#   RcInput   : RcReader + RcMap. poll() every loop (no line for stale_sec -> None + reason)
#   Standard library only (termios), no pyserial.
#
#   Check the device without the sim (values only, Ctrl+C to stop):
#       ros2 run ocean_sim2sim_drone rc_watch.py [--port /dev/ttyACM0]
#   Permission: /dev/ttyACM* is root:dialout -> sudo usermod -aG dialout $USER, then log in again (never run the sim as root)
#
#   Changes from the Isaac Sim file: messages in English; the A-0 probe (stick-by-stick measurement) is not ported
#     (the axis values in rc_input.yaml are the ones it measured); watch() kept.
# =============================================================

import os
import threading
import time

HEADER = "RC"
N_VALUES = 10
AXES = ("pitch", "roll", "throttle", "yaw")       # PX4Commander.set_sticks order


def parse_line(line):
    """'RC,v1,...,v10' -> [int] * 10. Other formats = ValueError."""
    parts = line.strip().split(",")
    if parts[0] != HEADER:
        raise ValueError(f"header is not {HEADER!r}: {line!r}")
    if len(parts) != 1 + N_VALUES:
        raise ValueError(f"not {N_VALUES} values ({len(parts) - 1}): {line!r}")
    try:
        return [int(p) for p in parts[1:]]
    except ValueError:
        raise ValueError(f"non-integer value: {line!r}") from None


class RcReader:
    """Reads the serial device in a thread. samples = [(time.monotonic(), 10 values)], bad = [(time, line, reason)]."""

    def __init__(self, port, baudrate=115200):
        import termios
        import tty

        self.fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)                       # no line editing / echo / character translation
            attr = termios.tcgetattr(self.fd)
            speed = getattr(termios, f"B{int(baudrate)}", None)
            if speed is None:
                raise ValueError(f"unsupported baudrate: {baudrate}")
            attr[4] = attr[5] = speed
            attr[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        except Exception:
            os.close(self.fd)
            raise
        self.samples, self.bad = [], []
        self.error = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        import termios

        termios.tcflush(self.fd, termios.TCIFLUSH)    # drop old lines queued before opening
        self._thread.start()

    def _run(self):
        import select

        buf, first = b"", True
        try:
            while not self._stop.is_set():
                if not select.select([self.fd], [], [], 0.2)[0]:
                    continue
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    raise OSError("serial device closed (unplugged?)")
                t = time.monotonic()
                buf += chunk
                *raws, buf = buf.split(b"\n")
                for raw in raws:
                    if first:                         # the first line after the flush may start in the middle
                        first = False
                        continue
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    try:
                        self.samples.append((t, parse_line(line)))
                    except ValueError as e:
                        self.bad.append((t, line, str(e)))
        except Exception as e:  # noqa: BLE001  (unplugged, ... -> raised again on the reading side)
            self.error = e

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"serial read error: {self.error!r}") from self.error

    def latest(self):
        return self.samples[-1] if self.samples else None

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        os.close(self.fd)


def load_config(path):
    import yaml

    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    for k in ("port", "baudrate", "stale_sec", "lost_flags", "axes", "range_margin", "deadzone", "handover_center", "px4_params",
              "handover_params"):
        if k not in cfg:
            raise KeyError(f"{path}: missing '{k}'")
    for k, v in (cfg["handover_params"] or {}).items():
        if not isinstance(v, float):
            raise ValueError(f"{path}: handover_params.{k} = {v!r} must be written as a float (runtime PARAM_SET is float only)")
    used = set()
    for name in AXES:
        if name not in cfg["axes"]:
            raise KeyError(f"{path}: missing axes.{name}")
        ax = cfg["axes"][name]
        if not (1 <= ax["index"] <= N_VALUES) or ax["index"] in used or ax["index"] in cfg["lost_flags"]:
            raise ValueError(f"{path}: axes.{name}.index {ax['index']} out of range or used twice")
        used.add(ax["index"])
        if not (ax["min"] < ax["center"] < ax["max"]) or ax["sign"] not in (1, -1):
            raise ValueError(f"{path}: axes.{name} needs min < center < max, sign +-1: {ax}")
    cfg["path"] = path
    return cfg


class RcMap:
    """10 serial values -> sticks (pitch, roll, throttle, yaw) -1 ~ 1 (centre 0). Signal lost / out of range -> (None, reason)."""

    def __init__(self, cfg):
        self.cfg = cfg

    def sticks(self, values):
        c = self.cfg
        if any(values[i - 1] != 0 for i in c["lost_flags"]):
            return None, f"transmitter signal lost (status values {[values[i - 1] for i in c['lost_flags']]})"
        out = []
        for name in AXES:
            ax = c["axes"][name]
            v = values[ax["index"] - 1]
            if not (ax["min"] - c["range_margin"] <= v <= ax["max"] + c["range_margin"]):
                return None, f"{name} value {v} outside {ax['min']} ~ {ax['max']}"
            x = (v - ax["center"]) / ((ax["max"] - ax["center"]) if v >= ax["center"] else (ax["center"] - ax["min"]))
            x = max(-1.0, min(1.0, x)) * ax["sign"]
            out.append(0.0 if abs(x) < c["deadzone"] else x)
        return tuple(out), None

    def centered(self, sticks):
        return all(abs(x) <= self.cfg["handover_center"] for x in sticks)


class RcInput:
    """One transmitter input: serial reader (RcReader) + mapping (RcMap). poll() -> (sticks or None, reason).
    No line for stale_sec (wall time) = no input."""

    def __init__(self, cfg, port=None):
        self.cfg, self.map = cfg, RcMap(cfg)
        self.reader = RcReader(cfg["port"] if not port else port, cfg["baudrate"])
        self.reader.start()

    def poll(self):
        self.reader.check()
        last = self.reader.latest()
        del self.reader.samples[:-1]              # only the newest line is used (they pile up at 142.9 Hz)
        if last is None:
            return None, "no line received"
        age = time.monotonic() - last[0]
        if age > self.cfg["stale_sec"]:
            return None, f"no line for {age:.2f} s (Pico / USB)"
        return self.map.sticks(last[1])

    def close(self):
        self.reader.close()


def watch(cfg, port=None, rate_hz=10.0):
    """Print raw values and the mapped sticks until Ctrl+C."""
    rc = RcInput(cfg, port)
    try:
        while True:
            time.sleep(1.0 / rate_hz)
            last = rc.reader.latest()
            sticks, why = rc.poll()
            raw = None if last is None else last[1]
            s = "None" if sticks is None else " ".join(f"{n} {x:+.2f}" for n, x in zip(AXES, sticks))
            print(f"raw {raw}  sticks {s}" + (f"  ({why})" if why else "") + f"  bad lines {len(rc.reader.bad)}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        rc.close()
