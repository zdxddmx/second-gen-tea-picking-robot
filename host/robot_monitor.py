# -*- coding: utf-8 -*-
"""host/robot_monitor.py -- PC monitoring / debugging tool for the robot.

Two front ends over one shared core
-----------------------------------
* a Tkinter GUI with live numeric readouts, a rolling time-series plot drawn on
  a plain ``tkinter.Canvas`` (matplotlib is deliberately NOT required) and
  buttons / entry fields for every host command
* a ``--nogui`` headless text dashboard that renders to stdout with an ASCII
  sparkline, so the tool is fully usable on a machine with no display

Both front ends call exactly the same :class:`RobotController` command-sending
methods -- there is no duplicated command logic.

``tkinter`` is imported inside a ``try/except``; importing this module always
succeeds even when Tk is missing or unusable, and ``--nogui`` never touches Tk.

Examples::

    python host/robot_monitor.py --demo --nogui --duration 5 --send-demo-sequence
    python host/robot_monitor.py --demo --nogui --duration 3 --csv data/logs/x.csv
    python host/robot_monitor.py --port COM5 --baud 921600
    python host/robot_monitor.py --tcp 127.0.0.1:9000

Python 3.8 compatible.
"""

import argparse
import os
import sys
import time
from collections import deque
from typing import Callable, Dict, List, Optional, Tuple

if __package__ in (None, ""):  # allow ``python host/robot_monitor.py``
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import protocol
from host.link import Link, LinkError, LoopbackLink, open_link, parse_hostport
from host.protocol import CMD, FAULT, STATE, Frame

# --------------------------------------------------------------------------
# Tkinter is optional: import defensively so this module always imports.
# --------------------------------------------------------------------------
tk = None
ttk = None
TK_IMPORT_ERROR = None  # type: Optional[str]
try:  # pragma: no cover - depends on the host machine
    import tkinter as tk
    from tkinter import ttk
except Exception as _tk_exc:  # noqa: BLE001 - any failure means "no GUI"
    tk = None
    ttk = None
    TK_IMPORT_ERROR = "%s: %s" % (type(_tk_exc).__name__, _tk_exc)

TkUnavailable = RuntimeError

__all__ = ["TelemetryModel", "RobotController", "MonitorBase", "HeadlessMonitor",
           "GuiMonitor", "CanvasPlot", "DEMO_SEQUENCE", "main", "tk", "ttk",
           "TK_IMPORT_ERROR"]


# --------------------------------------------------------------------------
# Telemetry model
# --------------------------------------------------------------------------

class TelemetryModel(object):
    """Latest robot values plus rolling history for the plot."""

    FIELDS = ("t_ms", "pos_mrad", "vel_mrad_s", "force_mN", "pwm_permille",
              "state", "fault", "current_ma", "cycle_count", "adc_raw",
              "temp_c_x10")

    def __init__(self, history=1500):
        # type: (int) -> None
        self.history = int(history)
        self.t0 = time.time()
        self.reset()

    def reset(self):
        # type: () -> None
        self.t_ms = 0
        self.pos_mrad = 0
        self.vel_mrad_s = 0
        self.force_mN = 0
        self.pwm_permille = 0
        self.state = STATE.INIT
        self.fault = FAULT.NONE
        self.current_ma = 0
        self.cycle_count = 0
        self.adc_raw = 0
        self.temp_c_x10 = 0
        self.robot_t_ms = 0     # robot-reported timestamp (TELEMETRY frames)
        self.samples = 0
        self.last_host_time = time.time()
        self.series = {
            "pos_mrad": deque(maxlen=self.history),
            "vel_mrad_s": deque(maxlen=self.history),
            "force_mN": deque(maxlen=self.history),
            "pwm_permille": deque(maxlen=self.history),
        }  # type: Dict[str, deque]

    # -- accessors --------------------------------------------------------
    @property
    def state_name(self):
        # type: () -> str
        return protocol.state_name(self.state)

    @property
    def fault_name(self):
        # type: () -> str
        return protocol.fault_name(self.fault)

    @property
    def force_n(self):
        # type: () -> float
        return self.force_mN / 1000.0

    def _record(self):
        # type: () -> None
        t = self.t_ms / 1000.0
        self.samples += 1
        for key in self.series:
            self.series[key].append((t, float(getattr(self, key))))

    def update_from_frame(self, frame):
        # type: (Frame) -> Optional[str]
        """Refresh the model from a robot->host frame; returns a log line."""
        cmd = frame.cmd
        if cmd == CMD.STATUS_RPT:
            r = protocol.decode_status_rpt(frame.payload)
            self.state, self.fault = r.state, r.fault_code
            self.pos_mrad, self.vel_mrad_s = r.pos_mrad, r.vel_mrad_s
            self.force_mN, self.pwm_permille = r.force_mN, r.pwm_permille
            self.current_ma, self.cycle_count = r.current_ma, r.cycle_count
            self._stamp()
            return ("STATUS state=%s fault=%s pos=%d mrad vel=%d mrad/s "
                    "force=%d mN pwm=%d/1000 i=%d mA cyc=%d"
                    % (self.state_name, self.fault_name, self.pos_mrad,
                       self.vel_mrad_s, self.force_mN, self.pwm_permille,
                       self.current_ma, self.cycle_count))
        if cmd == CMD.TELEMETRY:
            t = protocol.decode_telemetry(frame.payload)
            self.robot_t_ms = t.t_ms
            self.pos_mrad, self.vel_mrad_s = t.pos_mrad, t.vel_mrad_s
            self.force_mN, self.pwm_permille = t.force_mN, t.pwm_permille
            self.state, self.fault = t.state, t.fault
            self._stamp()
            return None       # telemetry is high rate: not logged line by line
        if cmd == CMD.SENSOR_RPT:
            s = protocol.decode_sensor_rpt(frame.payload)
            self.pos_mrad, self.vel_mrad_s = s.pos_mrad, s.vel_mrad_s
            self.force_mN = s.force_mN
            self.adc_raw, self.current_ma = s.adc_raw, s.current_ma
            self.temp_c_x10 = s.temp_c_x10
            self._stamp()
            return ("SENSOR pos=%d mrad vel=%d mrad/s force=%d mN adc=%d "
                    "i=%d mA temp=%.1f C"
                    % (s.pos_mrad, s.vel_mrad_s, s.force_mN, s.adc_raw,
                       s.current_ma, s.temp_c_x10 / 10.0))
        return None

    def _stamp(self):
        # type: () -> int
        """Advance the sample clock by the host clock, strictly monotonically.

        STATUS_RPT/SENSOR_RPT carry no timestamp and TELEMETRY carries the
        robot's own clock; mixing the two directly would produce non-monotonic
        CSV rows, so every sample is stamped from one monotonic host clock.
        """
        t = self._host_ms()
        if t <= self.t_ms:
            t = self.t_ms + 1
        self.t_ms = t
        self._record()
        return t

    def _host_ms(self):
        # type: () -> int
        return int((time.time() - self.t0) * 1000.0)

    def as_dict(self):
        # type: () -> Dict[str, object]
        out = {name: getattr(self, name) for name in self.FIELDS}
        out["state_name"] = self.state_name
        out["fault_name"] = self.fault_name
        return out


class CsvLogger(object):
    """Appends received telemetry to a CSV file with the analyser's schema."""

    HEADER = "t_ms,pos_mrad,vel_mrad_s,force_mN,pwm_permille,state,fault"

    def __init__(self, path):
        # type: (Optional[str]) -> None
        self.path = path
        self._fh = None
        self.rows = 0
        if path:
            directory = os.path.dirname(os.path.abspath(path))
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            self._fh = open(path, "w", encoding="utf-8", newline="\n")
            self._fh.write(self.HEADER + "\n")
            self._fh.flush()

    def write(self, model):
        # type: (TelemetryModel) -> None
        if self._fh is None:
            return
        self._fh.write("%d,%d,%d,%d,%d,%d,%d\n" % (
            model.t_ms, model.pos_mrad, model.vel_mrad_s, model.force_mN,
            model.pwm_permille, model.state, model.fault))
        self.rows += 1

    def flush(self):
        # type: () -> None
        if self._fh is not None:
            self._fh.flush()

    def close(self):
        # type: () -> None
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            finally:
                self._fh = None


# --------------------------------------------------------------------------
# Command layer -- used identically by the GUI and the headless dashboard
# --------------------------------------------------------------------------

class RobotController(object):
    """Every host->robot command, in one place."""

    def __init__(self, link, log=None):
        # type: (Link, Optional[Callable[[str], None]]) -> None
        self.link = link
        self.log = log if log is not None else (lambda _msg: None)
        self.heartbeat_seq = 0

    def _send(self, cmd, payload=b"", note=""):
        # type: (int, bytes, str) -> bytes
        wire = self.link.send_frame(cmd, payload)
        text = "TX %-16s %s" % (protocol.cmd_name(cmd), note)
        self.log(text.rstrip())
        return wire

    # -- safety / lifecycle ----------------------------------------------
    def send_reset(self):
        # type: () -> bytes
        return self._send(CMD.RESET, protocol.encode_reset(), "return to INIT")

    def send_stop(self):
        # type: () -> bytes
        return self._send(CMD.STOP, protocol.encode_stop(), "halt motion")

    def send_emergency_stop(self):
        # type: () -> bytes
        return self._send(CMD.EMERGENCY_STOP, protocol.encode_emergency_stop(),
                          "EMERGENCY STOP")

    def send_clear_fault(self):
        # type: () -> bytes
        return self._send(CMD.CLEAR_FAULT, protocol.encode_clear_fault(), "clear fault latch")

    def send_set_mode(self, mode):
        # type: (int) -> bytes
        return self._send(CMD.SET_MODE, protocol.encode_set_mode(mode),
                          "mode=%s" % protocol.MODE_NAMES.get(mode, mode))

    # -- pick cycle -------------------------------------------------------
    def send_set_target(self, x_mm, y_mm, z_mm, confidence=90, target_id=1):
        # type: (int, int, int, int, int) -> bytes
        payload = protocol.encode_set_target(x_mm, y_mm, z_mm, confidence, target_id)
        return self._send(CMD.SET_TARGET, payload,
                          "x=%d y=%d z=%d mm conf=%d%% id=%d"
                          % (x_mm, y_mm, z_mm, confidence, target_id))

    def send_start_pick(self, target_id=0):
        # type: (int) -> bytes
        return self._send(CMD.START_PICK, protocol.encode_start_pick(target_id),
                          "id=%d" % target_id)

    def send_start(self, target_id=0):
        # type: (int) -> bytes
        """GUI 'Start' button: begin the pick cycle for ``target_id``."""
        return self.send_start_pick(target_id)

    # -- tuning -----------------------------------------------------------
    def send_set_pid(self, loop_id, kp, ki, kd):
        # type: (int, float, float, float) -> bytes
        payload = protocol.encode_set_pid(loop_id, kp, ki, kd)
        return self._send(CMD.SET_PID, payload,
                          "loop=%d kp=%.4f ki=%.4f kd=%.4f"
                          % (loop_id, kp, ki, kd))

    def send_get_pid(self, loop_id=0):
        # type: (int) -> bytes
        return self._send(CMD.GET_PID, protocol.encode_get_pid(loop_id),
                          "loop=%d" % loop_id)

    def send_set_force_target(self, force_mN):
        # type: (int) -> bytes
        return self._send(CMD.SET_FORCE_TARGET,
                          protocol.encode_set_force_target(force_mN),
                          "%d mN (%.2f N)" % (force_mN, force_mN / 1000.0))

    # -- polling ----------------------------------------------------------
    def send_get_status(self):
        # type: () -> bytes
        return self._send(CMD.GET_STATUS, protocol.encode_get_status())

    def send_get_sensor(self):
        # type: () -> bytes
        return self._send(CMD.GET_SENSOR, protocol.encode_get_sensor())

    def send_get_fault(self):
        # type: () -> bytes
        return self._send(CMD.GET_FAULT, protocol.encode_get_fault())

    def send_heartbeat(self):
        # type: () -> bytes
        self.heartbeat_seq = (self.heartbeat_seq + 1) & 0xFFFFFFFF
        return self._send(CMD.HEARTBEAT,
                          protocol.encode_heartbeat(self.heartbeat_seq),
                          "seq=%d" % self.heartbeat_seq)


# --------------------------------------------------------------------------
# Demo sequence: (offset seconds, controller method, args, label[, wait_for])
#
# ``wait_for`` optionally names a robot state that must have been observed
# before the step is allowed to fire; it stops the demo racing the plant when a
# step must happen *after* the pick cycle finishes.
# --------------------------------------------------------------------------

DEMO_SEQUENCE = [
    (0.10, "send_reset", (), "RESET"),
    (0.30, "send_set_mode", (protocol.MODE_AUTO,), "SET_MODE AUTO"),
    (0.50, "send_clear_fault", (), "CLEAR_FAULT"),
    (0.70, "send_set_force_target", (1200,), "SET_FORCE_TARGET"),
    (0.90, "send_set_pid", (0, 2.4, 0.35, 0.05), "SET_PID loop 0"),
    (1.10, "send_get_pid", (0,), "GET_PID loop 0"),
    (1.30, "send_set_target", (12, -8, 180, 92, 101), "SET_TARGET bud 101"),
    (1.50, "send_start_pick", (101,), "START_PICK bud 101"),
    # the pick cycle needs ~3.1 s of plant time; wait for IDLE rather than
    # guessing, then read the final status and stop
    (4.20, "send_get_status", (), "GET_STATUS", "IDLE"),
    (4.40, "send_get_sensor", (), "GET_SENSOR", "IDLE"),
    (4.70, "send_stop", (), "STOP", "IDLE"),
]  # type: List[tuple]


# --------------------------------------------------------------------------
# Shared monitor core
# --------------------------------------------------------------------------

class MonitorBase(object):
    """Transport-independent monitor: pumping, frame routing, demo sequencing."""

    #: how long a demo step may wait for its required robot state
    DEMO_WAIT_TIMEOUT = 2.0

    def __init__(self, link, sim=None, csv_path=None, max_events=400):
        # type: (Link, Optional[object], Optional[str], int) -> None
        self.link = link
        self.sim = sim
        self.model = TelemetryModel()
        self.csv = CsvLogger(csv_path)
        self.events = deque(maxlen=int(max_events))  # type: deque
        self.event_sinks = []  # type: List[Callable[[str], None]]
        self.controller = RobotController(link, self.log)
        self.t0 = time.time()
        self.running = False
        self.demo_enabled = False
        self.demo_index = 0
        self.demo_log = []  # type: List[Tuple[float, str]]
        self._last_heartbeat = 0.0
        self._last_status = 0.0
        self._last_sensor = 0.0
        self.counts = {"rx_frames": 0, "acks": 0, "nacks": 0, "fault_rpts": 0,
                       "status": 0, "telemetry": 0, "sensor": 0, "pid": 0,
                       "heartbeat_ack": 0, "other": 0}
        self.fault_events = []  # type: List[Tuple[float, int, int]]

    # -- logging ----------------------------------------------------------
    def log(self, message):
        # type: (str) -> None
        stamp = time.time() - self.t0
        entry = "[%8.3fs] %s" % (stamp, message)
        self.events.append(entry)
        for sink in self.event_sinks:
            sink(entry)

    def attach_event_sink(self, sink):
        # type: (Callable[[str], None]) -> None
        self.event_sinks.append(sink)

    # -- lifecycle --------------------------------------------------------
    def start(self):
        # type: () -> None
        self.running = True
        self.t0 = time.time()
        self.log("monitor started on %s" % self.link.describe())
        self.controller.send_get_status()

    def close(self):
        # type: () -> None
        self.running = False
        self.csv.close()
        try:
            self.link.close()
        except Exception:  # noqa: BLE001 - closing must never raise
            pass

    # -- pump -------------------------------------------------------------
    def pump(self, now=None):
        # type: (Optional[float]) -> None
        """Move bytes both ways: sim telemetry in, host frames out."""
        now = time.time() if now is None else now
        if self.sim is not None:
            data = self.sim.tick(now)
            if data and isinstance(self.link, LoopbackLink):
                self.link.push_rx(data)
        for frame in self.link.read_frames():
            self.on_frame(frame, now)
        self.service_periodic(now)

    def service_periodic(self, now):
        # type: (float) -> None
        if not self.running:
            return
        if now - self._last_heartbeat >= 1.0:
            self._last_heartbeat = now
            self.controller.send_heartbeat()
        if now - self._last_status >= 0.2:
            self._last_status = now
            self.controller.send_get_status()
        if now - self._last_sensor >= 1.0:
            self._last_sensor = now
            self.controller.send_get_sensor()

    # -- receive path -----------------------------------------------------
    def on_frame(self, frame, now=None):
        # type: (Frame, Optional[float]) -> None
        now = time.time() if now is None else now
        self.counts["rx_frames"] = self.counts["rx_frames"] + 1
        cmd = frame.cmd
        detail = self.model.update_from_frame(frame)
        if cmd in (CMD.STATUS_RPT, CMD.SENSOR_RPT):
            self.counts["status" if cmd == CMD.STATUS_RPT else "sensor"] += 1
            self.csv.write(self.model)
            if detail:
                self.log("RX " + detail)
        elif cmd == CMD.TELEMETRY:
            self.counts["telemetry"] += 1
            self.csv.write(self.model)
        elif cmd == CMD.ACK:
            self.counts["acks"] += 1
            ack = protocol.decode_ack(frame.payload)
            self.log("RX ACK            %s" % protocol.cmd_name(ack.acked_cmd))
        elif cmd == CMD.NACK:
            nack = protocol.decode_nack(frame.payload)
            self.counts["nacks"] += 1
            self.log("RX NACK           %s err=%s"
                     % (protocol.cmd_name(nack.ref_cmd),
                        protocol.ERRCODE_NAMES.get(nack.err_code, nack.err_code)))
        elif cmd == CMD.HEARTBEAT_ACK:
            self.counts["heartbeat_ack"] += 1
            hb = protocol.decode_heartbeat_ack(frame.payload)
            self.log("RX HEARTBEAT_ACK  seq=%d" % hb.seq)
        elif cmd == CMD.PID_RPT:
            self.counts["pid"] += 1
            r = protocol.decode_pid_rpt(frame.payload)
            self.log("RX PID_RPT        loop=%d kp=%.4f ki=%.4f kd=%.4f"
                     % (r.loop_id, r.kp, r.ki, r.kd))
        elif cmd == CMD.FAULT_RPT:
            self.counts["fault_rpts"] += 1
            r = protocol.decode_fault_rpt(frame.payload)
            self.fault_events.append((time.time() - self.t0, r.fault_code, r.timestamp_ms))
            self.log("RX FAULT_RPT      %s t=%d ms detail=%d"
                     % (protocol.fault_name(r.fault_code), r.timestamp_ms, r.detail))
        else:
            self.counts["other"] += 1
            self.log("RX " + protocol.describe_frame(frame))

    # -- demo sequencing ---------------------------------------------------
    def enable_demo_sequence(self, sequence=None):
        # type: (Optional[List[tuple]]) -> None
        self.demo_sequence = list(DEMO_SEQUENCE if sequence is None else sequence)
        self.demo_enabled = True
        self.demo_index = 0
        self.demo_t0 = time.time()
        self.log("demo sequence armed (%d steps)" % len(self.demo_sequence))

    def step_demo(self, now=None):
        # type: (Optional[float]) -> None
        """Fire any demo step whose time has come (uses the controller API).

        A step may carry a 5th element naming a robot state that must have been
        observed first; it is then deferred (up to ``DEMO_WAIT_TIMEOUT`` seconds
        past its scheduled time) so the sequence never races the plant.
        """
        if not getattr(self, "demo_enabled", False):
            return
        now = time.time() if now is None else now
        elapsed = now - self.demo_t0
        while self.demo_index < len(self.demo_sequence):
            step = self.demo_sequence[self.demo_index]
            offset, method_name, args, label = step[0], step[1], step[2], step[3]
            wait_for = step[4] if len(step) > 4 else None
            if elapsed < offset:
                break
            if wait_for is not None and (elapsed - offset) < self.DEMO_WAIT_TIMEOUT:
                if self.model.state_name != wait_for:
                    break        # robot has not got there yet: retry next tick
            self.demo_index += 1
            method = getattr(self.controller, method_name, None)
            if method is None:
                self.log("demo: no such command %s" % method_name)
                continue
            self.log("demo step %d/%d: %s"
                     % (self.demo_index, len(self.demo_sequence), label))
            self.demo_log.append((elapsed, label))
            try:
                method(*args)
            except LinkError as exc:
                self.log("demo step failed: %s" % exc)

    # -- summary ----------------------------------------------------------
    def summary(self):
        # type: () -> str
        stats = self.link.stats
        lines = [
            "--- monitor summary ---",
            "transport        : %s" % self.link.describe(),
            "elapsed          : %.2f s" % (time.time() - self.t0),
            "link             : %s" % stats.summary(),
            "frames received  : %d" % self.counts["rx_frames"],
            "  status reports : %d" % self.counts["status"],
            "  telemetry      : %d" % self.counts["telemetry"],
            "  sensor reports : %d" % self.counts["sensor"],
            "  acks / nacks   : %d / %d" % (self.counts["acks"], self.counts["nacks"]),
            "  fault reports  : %d" % self.counts["fault_rpts"],
            "final state      : %s" % self.model.state_name,
            "final fault      : %s" % self.model.fault_name,
            "final position   : %d mrad" % self.model.pos_mrad,
            "final force      : %d mN (%.3f N)" % (self.model.force_mN, self.model.force_n),
            "csv rows written : %d%s" % (self.csv.rows,
                                         " -> %s" % self.csv.path if self.csv.path else ""),
        ]
        if self.sim is not None:
            lines.append("sim cycle count  : %d" % self.sim.cycle_count)
        if self.demo_log:
            lines.append("demo steps run   : %d/%d"
                         % (len(self.demo_log), len(self.demo_sequence)))
            for stamp, label in self.demo_log:
                lines.append("  t=%7.3f s  %s" % (stamp, label))
        if self.fault_events:
            lines.append("fault timeline   :")
            for stamp, code, robot_ms in self.fault_events:
                lines.append("  t=%7.3f s  %s (robot t=%d ms)"
                             % (stamp, protocol.fault_name(code), robot_ms))
        return "\n".join(lines)


# --------------------------------------------------------------------------
# Text rendering helpers
# --------------------------------------------------------------------------

SPARK_CHARS = " .:-=+*#%@"


def sparkline(values, width=48, low=None, high=None):
    # type: (List[float], int, Optional[float], Optional[float]) -> str
    """ASCII sparkline for the headless dashboard."""
    if not values:
        return " " * width
    data = list(values)[-int(width):]
    lo = min(data) if low is None else low
    hi = max(data) if high is None else high
    if hi - lo < 1e-9:
        hi = lo + 1.0
    out = []
    for v in data:
        idx = int((v - lo) / (hi - lo) * (len(SPARK_CHARS) - 1))
        out.append(SPARK_CHARS[max(0, min(len(SPARK_CHARS) - 1, idx))])
    return "".join(out).ljust(int(width))


def _bar(value, low, high, width=22):
    # type: (float, float, float, int) -> str
    if high - low < 1e-9:
        return " " * width
    frac = (float(value) - low) / (high - low)
    frac = max(0.0, min(1.0, frac))
    filled = int(round(frac * width))
    return "[" + "#" * filled + "." * (width - filled) + "]"


# --------------------------------------------------------------------------
# Headless front end
# --------------------------------------------------------------------------

class HeadlessMonitor(MonitorBase):
    """Prints the monitoring dashboard to stdout; needs no display."""

    def __init__(self, link, sim=None, csv_path=None, render_interval=1.0,
                 interactive=None):
        # type: (Link, Optional[object], Optional[str], float, Optional[bool]) -> None
        MonitorBase.__init__(self, link, sim=sim, csv_path=csv_path)
        self.render_interval = float(render_interval)
        self._last_render = 0.0
        self.renders = 0
        if interactive is None:
            interactive = bool(getattr(sys.stdout, "isatty", lambda: False)())
        self.interactive = bool(interactive)
        # stream milestone events immediately: the scrolling window in the
        # dashboard is only ever the last few entries, so a run is not auditable
        # from the dashboard alone
        self.attach_event_sink(self._stream_milestone)

    def _stream_milestone(self, entry):
        # type: (str) -> None
        if "demo step" in entry or "FAULT_RPT" in entry or "nacks" in entry:
            sys.stdout.write("EVENT %s\n" % entry)
            sys.stdout.flush()

    def render(self, force=False):
        # type: (bool) -> None
        now = time.time()
        if not force and (now - self._last_render) < self.render_interval:
            return
        self._last_render = now
        self.renders += 1
        sys.stdout.write(self.dashboard() + "\n")
        sys.stdout.flush()

    def dashboard(self):
        # type: () -> str
        m = self.model
        elapsed = time.time() - self.t0
        pos_hist = [v for _t, v in m.series["pos_mrad"]]
        force_hist = [v for _t, v in m.series["force_mN"]]
        rule = "+" + "-" * 78 + "+"

        def row(text):
            return "| %-76s |" % text[:76]

        lines = [rule,
                 row("TEA-PICKING ROBOT MONITOR  (headless)   t = %7.2f s" % elapsed),
                 rule,
                 row("Position      : %10d mrad   %s"
                     % (m.pos_mrad, _bar(m.pos_mrad, -2000, 4000))),
                 row("                %s" % sparkline(pos_hist, 48)),
                 row("Speed         : %10d mrad/s" % m.vel_mrad_s),
                 row("Force         : %10d mN  (%6.3f N)  %s"
                     % (m.force_mN, m.force_n, _bar(m.force_mN, 0, 4000))),
                 row("                %s" % sparkline(force_hist, 48, 0.0, 4000.0)),
                 row("PID output    : %10d permille  %s"
                     % (m.pwm_permille, _bar(m.pwm_permille, -1000, 1000))),
                 row("Robot state   : %-20s  cycles = %d" % (m.state_name, m.cycle_count)),
                 row("Fault code    : %-20s  %s"
                     % (m.fault_name, "OK" if m.fault == FAULT.NONE else "*** FAULT ***")),
                 row("Motor current : %10d mA      temp = %.1f C"
                     % (m.current_ma, m.temp_c_x10 / 10.0)),
                 row("Samples       : %10d         adc = %d" % (m.samples, m.adc_raw)),
                 rule,
                 row("Link          : tx=%d rx=%d crc_err=%d discarded=%d lost=%d"
                     % (self.link.stats.tx_frames, self.link.stats.rx_frames,
                        self.link.stats.crc_errors, self.link.stats.bytes_discarded,
                        self.link.stats.packets_lost)),
                 row("                %s" % self.link.describe()),
                 row("RX counts     : status=%d telemetry=%d sensor=%d ack=%d nack=%d fault=%d"
                     % (self.counts["status"], self.counts["telemetry"],
                        self.counts["sensor"], self.counts["acks"],
                        self.counts["nacks"], self.counts["fault_rpts"])),
                 rule]
        recent = list(self.events)[-6:]
        lines.append("| recent events" + " " * 63 + "|")
        for entry in recent:
            lines.append(row(entry))
        lines.append(rule)
        return "\n".join(lines)

    def run(self, duration=0.0):
        # type: (float) -> int
        """Blocking loop: pump at ~50 Hz, render the dashboard periodically."""
        self.start()
        self.render(force=True)
        deadline = (time.time() + duration) if duration > 0 else None
        try:
            while True:
                now = time.time()
                if deadline is not None and now >= deadline:
                    break
                self.pump(now)
                self.step_demo(now)
                self.render()
                time.sleep(0.02)
        except KeyboardInterrupt:
            self.log("interrupted by user")
        self.render(force=True)
        return 0


# --------------------------------------------------------------------------
# GUI front end
# --------------------------------------------------------------------------

class CanvasPlot(object):
    """Rolling, auto-scaling multi-series plot drawn on a plain Tk Canvas."""

    SERIES = (("pos_mrad", "Position (mrad)", "#4fc3f7"),
              ("vel_mrad_s", "Speed (mrad/s)", "#81c784"),
              ("force_mN", "Force (mN)", "#ffb74d"),
              ("pwm_permille", "PWM (permille)", "#e57373"))

    BG = "#101418"
    GRID = "#2a3038"
    TEXT = "#9aa4b2"

    def __init__(self, canvas, model, window_s=10.0, width=660, height=280):
        # type: (object, TelemetryModel, float, int, int) -> None
        self.canvas = canvas
        self.model = model
        self.window_s = float(window_s)
        self.width = int(width)
        self.height = int(height)
        self.margin_l, self.margin_r, self.margin_t, self.margin_b = 56, 10, 26, 20

    def draw(self):
        # type: () -> None
        c = self.canvas
        c.delete("all")
        c.configure(bg=self.BG)
        w, h = self.width, self.height
        x0, x1 = self.margin_l, w - self.margin_r
        y0, y1 = self.margin_t, h - self.margin_b
        c.create_rectangle(x0, y0, x1, y1, outline=self.GRID)

        t_now = self.model.t_ms / 1000.0
        t_min = max(0.0, t_now - self.window_s)
        t_max = max(self.window_s, t_now)

        visible = []  # type: List[Tuple[str, List[Tuple[float, float]], str]]
        lo, hi = None, None
        for key, label, color in self.SERIES:
            pts = [(t, v) for t, v in self.model.series[key] if t >= t_min]
            visible.append((key, pts, color))
            for _t, v in pts:
                lo = v if lo is None else min(lo, v)
                hi = v if hi is None else max(hi, v)

        if lo is None:
            lo, hi = -1000.0, 1000.0
        if hi - lo < 1e-6:
            lo, hi = lo - 1.0, hi + 1.0
        pad = (hi - lo) * 0.12
        lo, hi = lo - pad, hi + pad

        def sx(t):
            return x0 + (t - t_min) / max(1e-9, (t_max - t_min)) * (x1 - x0)

        def sy(v):
            return y1 - (v - lo) / max(1e-9, (hi - lo)) * (y1 - y0)

        # horizontal grid + value labels (autoscaled)
        for i in range(5):
            frac = i / 4.0
            v = lo + frac * (hi - lo)
            y = sy(v)
            c.create_line(x0, y, x1, y, fill=self.GRID)
            c.create_text(x0 - 6, y, anchor="e", fill=self.TEXT, text="%d" % int(v),
                          font=("TkDefaultFont", 8))
        # vertical grid + time labels
        for i in range(5):
            frac = i / 4.0
            t = t_min + frac * (t_max - t_min)
            x = sx(t)
            c.create_line(x, y0, x, y1, fill=self.GRID)
            c.create_text(x, y1 + 10, anchor="n", fill=self.TEXT, text="%.1fs" % t,
                          font=("TkDefaultFont", 8))

        for key, pts, color in visible:
            if len(pts) < 2:
                continue
            flat = []  # type: List[float]
            for t, v in pts:
                flat.append(sx(t))
                flat.append(sy(v))
            c.create_line(*flat, fill=color, width=2, smooth=False)

        # legend, including the live value of each series
        lx = x0 + 4
        for key, _pts, color in visible:
            text = "%s: %d" % (dict((k, l) for k, l, _c in self.SERIES)[key],
                               int(getattr(self.model, key)))
            c.create_line(lx, y0 - 13, lx + 16, y0 - 13, fill=color, width=2)
            c.create_text(lx + 20, y0 - 13, anchor="w", fill=self.TEXT, text=text,
                          font=("TkDefaultFont", 8))
            lx += 24 + int(len(text) * 5.6)


class GuiMonitor(MonitorBase):
    """Tkinter dashboard.  Raises TkUnavailable when Tk cannot start."""

    REFRESH_MS = 100
    PLOT_EVERY = 4   # redraw the canvas every 4 ticks => 2.5 Hz

    def __init__(self, link, sim=None, csv_path=None):
        # type: (Link, Optional[object], Optional[str]) -> None
        if tk is None:
            raise TkUnavailable("tkinter is not usable: %s" % TK_IMPORT_ERROR)
        MonitorBase.__init__(self, link, sim=sim, csv_path=csv_path)
        try:
            self.root = tk.Tk()
        except Exception as exc:  # noqa: BLE001 - headless X / no display
            raise TkUnavailable("cannot create a Tk root window: %s: %s"
                                % (type(exc).__name__, exc))
        self.root.title("Tea-Picking Robot Monitor")
        self.root.configure(bg="#101418")
        self.root.minsize(940, 640)
        self._tick_count = 0
        self._deadline = None
        self._build_widgets()

    # -- layout -----------------------------------------------------------
    def _build_widgets(self):
        # type: () -> None
        root = self.root
        wrap = tk.Frame(root, bg="#101418")
        wrap.pack(fill="both", expand=True, padx=8, pady=8)

        left = tk.Frame(wrap, bg="#101418")
        left.grid(row=0, column=0, sticky="nw")
        right = tk.Frame(wrap, bg="#101418")
        right.grid(row=0, column=1, sticky="nw", padx=(10, 0))

        self.vars = {}  # type: Dict[str, object]
        readouts = [
            ("Position (mrad)", "pos_mrad"),
            ("Speed (mrad/s)", "vel_mrad_s"),
            ("Force (mN / N)", "force_text"),
            ("Robot state", "state_name"),
            ("Fault code", "fault_text"),
            ("PID output (permille)", "pwm_permille"),
            ("Motor current (mA)", "current_ma"),
            ("Cycle count", "cycle_count"),
            ("Link statistics", "link_text"),
            ("RX frames", "rx_text"),
        ]
        for row, (label, key) in enumerate(readouts):
            tk.Label(left, text=label + ":", fg="#9aa4b2", bg="#101418",
                     anchor="w", width=22).grid(row=row, column=0, sticky="w")
            var = tk.StringVar(value="-")
            self.vars[key] = var
            tk.Label(left, textvariable=var, fg="#e6edf3", bg="#101418",
                     anchor="w", width=34,
                     font=("Consolas", 11, "bold")).grid(row=row, column=1, sticky="w")

        self.canvas = tk.Canvas(right, width=660, height=280, bg=CanvasPlot.BG,
                                highlightthickness=0)
        self.canvas.pack()
        self.plot = CanvasPlot(self.canvas, self.model)

        controls = tk.Frame(wrap, bg="#101418")
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        buttons = tk.Frame(controls, bg="#101418")
        buttons.pack(anchor="w")
        self._button(buttons, "Start", self.on_start, "#1f6feb")
        self._button(buttons, "Stop", self.on_stop, "#8b949e")
        self._button(buttons, "Reset", self.on_reset, "#8b949e")
        self._button(buttons, "Emergency Stop", self.on_emergency_stop, "#b62324")
        self._button(buttons, "Clear Fault", self.on_clear_fault, "#8b949e")
        self._button(buttons, "Set Mode AUTO", self.on_set_mode_auto, "#8b949e")
        self._button(buttons, "Get Status", self.on_get_status, "#8b949e")
        self._button(buttons, "Get Sensor", self.on_get_sensor, "#8b949e")
        self._button(buttons, "Get Fault", self.on_get_fault, "#8b949e")

        entries = tk.Frame(controls, bg="#101418")
        entries.pack(anchor="w", pady=(8, 0))

        tk.Label(entries, text="Target Position (mm) x/y/z", fg="#9aa4b2",
                 bg="#101418").grid(row=0, column=0, sticky="w")
        self.e_x = self._entry(entries, "12", 1, 0)
        self.e_y = self._entry(entries, "-8", 2, 0)
        self.e_z = self._entry(entries, "180", 3, 0)
        tk.Label(entries, text="conf%", fg="#9aa4b2", bg="#101418").grid(row=0, column=4, sticky="w")
        self.e_conf = self._entry(entries, "92", 5, 0, width=4)
        tk.Label(entries, text="target id", fg="#9aa4b2", bg="#101418").grid(row=0, column=6, sticky="w")
        self.e_tid = self._entry(entries, "101", 7, 0, width=6)
        self._button(entries, "Send Target", self.on_send_target, "#1f6feb",
                     row=8, column=0, padx=(8, 0))

        tk.Label(entries, text="PID loop", fg="#9aa4b2", bg="#101418").grid(row=1, column=0,
                                                                            sticky="w", pady=(6, 0))
        self.loop_choice = tk.StringVar(value="0 (POSITION)")
        loops = tk.OptionMenu(entries, self.loop_choice, *["0 (POSITION)", "1 (FORCE)",
                                                           "2 (GRIPPER)"])
        loops.configure(bg="#21262d", fg="#e6edf3", highlightthickness=0)
        loops.grid(row=1, column=1, sticky="w", pady=(6, 0))
        tk.Label(entries, text="kp/ki/kd", fg="#9aa4b2", bg="#101418").grid(row=1, column=2,
                                                                            sticky="w", pady=(6, 0))
        self.e_kp = self._entry(entries, "2.4", 3, 1, width=7)
        self.e_ki = self._entry(entries, "0.35", 4, 1, width=7)
        self.e_kd = self._entry(entries, "0.05", 5, 1, width=7)
        self._button(entries, "Apply PID", self.on_apply_pid, "#1f6feb",
                     row=6, column=1, padx=(8, 0))

        tk.Label(entries, text="Force target (mN)", fg="#9aa4b2", bg="#101418").grid(
            row=2, column=0, sticky="w", pady=(6, 0))
        self.e_force = self._entry(entries, "1200", 1, 2, width=7)
        self._button(entries, "Apply Force", self.on_apply_force, "#1f6feb",
                     row=2, column=0, padx=(200, 0))

        logwrap = tk.Frame(wrap, bg="#101418")
        logwrap.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        tk.Label(logwrap, text="Event log", fg="#9aa4b2", bg="#101418").pack(anchor="w")
        self.log_text = tk.Text(logwrap, height=12, bg="#0b0f13", fg="#c9d1d9",
                                insertbackground="#c9d1d9", font=("Consolas", 9),
                                wrap="none")
        scroll = tk.Scrollbar(logwrap, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.attach_event_sink(self._append_log)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda _e: self.on_emergency_stop())

    def _button(self, parent, text, command, color, row=0, column=0, padx=(0, 6)):
        b = tk.Button(parent, text=text, command=command, bg=color, fg="#ffffff",
                      activebackground=color, activeforeground="#ffffff",
                      relief="flat", padx=8, pady=3)
        b.grid(row=row, column=column, padx=padx, pady=2, sticky="w")
        return b

    def _entry(self, parent, default, column, row, width=6):
        e = tk.Entry(parent, width=width, bg="#21262d", fg="#e6edf3",
                     insertbackground="#e6edf3")
        e.insert(0, default)
        e.grid(row=row, column=column, padx=2, sticky="w")
        return e

    def _append_log(self, entry):
        # type: (str) -> None
        self.log_text.insert("end", entry + "\n")
        # keep the pane bounded so long runs stay responsive
        if int(self.log_text.index("end-1c").split(".")[0]) > 800:
            self.log_text.delete("1.0", "200.0")
        self.log_text.see("end")

    # -- button handlers: thin wrappers over RobotController ---------------
    def _int(self, entry, name, default=0):
        # type: (object, str, int) -> int
        try:
            return int(float(entry.get().strip()))
        except (ValueError, AttributeError):
            self.log("bad %s value %r, using %d" % (name, entry.get(), default))
            return default

    def _float(self, entry, name, default=0.0):
        # type: (object, str, float) -> float
        try:
            return float(entry.get().strip())
        except (ValueError, AttributeError):
            self.log("bad %s value, using %.3f" % (name, default))
            return default

    def _guard(self, fn, *args):
        # type: (Callable, *object) -> None
        try:
            fn(*args)
        except LinkError as exc:
            self.log("command failed: %s" % exc)
        except ValueError as exc:
            self.log("bad command parameter: %s" % exc)

    def on_start(self):
        self._guard(self.controller.send_start, self._int(self.e_tid, "target id", 1))

    def on_stop(self):
        self._guard(self.controller.send_stop)

    def on_reset(self):
        self._guard(self.controller.send_reset)

    def on_emergency_stop(self):
        self._guard(self.controller.send_emergency_stop)

    def on_clear_fault(self):
        self._guard(self.controller.send_clear_fault)

    def on_set_mode_auto(self):
        self._guard(self.controller.send_set_mode, protocol.MODE_AUTO)

    def on_get_status(self):
        self._guard(self.controller.send_get_status)

    def on_get_sensor(self):
        self._guard(self.controller.send_get_sensor)

    def on_get_fault(self):
        self._guard(self.controller.send_get_fault)

    def on_send_target(self):
        self._guard(self.controller.send_set_target,
                    self._int(self.e_x, "x"), self._int(self.e_y, "y"),
                    self._int(self.e_z, "z"), self._int(self.e_conf, "confidence", 90),
                    self._int(self.e_tid, "target id", 1))

    def on_apply_pid(self):
        loop = int(str(self.loop_choice.get()).split()[0])
        self._guard(self.controller.send_set_pid, loop,
                    self._float(self.e_kp, "kp", 0.0),
                    self._float(self.e_ki, "ki", 0.0),
                    self._float(self.e_kd, "kd", 0.0))

    def on_apply_force(self):
        self._guard(self.controller.send_set_force_target,
                    self._int(self.e_force, "force", 1000))

    def on_close(self):
        # type: () -> None
        self.log("window closed by user")
        self.running = False
        try:
            self.root.destroy()
        except Exception:  # noqa: BLE001
            pass

    # -- rendering ---------------------------------------------------------
    def refresh_readouts(self):
        # type: () -> None
        m = self.model
        self.vars["pos_mrad"].set("%d" % m.pos_mrad)
        self.vars["vel_mrad_s"].set("%d" % m.vel_mrad_s)
        self.vars["force_text"].set("%d  (%.3f N)" % (m.force_mN, m.force_n))
        self.vars["state_name"].set(m.state_name)
        self.vars["fault_text"].set(m.fault_name if m.fault == FAULT.NONE
                                    else "%s  <-- ACTIVE" % m.fault_name)
        self.vars["pwm_permille"].set("%d" % m.pwm_permille)
        self.vars["current_ma"].set("%d" % m.current_ma)
        self.vars["cycle_count"].set("%d" % m.cycle_count)
        self.vars["link_text"].set(self.link.stats.summary())
        self.vars["rx_text"].set(str(self.counts["rx_frames"]))

    def tick(self):
        # type: () -> None
        now = time.time()
        self._tick_count += 1
        self.pump(now)
        self.step_demo(now)
        self.refresh_readouts()
        if self._tick_count % self.PLOT_EVERY == 0:
            self.plot.draw()
        if self._deadline is not None and now >= self._deadline:
            self._finish()
            return
        self.root.after(self.REFRESH_MS, self.tick)

    def _finish(self):
        # type: () -> None
        if not self.running:
            return
        self.log("duration elapsed; closing")
        self.plot.draw()
        try:
            self.root.after(300, self.on_close)
        except Exception:  # noqa: BLE001
            pass

    def run(self, duration=0.0):
        # type: (float) -> int
        self.start()
        self.refresh_readouts()
        self.plot.draw()
        if duration > 0:
            self._deadline = time.time() + duration
        try:
            self.root.after(self.REFRESH_MS, self.tick)
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
        return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_arg_parser():
    # type: () -> argparse.ArgumentParser
    p = argparse.ArgumentParser(
        prog="robot_monitor.py",
        description="PC monitoring / debugging tool for the tea-picking robot.")
    g = p.add_argument_group("transport (default: --demo)")
    g.add_argument("--port", metavar="COM5", help="serial port, e.g. COM5")
    g.add_argument("--tcp", metavar="HOST:PORT", help="TCP bridge, e.g. 127.0.0.1:9000")
    g.add_argument("--demo", action="store_true",
                   help="use host.sim_robot + LoopbackLink (no hardware needed)")
    g.add_argument("--baud", type=int, default=921600, help="serial baud (default 921600)")
    g = p.add_argument_group("front end")
    g.add_argument("--nogui", action="store_true",
                   help="headless text dashboard (works without a display)")
    g.add_argument("--duration", type=float, default=0.0,
                   help="run for S seconds then exit (0 = until closed / Ctrl-C)")
    g.add_argument("--csv", metavar="PATH", help="log received telemetry to CSV")
    g.add_argument("--send-demo-sequence", action="store_true", dest="demo_sequence",
                   help="drive a full pick cycle through the same code path as the buttons")
    g.add_argument("--render-interval", type=float, default=1.0, dest="render_interval",
                   help="headless dashboard refresh period in seconds (default 1.0)")
    g.add_argument("--seed", type=int, default=1, help="simulation seed (default 1)")
    return p


def build_transport(args):
    # type: (argparse.Namespace) -> Tuple[Link, Optional[object], str]
    """Return ``(link, sim_or_None, description)``."""
    port = getattr(args, "port", None)
    tcp = getattr(args, "tcp", None)
    if port:
        return open_link("serial", port, baud=args.baud), None, "serial %s" % port
    if tcp:
        host, p = parse_hostport(tcp)
        return open_link("tcp", tcp), None, "tcp %s:%d" % (host, p)
    if not getattr(args, "demo", False):
        sys.stdout.write("no --port/--tcp given; defaulting to --demo (simulated robot)\n")
    from host.sim_robot import SimRobot
    sim = SimRobot(seed=args.seed)
    link = LoopbackLink(responder=sim.feed)
    link.open()
    return link, sim, "loopback + SimRobot(seed=%d)" % args.seed


def main(argv=None):
    # type: (Optional[List[str]]) -> int
    args = build_arg_parser().parse_args(argv)
    try:
        link, sim, transport = build_transport(args)
    except (LinkError, ValueError) as exc:
        sys.stderr.write("transport error: %s\n" % exc)
        return 2

    use_gui = not args.nogui
    monitor = None  # type: Optional[MonitorBase]
    if use_gui:
        if tk is None:
            sys.stdout.write("tkinter unavailable (%s); falling back to --nogui\n"
                             % TK_IMPORT_ERROR)
            use_gui = False
        else:
            try:
                monitor = GuiMonitor(link, sim=sim, csv_path=args.csv)
            except TkUnavailable as exc:
                sys.stdout.write("%s\nfalling back to --nogui\n" % exc)
                use_gui = False

    if not use_gui:
        monitor = HeadlessMonitor(link, sim=sim, csv_path=args.csv,
                                  render_interval=args.render_interval)

    assert monitor is not None
    sys.stdout.write("monitor transport: %s   mode: %s\n"
                     % (transport, "GUI" if use_gui else "headless"))
    sys.stdout.flush()
    if args.demo_sequence:
        monitor.enable_demo_sequence()

    exit_code = 0
    try:
        monitor.run(args.duration)
    except LinkError as exc:
        sys.stderr.write("link error: %s\n" % exc)
        exit_code = 3
    finally:
        monitor.close()
    sys.stdout.write(monitor.summary() + "\n")
    sys.stdout.flush()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
