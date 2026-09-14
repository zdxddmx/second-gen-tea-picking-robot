# -*- coding: utf-8 -*-
"""tools/analyze_log.py -- telemetry CSV / raw hex frame log analyser.

Inputs
------
1. a telemetry CSV written by ``host/robot_monitor.py --csv`` (or by this tool's
   ``--selftest``) with the schema::

       t_ms,pos_mrad,vel_mrad_s,force_mN,pwm_permille,state,fault

2. a raw hex frame log: any text file containing hex bytes (with or without a
   leading ``0x``/``AA 55`` context).  ``TELEMETRY`` frames found in it are
   extracted into the same row schema; other frames are counted.

Reports
-------
duration, sample count, effective sample rate, min/max/mean of every signal,
step-response metrics (rise time 10..90%, overshoot %, settling time to +/-2%,
steady-state error), peak force, fault occurrences with timestamps, and a state
dwell-time histogram.  ``--json`` emits the same data machine-readably.

Examples::

    python tools/analyze_log.py data/logs/monitor_selftest.csv
    python tools/analyze_log.py raw_hex.log --json
    python tools/analyze_log.py --selftest

Python 3.8 compatible.
"""

import argparse
import json
import math
import os
import random
import statistics
import sys
import tempfile
import time
from typing import Dict, List, Optional, Tuple

if __package__ in (None, ""):  # allow ``python tools/analyze_log.py``
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import protocol
from host.protocol import CMD, FrameParser

__all__ = ["Sample", "load_csv", "load_hex_log", "analyse", "main"]

CSV_COLUMNS = ["t_ms", "pos_mrad", "vel_mrad_s", "force_mN", "pwm_permille",
               "state", "fault"]

SIGNALS = ("pos_mrad", "vel_mrad_s", "force_mN", "pwm_permille")


class Sample(object):
    """One telemetry row."""

    __slots__ = ("t_ms", "pos_mrad", "vel_mrad_s", "force_mN", "pwm_permille",
                 "state", "fault")

    def __init__(self, t_ms, pos_mrad, vel_mrad_s, force_mN, pwm_permille, state, fault):
        self.t_ms = int(t_ms)
        self.pos_mrad = int(pos_mrad)
        self.vel_mrad_s = int(vel_mrad_s)
        self.force_mN = int(force_mN)
        self.pwm_permille = int(pwm_permille)
        self.state = int(state)
        self.fault = int(fault)

    def __repr__(self):
        return ("Sample(t=%d, pos=%d, vel=%d, force=%d, pwm=%d, state=%s, fault=%s)"
                % (self.t_ms, self.pos_mrad, self.vel_mrad_s, self.force_mN,
                   self.pwm_permille, protocol.state_name(self.state),
                   protocol.fault_name(self.fault)))


# --------------------------------------------------------------------------
# Loaders
# --------------------------------------------------------------------------

class LogError(Exception):
    """Raised when an input file cannot be interpreted."""


def _looks_like_header(first_line):
    # type: (str) -> bool
    low = first_line.strip().lower()
    return low.startswith("t_ms") or ("," in low and not low[0].isdigit()
                                      and not low[0] == "-")


def load_csv(path):
    # type: (str) -> List[Sample]
    """Read the telemetry CSV schema.  Tolerates a header and blank lines."""
    if not os.path.isfile(path):
        raise LogError("no such file: %s" % path)
    rows = []  # type: List[Sample]
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        first = True
        for lineno, line in enumerate(fh, 1):
            text = line.strip()
            if not text:
                continue
            if first and _looks_like_header(text):
                first = False
                continue
            first = False
            parts = [p.strip() for p in text.replace(";", ",").split(",")]
            if len(parts) < len(CSV_COLUMNS):
                raise LogError("line %d: expected %d columns, got %d"
                               % (lineno, len(CSV_COLUMNS), len(parts)))
            try:
                values = [float(p) for p in parts[:len(CSV_COLUMNS)]]
            except ValueError:
                raise LogError("line %d: non-numeric field in %r" % (lineno, text))
            rows.append(Sample(*[int(round(v)) for v in values]))
    if not rows:
        raise LogError("no data rows found in %s" % path)
    return rows


def _iter_hex_bytes(text):
    # type: (str) -> object
    """Yield ints from a text blob full of hex tokens (``AA 55 10 09``, ``0xAA``).

    Comment lines (``#`` or ``//``) and trailing annotations after ``#``/``|``
    are ignored, so the output of ``vision_mock.py --stdout`` and of
    ``robot_monitor.py`` can be fed in directly.
    """
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("//"):
            continue
        for marker in ("|", "#", ";"):
            idx = stripped.find(marker)
            if idx >= 0:
                stripped = stripped[:idx]
        token = ""
        for ch in stripped:
            if ch in "0123456789abcdefABCDEF":
                token += ch
            else:
                if len(token) == 2:
                    yield int(token, 16)
                token = ""
        if len(token) == 2:
            yield int(token, 16)


def load_hex_log(path):
    # type: (str) -> Tuple[List[Sample], Dict[str, int]]
    """Extract TELEMETRY rows from a raw hex frame log."""
    if not os.path.isfile(path):
        raise LogError("no such file: %s" % path)
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        blob = fh.read()
    parser = FrameParser()
    counts = {}  # type: Dict[str, int]
    rows = []  # type: List[Sample]
    t0 = None  # type: Optional[int]
    for byte in _iter_hex_bytes(blob):
        for frame in parser.feed(bytes(bytearray([byte]))):
            name = protocol.cmd_name(frame.cmd)
            counts[name] = counts.get(name, 0) + 1
            if frame.cmd == CMD.TELEMETRY:
                t = protocol.decode_telemetry(frame.payload)
                if t0 is None:
                    t0 = t.t_ms
                rows.append(Sample(t.t_ms - t0, t.pos_mrad, t.vel_mrad_s, t.force_mN,
                                   t.pwm_permille, t.state, t.fault))
    if not rows:
        raise LogError("no TELEMETRY frames found in %s (%d frames parsed in total)"
                       % (path, sum(counts.values())))
    counts["_crc_errors"] = parser.frames_crc_error
    counts["_bytes_discarded"] = parser.bytes_discarded
    return rows, counts


def load_any(path):
    # type: (str) -> Tuple[List[Sample], str, Dict[str, int]]
    """Auto-detect CSV vs hex log."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        head = fh.read(4096)
    if "," in head and any(col in head for col in ("t_ms", "pos_mrad", "force_mN")):
        return load_csv(path), "csv", {}
    if "AA 55" in head.upper() or "aa55" in head.lower():
        rows, counts = load_hex_log(path)
        return rows, "hex", counts
    # last resort: try CSV, then hex
    try:
        return load_csv(path), "csv", {}
    except LogError:
        rows, counts = load_hex_log(path)
        return rows, "hex", counts


# --------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------

def _stats(values):
    # type: (List[float]) -> Dict[str, float]
    if not values:
        return {"min": 0.0, "max": 0.0, "mean": 0.0, "std": 0.0}
    return {"min": min(values), "max": max(values),
            "mean": statistics.mean(values),
            "std": statistics.pstdev(values) if len(values) > 1 else 0.0}


def state_dwell(samples):
    # type: (List[Sample]) -> Dict[str, Dict[str, float]]
    """Time spent in each robot state (last sample's dwell is not extrapolated)."""
    out = {}  # type: Dict[str, Dict[str, float]]
    if not samples:
        return out
    current = samples[0].state
    start = samples[0].t_ms
    for i in range(1, len(samples)):
        s = samples[i]
        if s.state != current:
            name = protocol.state_name(current)
            entry = out.setdefault(name, {"seconds": 0.0, "entries": 0})
            entry["seconds"] += (s.t_ms - start) / 1000.0
            entry["entries"] += 1
            current, start = s.state, s.t_ms
    # close the final interval at the last timestamp
    name = protocol.state_name(current)
    entry = out.setdefault(name, {"seconds": 0.0, "entries": 0})
    entry["seconds"] += (samples[-1].t_ms - start) / 1000.0
    entry["entries"] += 1
    return out


def fault_occurrences(samples):
    # type: (List[Sample]) -> List[Dict[str, object]]
    """Contiguous runs where fault != 0, with start/end timestamps."""
    out = []  # type: List[Dict[str, object]]
    run = None  # type: Optional[Dict[str, object]]
    for s in samples:
        if s.fault != 0:
            if run is None:
                run = {"fault": s.fault, "name": protocol.fault_name(s.fault),
                       "t_start_ms": s.t_ms, "t_end_ms": s.t_ms, "samples": 1,
                       "state": protocol.state_name(s.state)}
            else:
                run["t_end_ms"] = s.t_ms
                run["samples"] = int(run["samples"]) + 1
        elif run is not None:
            out.append(run)
            run = None
    if run is not None:
        out.append(run)
    for entry in out:
        entry["t_start_s"] = round(int(entry["t_start_ms"]) / 1000.0, 4)
        entry["duration_s"] = round(
            (int(entry["t_end_ms"]) - int(entry["t_start_ms"])) / 1000.0, 4)
    return out


def step_response(samples, setpoint=None, settle_band=0.02):
    # type: (List[Sample], Optional[float], float) -> Dict[str, object]
    """Step-response metrics of the position trace.

    The step is located as the first excursion of ``pos_mrad`` beyond 2 % of the
    total travel.  ``steady_state_error`` is only meaningful when ``setpoint`` is
    given; otherwise ``residual_ripple`` (std of the final 10 %) is reported and
    ``steady_state_error`` is ``None``.
    """
    out = {"available": False, "reason": None}  # type: Dict[str, object]
    if len(samples) < 10:
        out["reason"] = "need at least 10 samples"
        return out
    t = [(s.t_ms - samples[0].t_ms) / 1000.0 for s in samples]
    y = [float(s.pos_mrad) for s in samples]

    head = statistics.mean(y[:max(1, len(y) // 10)])
    tail = statistics.mean(y[-max(1, len(y) // 10):])
    travel = tail - head
    if abs(travel) < 1.0:
        out["reason"] = "no detectable step (travel %.3f mrad)" % travel
        return out

    direction = 1.0 if travel > 0 else -1.0
    lo_thr = head + 0.10 * travel
    hi_thr = head + 0.90 * travel

    def first_beyond(threshold):
        for i, v in enumerate(y):
            if direction * (v - threshold) >= 0.0:
                return i
        return None

    i10 = first_beyond(lo_thr)
    i90 = first_beyond(hi_thr)
    i0 = None
    for i, v in enumerate(y):
        if abs(v - head) > 0.02 * abs(travel):
            i0 = i
            break
    if i0 is None or i10 is None or i90 is None or i90 <= i10:
        out["reason"] = "incomplete edge (could not find 10% and 90% crossings)"
        return out

    t0 = t[i0]
    rise = t[i90] - t[i10]

    peak = max(y) if direction > 0 else min(y)
    overshoot = 0.0
    if abs(travel) > 1e-9:
        overshoot = max(0.0, direction * (peak - tail) / abs(travel) * 100.0)

    band = abs(travel) * settle_band
    i_settle = i90
    for i in range(len(y) - 1, i90 - 1, -1):
        if abs(y[i] - tail) > band:
            i_settle = min(len(y) - 1, i + 1)
            break
    settle = t[i_settle] - t0

    final_mean = statistics.mean(y[-max(1, len(y) // 10):])
    ripple = statistics.pstdev(y[-max(1, len(y) // 10):]) if len(y) > 10 else 0.0

    out.update({
        "available": True,
        "initial_value": round(head, 3),
        "final_value": round(final_mean, 3),
        "step_amplitude": round(travel, 3),
        "step_start_s": round(t0, 4),
        "t10_s": round(t[i10], 4),
        "t90_s": round(t[i90], 4),
        "rise_time_s": round(rise, 4),
        "peak_value": round(float(peak), 3),
        "overshoot_percent": round(overshoot, 3),
        "settling_time_s": round(settle, 4),
        "settling_band_percent": settle_band * 100.0,
        "setpoint": None if setpoint is None else round(float(setpoint), 3),
        "steady_state_error": (None if setpoint is None
                               else round(abs(setpoint - final_mean), 3)),
        "residual_ripple": round(ripple, 3),
    })
    return out


def analyse(samples, setpoint=None, settle_band=0.02):
    # type: (List[Sample], Optional[float], float) -> Dict[str, object]
    """Compute the full metric report."""
    if not samples:
        raise LogError("no samples to analyse")
    t_ms = [s.t_ms for s in samples]
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    dt = [(t_ms[i] - t_ms[i - 1]) for i in range(1, len(t_ms))]
    effective_rate = ((len(samples) - 1) / duration_s) if duration_s > 0 else 0.0

    signals = {}  # type: Dict[str, Dict[str, float]]
    for name in SIGNALS:
        signals[name] = _stats([float(getattr(s, name)) for s in samples])

    peak = max(samples, key=lambda s: abs(s.force_mN))
    report = {
        "samples": len(samples),
        "duration_s": round(duration_s, 4),
        "effective_sample_rate_hz": round(effective_rate, 3),
        "sample_interval_ms": {
            "min": min(dt) if dt else 0,
            "max": max(dt) if dt else 0,
            "mean": round(statistics.mean(dt), 3) if dt else 0.0,
        },
        "signals": dict((k, dict((kk, round(vv, 4)) for kk, vv in v.items()))
                        for k, v in signals.items()),
        "peak_force_mN": peak.force_mN,
        "peak_force_N": round(peak.force_mN / 1000.0, 4),
        "peak_force_at_s": round((peak.t_ms - t_ms[0]) / 1000.0, 4),
        "step_response": step_response(samples, setpoint=setpoint,
                                       settle_band=settle_band),
        "faults": fault_occurrences(samples),
        "state_dwell": dict((k, {"seconds": round(v["seconds"], 4),
                                 "entries": v["entries"],
                                 "percent": round(100.0 * v["seconds"] / duration_s, 2)
                                 if duration_s > 0 else 0.0})
                            for k, v in state_dwell(samples).items()),
        "states_seen": [protocol.state_name(code) for code in
                        sorted(set(x.state for x in samples))],
    }
    return report


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------

def _fmt_stats(name, st):
    # type: (str, Dict[str, float]) -> List[str]
    return ["  %-14s min %10.2f   max %10.2f   mean %10.2f   std %9.2f"
            % (name, st["min"], st["max"], st["mean"], st["std"])]


def render(report, source, kind, frame_counts=None, setpoint=None):
    # type: (Dict[str, object], str, str, Optional[Dict[str, int]], Optional[float]) -> str
    lines = []  # type: List[str]
    rule = "-" * 74
    lines.append("=" * 74)
    lines.append("TELEMETRY LOG ANALYSIS")
    lines.append("=" * 74)
    lines.append("source           : %s (%s)" % (source, kind))
    lines.append("duration         : %.4f s" % report["duration_s"])
    lines.append("samples          : %d" % report["samples"])
    lines.append("effective rate   : %.3f Hz" % report["effective_sample_rate_hz"])
    iv = report["sample_interval_ms"]  # type: ignore[index]
    lines.append("sample interval  : min %s ms  max %s ms  mean %.3f ms"
                 % (iv["min"], iv["max"], iv["mean"]))
    if frame_counts:
        parsed = sorted((k, v) for k, v in frame_counts.items() if not k.startswith("_"))
        lines.append("hex frames       : " + ", ".join("%s=%d" % kv for kv in parsed))
        if "_crc_errors" in frame_counts:
            lines.append("hex crc errors   : %d (bytes discarded %d)"
                         % (frame_counts["_crc_errors"], frame_counts["_bytes_discarded"]))
    lines.append(rule)
    lines.append("SIGNAL STATISTICS")
    for name in SIGNALS:
        lines.extend(_fmt_stats(name, report["signals"][name]))  # type: ignore[index]
    lines.append(rule)
    lines.append("FORCE")
    lines.append("  peak force     : %d mN (%.3f N) at t=%.3f s"
                 % (report["peak_force_mN"], report["peak_force_N"],
                    report["peak_force_at_s"]))
    lines.append(rule)
    lines.append("STEP RESPONSE (position)")
    step = report["step_response"]  # type: ignore[assignment]
    if step.get("available"):
        lines.append("  initial value  : %.3f mrad" % step["initial_value"])
        lines.append("  final value    : %.3f mrad" % step["final_value"])
        lines.append("  step amplitude : %.3f mrad" % step["step_amplitude"])
        lines.append("  step start     : %.4f s" % step["step_start_s"])
        lines.append("  rise time      : %.4f s  (10%% -> 90%%)" % step["rise_time_s"])
        lines.append("  overshoot      : %.3f %%" % step["overshoot_percent"])
        lines.append("  settling time  : %.4f s  (+/-%.1f%%)"
                     % (step["settling_time_s"], step["settling_band_percent"]))
        if step["steady_state_error"] is None:
            lines.append("  steady-state   : n/a (pass --setpoint MM to compute)")
        else:
            lines.append("  steady-state   : %.3f mrad error vs setpoint %s"
                         % (step["steady_state_error"], step.get("setpoint")))
        lines.append("  residual ripple: %.3f mrad (final 10%%)" % step["residual_ripple"])
    else:
        lines.append("  not available  : %s" % step.get("reason"))
    lines.append(rule)
    lines.append("FAULTS")
    faults = report["faults"]  # type: ignore[assignment]
    if not faults:
        lines.append("  none recorded")
    else:
        for f in faults:
            lines.append("  t=%.4f s  %-20s duration %.4f s  (%d samples, state %s)"
                         % (f["t_start_s"], f["name"], f["duration_s"],
                            f["samples"], f["state"]))
    lines.append(rule)
    lines.append("STATE DWELL TIME")
    dwell = report["state_dwell"]  # type: ignore[assignment]
    for name in sorted(dwell, key=lambda k: -dwell[k]["seconds"]):
        entry = dwell[name]
        lines.append("  %-18s %8.4f s  %6.2f %%  (%d entries)"
                     % (name, entry["seconds"], entry["percent"], entry["entries"]))
    lines.append("=" * 74)
    return "\n".join(lines)


# --------------------------------------------------------------------------
# Self test
# --------------------------------------------------------------------------

def _make_selftest_csv(path, setpoint=1000.0):
    # type: (str, float) -> None
    """Write a synthetic first-order step response with a short fault burst."""
    rng = random.Random(20240517)
    rows = []
    pos = 0.0
    tau = 0.25
    for i in range(600):
        t = i * 0.01
        target = setpoint if t >= 1.0 else 0.0
        pos = pos + (target - pos) * (0.01 / tau)   # explicit first-order step
        vel = (target - pos) / tau
        fault = protocol.FAULT.FORCE_OVERLOAD if 2.00 <= t < 2.10 else 0
        state = protocol.STATE.IDLE if t < 1.0 else (
            protocol.STATE.ERROR if fault else protocol.STATE.APPROACH)
        force = 0 if t < 1.0 else int(400 + 120 * math.sin(t * 6.0))
        pwm = int(max(-1000, min(1000, vel * 0.5)))
        rows.append("%d,%d,%d,%d,%d,%d,%d"
                    % (int(t * 1000), int(pos + rng.gauss(0, 1.5)), int(vel),
                       force, pwm, state, fault))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(",".join(CSV_COLUMNS) + "\n")
        fh.write("\n".join(rows) + "\n")


def selftest(verbose=True):
    # type: (bool) -> bool
    """Generate a CSV in a temp dir, analyse it, and check the metrics."""
    problems = []  # type: List[str]
    tmpdir = tempfile.mkdtemp(prefix="tea_analyze_")
    csv_path = os.path.join(tmpdir, "selftest_telemetry.csv")
    hex_path = os.path.join(tmpdir, "selftest_frames.log")
    setpoint = 1000.0
    _make_selftest_csv(csv_path, setpoint=setpoint)

    # a raw hex log carrying the same samples as TELEMETRY frames
    samples = load_csv(csv_path)
    with open(hex_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# raw hex frame log generated by tools/analyze_log.py --selftest\n")
        for s in samples:
            frame = protocol.build_frame(CMD.TELEMETRY, protocol.encode_telemetry(
                s.t_ms, s.pos_mrad, s.vel_mrad_s, s.force_mN, s.pwm_permille,
                s.state, s.fault))
            fh.write("%s\n" % " ".join("%02X" % b for b in bytearray(frame)))

    report = analyse(samples, setpoint=setpoint)
    step = report["step_response"]
    checks = [
        ("csv loads", report["samples"] == 600),
        ("duration", abs(report["duration_s"] - 5.99) < 1e-6),
        ("effective rate ~100 Hz", abs(report["effective_sample_rate_hz"] - 100.0) < 1.0),
        ("position reaches setpoint",
         abs(step.get("final_value", 0.0) - setpoint) < 20.0),
        ("step detected", bool(step.get("available"))),
        ("rise time plausible",
         0.1 < float(step.get("rise_time_s", 0.0)) < 1.5),
        ("overshoot small (first order)",
         float(step.get("overshoot_percent", 99.0)) < 5.0),
        ("settling inside window",
         0.0 < float(step.get("settling_time_s", 0.0)) < 3.0),
        ("steady-state error small",
         float(step.get("steady_state_error") or 99.0) < 20.0),
        ("peak force positive", report["peak_force_mN"] > 0),
        ("fault detected", len(report["faults"]) == 1),
        ("fault code correct",
         bool(report["faults"]) and report["faults"][0]["fault"]
         == protocol.FAULT.FORCE_OVERLOAD),
        ("fault timestamp ~2.0 s",
         bool(report["faults"]) and abs(report["faults"][0]["t_start_s"] - 2.0) < 0.05),
        ("state dwell covers ATTACH/IDLE",
         "IDLE" in report["state_dwell"] and "APPROACH" in report["state_dwell"]),
        ("dwell percentages sum to ~100",
         abs(sum(v["percent"] for v in report["state_dwell"].values()) - 100.0) < 0.5),
    ]

    hex_rows, kind, counts = load_any(hex_path)
    hex_report = analyse(hex_rows)
    checks.append(("hex log round-trip", kind == "hex" and hex_report["samples"] == 600))
    checks.append(("hex frames counted",
                   counts.get("TELEMETRY") == 600 and counts.get("_crc_errors") == 0))

    json_text = json.dumps(report, indent=2, sort_keys=True)
    checks.append(("json serialisable", len(json_text) > 200))

    for label, ok in checks:
        if not ok:
            problems.append(label)

    if verbose:
        print(render(report, csv_path, "csv (selftest)"))
        print("--- selftest checks ---")
        for label, ok in checks:
            print("[%s] %s" % ("PASS" if ok else "FAIL", label))
        print("hex log: %d TELEMETRY frames, kind=%s" % (hex_report["samples"], kind))
    try:
        os.remove(csv_path)
        os.remove(hex_path)
        os.rmdir(tmpdir)
    except OSError:
        pass
    if problems:
        if verbose:
            print("SELFTEST FAILED: %s" % ", ".join(problems))
        return False
    if verbose:
        print("SELFTEST PASS (%d checks)" % len(checks))
    return True


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_arg_parser():
    # type: () -> argparse.ArgumentParser
    p = argparse.ArgumentParser(
        prog="analyze_log.py",
        description="Analyse a robot telemetry CSV or a raw hex frame log.")
    p.add_argument("path", nargs="?", help="telemetry CSV or raw hex frame log")
    p.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    p.add_argument("--setpoint", type=float, default=None,
                   help="position setpoint in mrad, enables the steady-state error")
    p.add_argument("--settle-band", type=float, default=0.02, dest="settle_band",
                   help="settling band as a fraction of travel (default 0.02 = 2%%)")
    p.add_argument("--csv", metavar="PATH", help="write the metric table to PATH as JSON")
    p.add_argument("--selftest", action="store_true",
                   help="generate a synthetic CSV in a temp dir and verify the analysis")
    return p


def main(argv=None):
    # type: (Optional[List[str]]) -> int
    args = build_arg_parser().parse_args(argv)

    if args.selftest:
        return 0 if selftest(verbose=True) else 1

    if not args.path:
        sys.stderr.write("error: give a log path, or use --selftest\n")
        return 2
    if not (0.0 < args.settle_band < 1.0):
        sys.stderr.write("error: --settle-band must be within 0..1\n")
        return 2

    try:
        samples, kind, counts = load_any(args.path)
    except LogError as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 2

    report = analyse(samples, setpoint=args.setpoint, settle_band=args.settle_band)
    report["source"] = os.path.abspath(args.path)
    report["source_kind"] = kind
    if counts:
        report["frame_counts"] = counts

    if args.json:
        sys.stdout.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    else:
        sys.stdout.write(render(report, args.path, kind,
                                frame_counts=counts, setpoint=args.setpoint) + "\n")

    if args.csv:
        directory = os.path.dirname(os.path.abspath(args.csv))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(args.csv, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
        sys.stdout.write("metrics written to %s\n" % args.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
