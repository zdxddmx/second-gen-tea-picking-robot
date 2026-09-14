# -*- coding: utf-8 -*-
"""host/selftest.py -- one-shot programmatic test suite for the PC-side tools.

Runs the documented acceptance commands as real subprocesses from the project
root and prints a PASS/FAIL summary.  Intended to be called by the main test
suite (which may not have ``pytest`` available)::

    python host/selftest.py             # summary only
    python host/selftest.py --verbose   # also dump each child's output
    python host/selftest.py --only 4    # run a single check

Exit code 0 only when every check passes.

Python 3.8 compatible; no third-party imports.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from typing import Callable, List, Optional, Tuple

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


class Result(object):
    """Outcome of a single check."""

    def __init__(self, name, ok, detail, command="", output="", seconds=0.0):
        self.name = name
        self.ok = bool(ok)
        self.detail = detail
        self.command = command
        self.output = output
        self.seconds = float(seconds)


def run(args, timeout=120.0, env=None):
    # type: (List[str], float, Optional[dict]) -> Tuple[int, str, float]
    """Run ``python args...`` from the project root; return (rc, output, seconds)."""
    cmd = [sys.executable] + list(args)
    merged = dict(os.environ)
    if env:
        merged.update(env)
    started = time.time()
    try:
        proc = subprocess.run(cmd, cwd=PROJECT_ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=timeout, env=merged)
        output = proc.stdout.decode("utf-8", "replace")
        return proc.returncode, output, time.time() - started
    except subprocess.TimeoutExpired:
        return 124, "<timeout after %.0f s>" % timeout, time.time() - started
    except OSError as exc:
        return 125, "<cannot run %r: %s>" % (cmd, exc), time.time() - started


# --------------------------------------------------------------------------
# Individual checks
# --------------------------------------------------------------------------

def check_1_protocol():
    # type: () -> Result
    cmd = ["host/protocol.py"]
    rc, out, secs = run(cmd, timeout=30)
    ok = rc == 0 and "RESULT: PASS" in out and "[FAIL]" not in out
    detail = "protocol self test (CRC vector + round-trip)"
    if "RESULT: PASS" not in out:
        detail += " -- expected 'RESULT: PASS'"
    return Result("1. protocol codec self test", ok, detail, "python " + " ".join(cmd),
                  out, secs)


def check_2_vision_stdout():
    # type: () -> Result
    cmd = ["tools/vision_mock.py", "--stdout", "--count", "20", "--seed", "1"]
    rc, out, secs = run(cmd, timeout=60)
    frame_lines = [ln for ln in out.splitlines() if ln.startswith("AA 55")]
    sent = _find_int(out, r"frames sent\s*:\s*(\d+)")
    droppped = _find_int(out, r"frames dropped\s*:\s*(\d+)")
    ok = (rc == 0 and len(frame_lines) == 20 and sent == 20 and droppped == 0
          and "frames generated : 20" in out)
    detail = "20 hex frames printed, tx/drop summary present (sent=%s dropped=%s)" % (
        sent, droppped)
    return Result("2. vision mock --stdout 20 frames", ok, detail,
                  "python " + " ".join(cmd), out, secs)


def check_3_vision_loss():
    # type: () -> Result
    cmd = ["tools/vision_mock.py", "--stdout", "--count", "20", "--seed", "1",
           "--loss", "0.2", "--quiet"]
    rc, out, secs = run(cmd, timeout=60)
    generated = _find_int(out, r"frames generated\s*:\s*(\d+)")
    sent = _find_int(out, r"frames sent\s*:\s*(\d+)")
    dropped = _find_int(out, r"frames dropped\s*:\s*(\d+)")
    frame_lines = [ln for ln in out.splitlines() if ln.startswith("AA 55")]
    pct = (100.0 * dropped / generated) if generated else 0.0
    # 20 trials at p=0.2: accept anything in 5%..40% as "~20%"
    ok = (rc == 0 and generated == 20 and dropped == generated - sent
          and dropped > 0 and 5.0 <= pct <= 40.0 and len(frame_lines) == sent
          and "|" not in out)   # --quiet must suppress decoded summaries
    detail = ("generated=%s sent=%s dropped=%s (%.1f%%), %d hex lines, decoded "
              "summaries suppressed" % (generated, sent, dropped, pct, len(frame_lines)))
    return Result("3. vision mock --loss 0.2 really drops", ok, detail,
                  "python " + " ".join(cmd), out, secs)


def check_4_monitor_demo():
    # type: () -> Result
    cmd = ["host/robot_monitor.py", "--demo", "--nogui", "--duration", "5",
           "--send-demo-sequence"]
    rc, out, secs = run(cmd, timeout=120)
    cycles = _find_int(out, r"sim cycle count\s*:\s*(\d+)")
    steps = _find_int(out, r"demo steps run\s*:\s*(\d+)/(\d+)")
    ok = (rc == 0 and cycles >= 1 and steps >= 11
          and "TEA-PICKING ROBOT MONITOR" in out
          and "demo step 8/11: START_PICK" in out
          and "final state      : IDLE" in out)
    detail = ("headless dashboard rendered, demo sequence completed a pick cycle "
              "(sim cycle count=%s, demo steps=%s)" % (cycles, steps))
    return Result("4. monitor --demo --nogui pick cycle", ok, detail,
                  "python " + " ".join(cmd), out, secs)


def check_5_monitor_csv_to_analysis():
    # type: () -> Result
    csv_path = os.path.join("data", "logs", "monitor_selftest.csv")
    cmd1 = ["host/robot_monitor.py", "--demo", "--nogui", "--duration", "3",
            "--csv", csv_path]
    rc1, out1, secs1 = run(cmd1, timeout=90)
    cmd2 = ["tools/analyze_log.py", csv_path]
    rc2, out2, secs2 = run(cmd2, timeout=60)
    rows = _find_int(out1, r"csv rows written\s*:\s*(\d+)")
    samples = _find_int(out2, r"samples\s*:\s*(\d+)")
    ok = (rc1 == 0 and rc2 == 0 and rows > 0 and samples > 0
          and "TELEMETRY LOG ANALYSIS" in out2 and "STATE DWELL TIME" in out2)
    detail = ("csv rows=%s -> analysis samples=%s, metrics printed" % (rows, samples))
    return Result("5. monitor --csv then analyze_log", ok, detail,
                  "python %s && python %s" % (" ".join(cmd1), " ".join(cmd2)),
                  out1 + "\n" + ("-" * 60) + "\n" + out2, secs1 + secs2)


def check_6_analyze_selftest():
    # type: () -> Result
    cmd = ["tools/analyze_log.py", "--selftest"]
    rc, out, secs = run(cmd, timeout=60)
    ok = rc == 0 and "SELFTEST PASS" in out and "[FAIL]" not in out
    detail = "synthetic CSV + hex log analysed"
    match = re.search(r"SELFTEST PASS \((\d+) checks\)", out)
    if match:
        detail = "synthetic CSV + hex log analysed, %s checks passed" % match.group(1)
    return Result("6. analyze_log --selftest", ok, detail,
                  "python " + " ".join(cmd), out, secs)


def check_7_headless_import():
    # type: () -> Result
    """Requirement: ``import host.robot_monitor`` must work with Tk unusable."""
    blocker = tempfile.mkdtemp(prefix="tea_notk_")
    try:
        with open(os.path.join(blocker, "tkinter.py"), "w", encoding="ascii") as fh:
            fh.write("raise ImportError('simulated: Tk is not installed')\n")
        code = ("import host.robot_monitor as m;"
                "assert m.tk is None, 'tk should be None';"
                "assert m.TK_IMPORT_ERROR, 'expected a recorded import error';"
                "assert hasattr(m, 'HeadlessMonitor');"
                "print('headless import OK:', m.TK_IMPORT_ERROR)")
        env = {"PYTHONPATH": blocker + os.pathsep + PROJECT_ROOT}
        rc, out, secs = run(["-c", code], timeout=30, env=env)
        ok = rc == 0 and "headless import OK" in out
        detail = "imports cleanly with a broken tkinter (--nogui unaffected)"
        return Result("7. import with Tk unusable", ok, detail,
                      "PYTHONPATH=<fake tkinter> python -c \"import host.robot_monitor\"",
                      out, secs)
    finally:
        try:
            os.remove(os.path.join(blocker, "tkinter.py"))
            os.rmdir(blocker)
        except OSError:
            pass


CHECKS = [
    ("1", check_1_protocol),
    ("2", check_2_vision_stdout),
    ("3", check_3_vision_loss),
    ("4", check_4_monitor_demo),
    ("5", check_5_monitor_csv_to_analysis),
    ("6", check_6_analyze_selftest),
    ("7", check_7_headless_import),
]  # type: List[Tuple[str, Callable[[], Result]]]


def _find_int(text, pattern):
    # type: (str, str) -> int
    match = re.search(pattern, text)
    return int(match.group(1)) if match else -1


def main(argv=None):
    # type: (Optional[List[str]]) -> int
    parser = argparse.ArgumentParser(prog="selftest.py",
                                     description="Run the PC-side tool acceptance checks.")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="print each child process output")
    parser.add_argument("--only", metavar="N", help="run only check number N")
    args = parser.parse_args(argv)

    selected = [(key, fn) for key, fn in CHECKS
                if args.only is None or key == str(args.only)]
    if not selected:
        sys.stderr.write("no such check: %s\n" % args.only)
        return 2

    print("=" * 78)
    print("PC-SIDE TOOL SELF TEST   (project root: %s)" % PROJECT_ROOT)
    print("python %s" % sys.version.split()[0])
    print("=" * 78)

    results = []  # type: List[Result]
    started = time.time()
    for key, fn in selected:
        try:
            result = fn()
        except Exception as exc:  # noqa: BLE001 - a check must never abort the run
            result = Result("%s. <crashed>" % key, False,
                            "%s: %s" % (type(exc).__name__, exc))
        results.append(result)
        print("[%s] %-38s %6.2fs  %s"
              % ("PASS" if result.ok else "FAIL", result.name, result.seconds,
                 result.detail))
        if result.command:
            print("       cmd: %s" % result.command)
        if args.verbose and result.output:
            print("-" * 78)
            for line in result.output.splitlines():
                print("   | " + line)
            print("-" * 78)
        sys.stdout.flush()

    passed = sum(1 for r in results if r.ok)
    print("=" * 78)
    print("SUMMARY: %d/%d checks passed in %.1f s" % (passed, len(results),
                                                      time.time() - started))
    for r in results:
        if not r.ok:
            print("  FAILED: %s -- %s" % (r.name, r.detail))
            if not args.verbose:
                tail = r.output.strip().splitlines()[-12:]
                for line in tail:
                    print("      | " + line)
    print("RESULT: %s" % ("PASS" if passed == len(results) else "FAIL"))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
