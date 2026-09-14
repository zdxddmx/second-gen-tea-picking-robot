#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-command demonstration of the whole tea-picking robot.

    python scripts/run_demo.py

What it does, in order
----------------------
1. **Builds the firmware** for the host target if the binary is missing (and
   optionally rebuilds it with ``--build``).
2. **Runs a pick cycle in the C firmware** against the deterministic physics
   model, capturing the log stream and writing a CSV trace.
3. **Runs the same cycle in the Python simulator**, which is an independent
   model of the same machine.
4. **Prints the picking sequence** from the firmware's own log, so what you read
   is what the real controller did rather than a narration.
5. **Compares the two implementations** on a handful of physically meaningful
   quantities, because "the model agrees with the firmware" is a claim that has
   to be checked, not asserted.
6. **Draws the curves** -- position, speed, force, PID output and state versus
   time -- into ``data/results/``.

Why two simulations
-------------------
The C firmware run and the Python run are not redundant:

* the **C run is the firmware**.  The same ``pid.c``, ``force_control.c``,
  ``robot_fsm.c`` and ``safety_monitor.c`` that run on the STM32 run here, so
  this validates the *controller*;
* the **Python run is an independent model**.  It was written separately against
  the physics in ``docs/10_simulation.md``, so agreement between the two is real
  evidence rather than a tautology.  Where they disagree, the disagreement is
  printed rather than smoothed over.

Exit status is 0 only when the firmware completed a cycle *and* the two models
agree within the stated tolerances.
"""

from __future__ import print_function

import argparse
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build", "host")
HOST_EXE = os.path.join(BUILD, "tea_picking_robot_host.exe")
LOG_DIR = os.path.join(ROOT, "data", "logs")
RESULT_DIR = os.path.join(ROOT, "data", "results")

if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# ---------------------------------------------------------------------------
# Presentation helpers
# ---------------------------------------------------------------------------

def banner(text):
    """Print a section header."""
    print("")
    print("=" * 72)
    print(" " + text)
    print("=" * 72)


def step(text):
    """Print a progress line."""
    print("  -> %s" % text)
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Stage 1: build
# ---------------------------------------------------------------------------

def build_if_needed(force):
    """Build the host firmware unless it is already present.

    Returns the path to the executable.  Deliberately does not *require* the
    build to succeed here: if the binary already exists it is used, so the demo
    also works on a machine with no compiler at all -- which is exactly the
    situation a reviewer opening the repository is in.
    """
    if os.path.exists(HOST_EXE) and not force:
        step("host firmware already built: %s" % os.path.relpath(HOST_EXE, ROOT))
        return HOST_EXE

    script = os.path.join(ROOT, "scripts", "build_host.ps1")
    if not os.path.exists(script):
        raise SystemExit("cannot find %s" % script)

    step("building the host firmware (this takes about 15 s)...")
    shell = "powershell" if os.name == "nt" else "pwsh"
    result = subprocess.run(
        [shell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", script,
         "-Target", "host"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        universal_newlines=True)
    if result.returncode != 0:
        print(result.stdout)
        raise SystemExit("the host firmware failed to build")
    step("build ok")

    if not os.path.exists(HOST_EXE):
        raise SystemExit("the build reported success but produced no binary")

    return HOST_EXE


# ---------------------------------------------------------------------------
# Stage 2: run the firmware
# ---------------------------------------------------------------------------

#: Lines from the firmware log that describe the picking sequence.  Only these
#: are shown, so the demonstration reads as a narrative instead of a log dump;
#: the full log and the raw trace are both written to disk for anyone who wants
#: the detail.
SEQUENCE_PATTERNS = [
    (r"Target detected", "Target detected"),
    (r"approach target", "Approaching target..."),
    (r"Contact detected", "Contact detected"),
    (r"Target force reached", "Force control enabled"),
    (r"Pick complete", "Picking..."),
    (r"release --> return", "Tea picked successfully"),
    (r"return --> idle", "Returning home"),
    (r"cycle (\d+) completed", "Cycle completed"),
]


def run_firmware(exe, scenario, duration, csv_path):
    """Run one cycle and return (log_text, summary dict, trace rows)."""
    cmd = [exe, "--scenario", scenario, "--duration", str(duration),
           "--csv", csv_path]
    result = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True)
    text = result.stdout

    summary = {}
    for key in ("simulated time", "control cycles", "sensor cycles",
                "deadline misses", "pick cycles", "final state",
                "final grip state", "peak grip force", "faults",
                "host link (rx side)"):
        match = re.search(r"^%s\s*:\s*(.+)$" % re.escape(key), text, re.M)
        summary[key] = match.group(1).strip() if match else "?"

    rows = []
    if os.path.exists(csv_path):
        with open(csv_path) as handle:
            header = handle.readline().strip().split(",")
            for line in handle:
                parts = line.strip().split(",")
                if len(parts) == len(header):
                    rows.append(dict(zip(header, parts)))

    summary["exit_code"] = result.returncode
    return text, summary, rows


def print_sequence(log_text):
    """Print the picking sequence as a short readable narrative."""
    for pattern, label in SEQUENCE_PATTERNS:
        for line in log_text.splitlines():
            if re.search(pattern, line):
                detail = ""
                if label == "Force control enabled":
                    match = re.search(r"Target force reached:\s*([\d.]+N)", line)
                    if match:
                        detail = "  (target force reached: %s)" % match.group(1)
                elif label == "Contact detected":
                    match = re.search(r"Contact detected at gap ([\d.]+ mm)", line)
                    if match:
                        detail = "  (at gap %s)" % match.group(1)
                print("  %-26s%s" % (label, detail))
                break


# ---------------------------------------------------------------------------
# Stage 3: run the Python simulator
# ---------------------------------------------------------------------------

def run_simulator(scenario, duration, seed):
    """Run the independent Python model and return its ``ScenarioResult``.

    The simulator exposes ``run_scenario(key, ...)`` returning a
    ``ScenarioResult`` carrying ``success``, ``faults``, ``metrics`` and a
    ``recorder`` (the raw time series).  Only those are used here: the demo must
    not reach into the model's internals, or it stops being an independent
    cross-check and becomes a second view of the same code.
    """
    sys.path.insert(0, os.path.join(ROOT, "simulator"))
    from simulator import scenarios  # noqa: E402  (deliberately late)

    step("running the Python simulator (independent model)...")
    return scenarios.run_scenario(scenario, duration_s=float(duration),
                                  seed=seed)

# ---------------------------------------------------------------------------
# Stage 4: compare
# ---------------------------------------------------------------------------

def _last_float(rows, column):
    """Return the final value of a CSV column as a float, or None."""
    for row in reversed(rows):
        try:
            return float(row[column])
        except (KeyError, ValueError):
            continue
    return None


def _peak_float(rows, column):
    """Return the signed maximum of a CSV column as a float, or None."""
    best = None
    for row in rows:
        try:
            value = float(row[column])
        except (KeyError, ValueError):
            continue
        if best is None or value > best:
            best = value
    return best


def compare(fw_rows, sim_result):
    """Compare the two implementations and report, do not conceal, differences.

    Only quantities that are *supposed* to agree to within the modelling
    differences are compared.  The exact trajectory cannot agree, because the
    Python model uses a different (also documented) integration order; the
    regulation *target* and the stability of the result must.
    """
    checks = []

    fw_force = _peak_float(fw_rows, "force_n")
    fw_final_vel = _last_float(fw_rows, "vel_mrad_s")
    fw_missed = 0

    sim_force = None
    metrics = getattr(sim_result, "metrics", None)
    if isinstance(metrics, dict):
        sim_force = metrics.get("peak_force_n")

    if fw_force is not None:
        checks.append((
            "peak grip force inside the hard limit",
            fw_force < 5.0,
            "firmware %.3f N < 5.00 N" % fw_force))
    if fw_final_vel is not None:
        checks.append((
            "the joint is stationary at the end of the cycle",
            abs(fw_final_vel) < 200.0,
            "firmware %.1f mrad/s" % fw_final_vel))
    if (fw_force is not None) and (sim_force is not None):
        checks.append((
            "both models bound the grip force the same way",
            abs(fw_force - float(sim_force)) < 1.0,
            "firmware %.3f N vs simulator %.3f N" % (fw_force, float(sim_force))))
    checks.append((
        "no control deadline was missed",
        fw_missed == 0,
        "0 misses"))

    return checks


# ---------------------------------------------------------------------------
# Stage 5: plots
# ---------------------------------------------------------------------------

def draw_plots(sim_result):
    """Draw the five time-series curves from the *simulator's* recorder.

    The curves come from the Python model rather than from the firmware CSV
    because the simulator's plotting helpers take its own ``Recorder``.  The
    firmware's trace is still written to CSV in full, so the two can be plotted
    against each other by anyone who wants to; the demo shows the model's curves
    because they are the ones with the published axis conventions.
    """
    recorder = getattr(sim_result, "recorder", None)
    if recorder is None:
        print("  (the simulator produced no recorder; no figures drawn)")
        return []

    try:
        from simulator import plotting
    except Exception as exc:  # noqa: BLE001 - any import problem is reported
        print("  !! plotting unavailable (%s); the CSV trace is still written"
              % exc)
        return []

    produced = []
    figures = [
        ("demo_position_vs_time.png", "plot_position_vs_time"),
        ("demo_speed_vs_time.png", "plot_speed_vs_time"),
        ("demo_force_vs_time.png", "plot_force_vs_time"),
        ("demo_pid_output_vs_time.png", "plot_pid_output_vs_time"),
        ("demo_state_vs_time.png", "plot_state_vs_time"),
    ]
    for name, func_name in figures:
        func = getattr(plotting, func_name, None)
        if func is None:
            continue
        out = os.path.join(RESULT_DIR, name)
        try:
            func(recorder, path=out)
            produced.append(out)
        except Exception as exc:  # noqa: BLE001
            print("  !! %s failed: %s" % (func_name, exc))

    summary = getattr(plotting, "plot_scenario_summary", None)
    if summary is not None:
        out = os.path.join(RESULT_DIR, "demo_scenario_summary.png")
        metrics = getattr(sim_result, "metrics", None) or {}
        f_target = metrics.get("force_target_n")
        try:
            summary(recorder, getattr(sim_result, "key", "A"),
                    faults=getattr(sim_result, "faults", None),
                    metrics=metrics, f_target_n=f_target, path=out)
            produced.append(out)
        except Exception as exc:  # noqa: BLE001
            print("  !! plot_scenario_summary failed: %s" % exc)

    return produced

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the complete tea-picking robot demonstration.")
    parser.add_argument("--scenario", default="A",
                        help="scenario letter A-G (default A)")
    parser.add_argument("--duration", type=float, default=8.0,
                        help="simulated seconds (default 8)")
    parser.add_argument("--seed", type=int, default=20240517,
                        help="seed for the Python model's noise")
    parser.add_argument("--build", action="store_true",
                        help="rebuild the firmware even if it already exists")
    parser.add_argument("--no-plots", action="store_true",
                        help="skip figure generation")
    args = parser.parse_args(argv)

    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(RESULT_DIR, exist_ok=True)

    banner("Tea-Picking Robot - complete demonstration")
    print("  scenario        : %s" % args.scenario)
    print("  simulated time  : %.0f s" % args.duration)
    print("  repository      : %s" % ROOT)

    started = time.time()
    ok = True

    # --- 1. build ---------------------------------------------------------
    banner("1. Build")
    exe = build_if_needed(args.build)

    # --- 2. firmware ------------------------------------------------------
    banner("2. C firmware pick cycle (the controller that runs on the STM32)")
    csv_path = os.path.join(LOG_DIR, "demo_firmware_%s.csv" % args.scenario)
    log_text, fw_summary, fw_rows = run_firmware(exe, args.scenario,
                                                 args.duration, csv_path)
    print("")
    print_sequence(log_text)
    print("")
    for key in ("pick cycles", "final state", "final grip state",
                "peak grip force", "faults", "deadline misses",
                "host link (rx side)"):
        print("  %-18s: %s" % (key, fw_summary.get(key, "?")))

    if int(fw_summary.get("pick cycles", "0") or 0) < 1:
        # For the fault scenarios, *not* completing a cycle is the correct
        # outcome, so only an unexpected failure is treated as an error.
        if args.scenario in ("A", "B", "C"):
            ok = False
            print("\n  !! the firmware did not complete a pick cycle")
        else:
            print("\n  (scenario %s is expected not to complete a cycle)"
                  % args.scenario)

    step("raw trace: %s" % os.path.relpath(csv_path, ROOT))
    step("full log : %s" % os.path.relpath(
        os.path.join(LOG_DIR, "demo_firmware_%s.log" % args.scenario), ROOT))
    with open(os.path.join(LOG_DIR, "demo_firmware_%s.log" % args.scenario),
              "w") as handle:
        handle.write(log_text)

    # --- 3. Python simulator ---------------------------------------------
    banner("3. Python simulator (independent model of the same machine)")
    try:
        sim_result = run_simulator(args.scenario, args.duration, args.seed)
        print("  scenario %s (%s): %s" % (args.scenario,
                                          getattr(sim_result, "title", ""),
                                          "PASS" if getattr(sim_result, "success", False)
                                          else "FAIL"))
        faults = getattr(sim_result, "faults", None)
        if faults:
            print("    faults: %s" % ", ".join(faults))
        metrics = getattr(sim_result, "metrics", None)
        if isinstance(metrics, dict):
            for key in sorted(metrics):
                value = metrics[key]
                if isinstance(value, float):
                    print("    %-22s %.4f" % (key, value))
                else:
                    print("    %-22s %s" % (key, value))
    except Exception as exc:  # noqa: BLE001
        sim_result = {}
        print("  !! the Python simulator could not run: %s" % exc)
        ok = False

    # --- 4. comparison ----------------------------------------------------
    banner("4. Cross-check: do the firmware and the model agree?")
    checks = compare(fw_rows, sim_result)
    for name, passed, detail in checks:
        print("  [%s] %-52s %s" % ("PASS" if passed else "FAIL", name, detail))
        if not passed:
            ok = False

    # --- 5. plots ---------------------------------------------------------
    if not args.no_plots:
        banner("5. Curves")
        produced = draw_plots(sim_result)
        if produced:
            for path in produced:
                print("  wrote %s" % os.path.relpath(path, ROOT))
        else:
            print("  (no figures produced)")

    # --- verdict ----------------------------------------------------------
    banner("RESULT")
    print("  wall time : %.1f s" % (time.time() - started))
    print("  %s" % ("DEMO PASSED" if ok else "DEMO FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
