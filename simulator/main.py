"""
Command-line entry point for the tea-picking robot simulator.

What
----
Runs one scenario or all seven, prints the scenario's console trace, writes the
time-series plots, the summary panel and the raw log, and exits with a status that
reflects whether the nominal pick succeeded.

Why
---
The simulator is only useful if a result can be produced and checked without
writing code: ``python simulator/main.py`` must be enough to get the five plots,
a CSV and a pass/fail verdict, and ``--scenario all`` must be enough to reproduce
the whole experiment table.  Every number printed here is read out of the
recorded telemetry, so the trace cannot disagree with the CSV or the figures.

Exit codes
----------
``0``  Scenario A completed the full picking cycle.
``1``  Scenario A did not complete (or a run raised).
``2``  Invalid command-line arguments.

Usage
-----
``python simulator/main.py [--scenario A|B|C|D|E|F|G|all] [--duration S]``
``        [--no-plot] [--outdir PATH] [--seed INT] [--verbose]``
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import traceback
from typing import Dict, List, Optional

# Allow both "python simulator/main.py" and "python -m simulator.main" by making
# the package importable when this file is run as a script.
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator import config as C
from simulator import scenarios
from simulator.plotting import default_outdir
from simulator.simulation import ScenarioResult
from simulator.telemetry import ensure_dir, project_root

#: Keys of the metrics printed by --verbose, in report order.
VERBOSE_METRIC_KEYS = (
    "final_position_mrad",
    "target_position_mrad",
    "final_position_error_mrad",
    "position_error_pct",
    "rise_time_s",
    "overshoot_pct",
    "settling_time_s",
    "steady_state_error_mrad",
    "peak_force_n",
    "force_rmse_n",
    "damage_index_ns",
    "cycle_time_s",
    "contact_time_s",
    "force_reached_time_s",
    "pwm_peak_permille",
    "current_peak_a",
    "halt_time_s",
    "pwm_after_halt_permille",
    "fault_count",
)


def build_parser() -> argparse.ArgumentParser:
    """Create the argument parser.

    Returns
    -------
    argparse.ArgumentParser
        The parser, with every documented flag registered.
    """
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="Tea-picking robot simulator (mirrors firmware config v{0}).".format(
            C.FW_VERSION
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scenario",
        default="A",
        choices=list(C.SCENARIO_IDS) + ["all"],
        help="Scenario to run; 'all' runs A-G and prints a summary table.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run length in seconds (default: {0}).".format(C.DEFAULT_DURATION_S),
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip figure generation (faster; useful for batch runs).",
    )
    parser.add_argument(
        "--outdir",
        default=None,
        help="Directory for figures (default: {0}).".format(C.RESULTS_DIR),
    )
    parser.add_argument(
        "--logdir",
        default=None,
        help="Directory for CSV/JSON logs (default: {0}).".format(C.LOGS_DIR),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=C.DEFAULT_SEED,
        help="RNG seed; identical seeds give identical runs.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print the full metric table, the FSM transition trace and warnings.",
    )
    parser.add_argument(
        "--profile",
        default="trapezoid",
        choices=["trapezoid", "scurve", "step"],
        help="Trajectory generator used by the joint controller.",
    )
    return parser


def _log_dir(logdir: Optional[str]) -> str:
    """Resolve and create the log directory.

    Parameters
    ----------
    logdir:
        Explicit directory, or ``None`` for ``<project root>/data/logs``.

    Returns
    -------
    str
        An existing directory.
    """
    target = logdir if logdir else os.path.join(project_root(), C.LOGS_DIR)
    return ensure_dir(target)


def run_single(
    args: argparse.Namespace,
    key: str,
    results_dir: str,
    logs_dir: str,
) -> ScenarioResult:
    """Run one scenario, print its trace, and write its artefacts.

    Parameters
    ----------
    args:
        Parsed command-line arguments.
    key:
        Scenario letter.
    results_dir:
        Where to write figures.
    logs_dir:
        Where to write the CSV and JSON logs.

    Returns
    -------
    ScenarioResult
        The completed (or failed) run.
    """
    duration = args.duration if args.duration else C.DEFAULT_DURATION_S
    started = time.time()
    try:
        res = scenarios.run_scenario(
            key, duration_s=duration, seed=args.seed, profile_kind=args.profile
        )
    except Exception:  # noqa: BLE001 - the CLI must report, not crash
        print("Scenario {0} raised an exception:".format(key))
        traceback.print_exc()
        return ScenarioResult(
            key=key,
            title=C.scenario_title(key),
            success=False,
            faults=["EXCEPTION"],
            success_reason="unhandled exception",
        )

    for line in res.trace_lines:
        print(line)
    print(
        "Result: {0}  (faults: {1})".format(
            "PASS" if res.success else "FAIL", ", ".join(res.faults) if res.faults else "none"
        )
    )
    if args.verbose:
        print("Reason: {0}".format(res.success_reason))
        print_verbose(res)
    print("Wall time: {0:.2f}s".format(time.time() - started))

    write_artefacts(res, results_dir, logs_dir, no_plot=args.no_plot)
    return res


def print_verbose(res: ScenarioResult) -> None:
    """Print the metric table and FSM trace (and any warnings) for one run.

    Parameters
    ----------
    res:
        A completed run.
    """
    print("  --- metrics ---")
    for key in VERBOSE_METRIC_KEYS:
        if key in res.metrics:
            print("    {0:<30} {1:>14.6f}".format(key, float(res.metrics[key])))
    warnings = [e["message"] for e in (res.recorder.events if res.recorder else [])
                if e["kind"] in ("warn", "fault", "estop")]
    if warnings:
        print("  --- warnings/faults ---")
        for message in warnings:
            print("    {0}".format(message))


def write_artefacts(
    res: ScenarioResult,
    results_dir: str,
    logs_dir: str,
    no_plot: bool = False,
) -> List[str]:
    """Write the CSV log, the JSON log and (optionally) the figures.

    Parameters
    ----------
    res:
        A completed run.
    results_dir:
        Figure directory.
    logs_dir:
        Log directory.
    no_plot:
        When ``True`` the figures are skipped but the logs are still written.

    Returns
    -------
    List[str]
        Paths of every file written.
    """
    written: List[str] = []
    if res.recorder is None or len(res.recorder) == 0:
        return written
    csv_path = os.path.join(logs_dir, "scenario_{0}_log.csv".format(res.key))
    json_path = os.path.join(logs_dir, "scenario_{0}_metrics.json".format(res.key))
    written.append(res.recorder.to_csv(csv_path))
    written.append(
        res.recorder.to_json(
            json_path,
            extra={
                "scenario": res.key,
                "title": res.title,
                "success": bool(res.success),
                "success_reason": res.success_reason,
                "faults": list(res.faults),
                "metrics": {k: float(v) for k, v in res.metrics.items()},
            },
        )
    )
    if not no_plot:
        try:
            written.extend(
                scenarios.write_scenario_figures(
                    res, outdir=results_dir, f_target_n=_target_for(res)
                )
            )
        except Exception:  # noqa: BLE001 - a plotting failure must not lose the logs
            print("Plotting failed for scenario {0}:".format(res.key), file=sys.stderr)
            traceback.print_exc()
    return written


def _target_for(res: ScenarioResult) -> Optional[float]:
    """Return the grip-force target a scenario was aiming for, if known.

    Parameters
    ----------
    res:
        A completed run.

    Returns
    -------
    float or None
        ``None`` for scenarios with no force target, so the plot omits the line
        instead of drawing a misleading one.
    """
    if res.key == "D":
        return C.SCEN_D_FORCE_TARGET_N
    if res.key in ("A", "B", "C", "E", "F", "G"):
        return C.FORCE_TARGET_N
    return None


def run_all(args: argparse.Namespace, results_dir: str, logs_dir: str) -> Dict[str, ScenarioResult]:
    """Run every scenario, print the summary table, and write the comparison figure.

    Parameters
    ----------
    args:
        Parsed command-line arguments.
    results_dir:
        Figure directory.
    logs_dir:
        Log directory.

    Returns
    -------
    Dict[str, ScenarioResult]
        Results keyed by scenario letter.
    """
    duration = args.duration if args.duration else C.DEFAULT_DURATION_S
    print("===== Tea Picking Robot Simulation: all scenarios =====")
    print("duration {0:.1f}s | seed {1} | profile {2}".format(duration, args.seed, args.profile))
    print()
    started = time.time()
    results = scenarios.run_all(
        duration_s=duration, seed=args.seed, profile_kind=args.profile
    )
    for key in C.SCENARIO_IDS:
        res = results[key]
        write_artefacts(res, results_dir, logs_dir, no_plot=args.no_plot)
    print(scenarios.summary_table(results))
    print()
    for key in C.SCENARIO_IDS:
        res = results[key]
        print(
            "  {0}: {1:<45} faults: {2}".format(
                key,
                res.success_reason,
                ", ".join(res.faults) if res.faults else "none",
            )
        )
    if not args.no_plot:
        try:
            path = scenarios.write_comparison_figure(results, outdir=results_dir)
            if path:
                print("\nComparison figure: {0}".format(path))
        except Exception:  # noqa: BLE001
            print("Comparison figure failed:", file=sys.stderr)
            traceback.print_exc()
    print("\nWall time: {0:.2f}s".format(time.time() - started))
    nominal = results.get("A")
    if nominal is not None:
        print("Simulation: {0}".format("PASS" if nominal.success else "FAIL"))
    return results


def main(argv: Optional[List[str]] = None) -> int:
    """Program entry point.

    Parameters
    ----------
    argv:
        Argument list; defaults to ``sys.argv[1:]``.

    Returns
    -------
    int
        Process exit code (see the module docstring).
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    problems = C.validate()
    if problems:
        print("Configuration problems detected:")
        for p in problems:
            print("  - {0}".format(p))
        return 2

    results_dir = default_outdir(args.outdir)
    logs_dir = _log_dir(args.logdir)

    if args.scenario == "all":
        results = run_all(args, results_dir, logs_dir)
        nominal = results.get("A")
        return 0 if (nominal is not None and nominal.success) else 1

    res = run_single(args, args.scenario, results_dir, logs_dir)
    print("Simulation: {0}".format("PASS" if res.success else "FAIL"))
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())
