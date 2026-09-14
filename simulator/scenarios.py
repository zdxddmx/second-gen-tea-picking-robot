"""
The seven named experimental scenarios A-G.

What
----
Each scenario is a small builder that returns a :class:`~simulator.simulation.ScenarioSpec`,
plus a runner that executes it and returns a :class:`~simulator.simulation.ScenarioResult`
with a full time series, a success verdict, the latched faults and the metrics.

======================  ==========================================================
Key  Scenario            What it establishes
======================  ==========================================================
A    Normal pick         The nominal cycle completes with no faults at all.
B    Target error        An +8 mm lateral and +5 mm depth vision error is either
                         rejected or the residual is bounded and quantified.
C    Force noise         A 0.08 N RMS noise floor is rejected to a force-tracking
                         RMSE below 0.25 N, and the filtered and unfiltered
                         channels are both recorded for comparison.
D    Excessive force     A 4.5 N target -- above the 4 N stem damage threshold --
                         raises the soft-limit warning, latches stem damage and
                         trips the overload protection.
E    Comm timeout        The host link is cut at 300 ms; ERR_COMM_TIMEOUT is
                         raised inside the 200 ms watchdog window and the joint
                         halts with PWM at zero.
F    Motor stall         A jammed joint with a 30 N*m resisting torque is detected
                         as ERR_MOTOR_STALL after the 300 ms debounce.
G    Emergency stop      An E-stop at 1.2 s reaches EMERGENCY_STOP immediately,
                         with PWM driven to zero and no further motion.
======================  ==========================================================

Why these seven
---------------
They are the seven ways this machine can fail or be mis-commanded: wrong target,
noisy sensor, over-force setpoint, dead link, jammed joint and a human hitting the
button.  Between them they exercise every safety path in the firmware's
``safety`` task and every fault code in ``status.h``.

What happens without it
-----------------------
"Works on the bench" would be the only available evidence, and the damage and
stall thresholds -- the two numbers that decide whether a real tea bud is
destroyed or a gearbox is stripped -- could not be validated at all.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from . import config as C
from .plotting import (
    default_outdir,
    plot_force_vs_time,
    plot_pid_output_vs_time,
    plot_position_vs_time,
    plot_scenario_comparison,
    plot_scenario_summary,
    plot_speed_vs_time,
    plot_state_vs_time,
)
from .simulation import ScenarioResult, ScenarioSpec, Simulation


def spec_for(key: str, duration_s: Optional[float] = None) -> ScenarioSpec:
    """Build the specification for a scenario.

    Parameters
    ----------
    key:
        Scenario letter, ``A`` through ``G`` (case-insensitive).
    duration_s:
        Optional run length override, seconds.

    Returns
    -------
    ScenarioSpec
        The scenario description.  Rates are unchanged by the override; only the
        stop time moves, so a longer run is strictly more of the same experiment.
    """
    k = str(key).strip().upper()
    d = float(duration_s) if duration_s else C.DEFAULT_DURATION_S
    builders = {
        "A": _spec_a,
        "B": _spec_b,
        "C": _spec_c,
        "D": _spec_d,
        "E": _spec_e,
        "F": _spec_f,
        "G": _spec_g,
    }
    if k not in builders:
        raise ValueError(
            "unknown scenario {0!r}; expected one of {1}".format(k, ", ".join(C.SCENARIO_IDS))
        )
    spec = builders[k](d)
    return spec


def _spec_a(duration_s: float) -> ScenarioSpec:
    """Scenario A: nominal pick with no injected defect."""
    return ScenarioSpec("A", C.scenario_title("A"), duration_s=duration_s)


def _spec_b(duration_s: float) -> ScenarioSpec:
    """Scenario B: inject a lateral and a depth error into the vision target.

    The reported errors are converted into a joint-space target offset: 8 mm of
    lateral error at the nominal 300 mm working radius is
    ``atan(8/300) ~ 1.53 deg ~ 26.7 mrad``, and the 5 mm depth error adds the
    matching radial correction.  Injecting the equivalent joint offset is the
    only way a single-joint plant can be said to receive the error at all.
    """
    lateral_mm = C.SCEN_B_LATERAL_ERROR_MM
    depth_mm = C.SCEN_B_DEPTH_ERROR_MM
    radius_mm = 300.0
    extra_mrad = (lateral_mm / radius_mm) * 1000.0 + (depth_mm / radius_mm) * 1000.0 * 0.25
    return ScenarioSpec(
        "B",
        C.scenario_title("B"),
        duration_s=duration_s,
        target_extra_mrad=extra_mrad,
        lateral_error_mm=lateral_mm,
        depth_error_mm=depth_mm,
    )


def _spec_c(duration_s: float) -> ScenarioSpec:
    """Scenario C: add Gaussian noise to the measured force."""
    return ScenarioSpec(
        "C",
        C.scenario_title("C"),
        duration_s=duration_s,
        noise_sigma_n=C.SCEN_C_FORCE_NOISE_SIGMA_N,
        sensor_filter=True,
    )


def _spec_c_unfiltered(duration_s: float) -> ScenarioSpec:
    """Scenario C comparison arm: the same noise with the LPF bypassed.

    Running both arms is what turns "filtering helps" from a claim into a
    measurement; the only difference between them is one boolean.
    """
    return ScenarioSpec(
        "C-raw",
        "Force-sensor noise (unfiltered)",
        duration_s=duration_s,
        noise_sigma_n=C.SCEN_C_FORCE_NOISE_SIGMA_N,
        sensor_filter=False,
    )


def _spec_d(duration_s: float) -> ScenarioSpec:
    """Scenario D: command 4.5 N, which is above the 4 N damage threshold."""
    return ScenarioSpec(
        "D",
        C.scenario_title("D"),
        duration_s=duration_s,
        f_target_override_n=C.SCEN_D_FORCE_TARGET_N,
    )


def _spec_e(duration_s: float) -> ScenarioSpec:
    """Scenario E: drop the host link at 300 ms."""
    return ScenarioSpec(
        "E",
        C.scenario_title("E"),
        duration_s=duration_s,
        link_drop_s=C.SCEN_E_LINK_DROP_S,
    )


def _spec_f(duration_s: float) -> ScenarioSpec:
    """Scenario F: jam the joint at 400 ms and load it with 30 N*m."""
    return ScenarioSpec(
        "F",
        C.scenario_title("F"),
        duration_s=duration_s,
        clamp_joint_at_s=C.SCEN_F_CLAMP_AT_S,
        resist_torque_nm=C.SCEN_F_RESIST_TORQUE_NM,
    )


def _spec_g(duration_s: float) -> ScenarioSpec:
    """Scenario G: assert the E-stop at 1.2 s."""
    return ScenarioSpec(
        "G",
        C.scenario_title("G"),
        duration_s=duration_s,
        estop_at_s=C.SCEN_G_ESTOP_AT_S,
    )


def run_scenario(
    key: str,
    duration_s: Optional[float] = None,
    seed: int = C.DEFAULT_SEED,
    profile_kind: str = "trapezoid",
    force_max_hold_s: float = 0.0,
) -> ScenarioResult:
    """Run one scenario to completion.

    Parameters
    ----------
    key:
        Scenario letter.
    duration_s:
        Optional run-length override.
    seed:
        RNG seed; identical seeds give identical runs.
    profile_kind:
        Trajectory generator (``trapezoid``, ``scurve`` or ``step``).
    force_max_hold_s:
        Frozen tail kept after a fault, seconds.

    Returns
    -------
    ScenarioResult
        The completed run.
    """
    spec = spec_for(key, duration_s)
    sim = Simulation(
        spec,
        seed=seed,
        profile_kind=profile_kind,
        force_max_hold_s=force_max_hold_s,
    )
    return sim.run()


def run_scenario_c_comparison(
    duration_s: Optional[float] = None,
    seed: int = C.DEFAULT_SEED,
) -> Tuple[ScenarioResult, ScenarioResult]:
    """Run both arms of Scenario C: filtered and unfiltered.

    Returns
    -------
    tuple
        ``(filtered, unfiltered)`` results, produced with the same seed so that
        the noise realisation itself is identical and only the filtering differs.
    """
    filtered = Simulation(_spec_c(duration_s or C.DEFAULT_DURATION_S), seed=seed).run()
    unfiltered = Simulation(_spec_c_unfiltered(duration_s or C.DEFAULT_DURATION_S), seed=seed).run()
    filtered.metrics["rmse_unfiltered_n"] = unfiltered.metrics.get("force_rmse_n", 0.0)
    filtered.metrics["rmse_filtered_n"] = filtered.metrics.get("force_rmse_n", 0.0)
    filtered.metrics["noise_sigma_n"] = C.SCEN_C_FORCE_NOISE_SIGMA_N
    return filtered, unfiltered


def run_all(
    duration_s: Optional[float] = None,
    seed: int = C.DEFAULT_SEED,
    profile_kind: str = "trapezoid",
) -> Dict[str, ScenarioResult]:
    """Run every scenario A-G with the same duration and seed.

    Parameters
    ----------
    duration_s:
        Optional run-length override applied to all scenarios.
    seed:
        RNG seed.
    profile_kind:
        Trajectory generator.

    Returns
    -------
    Dict[str, ScenarioResult]
        Keyed by scenario letter, in A-G order.  A scenario that raises is
        recorded as a failure rather than aborting the batch, so one broken
        scenario cannot hide the result of the other six.
    """
    results: Dict[str, ScenarioResult] = {}
    for key in C.SCENARIO_IDS:
        try:
            results[key] = run_scenario(key, duration_s, seed, profile_kind)
        except Exception as exc:  # noqa: BLE001 - a batch must survive one failure
            results[key] = ScenarioResult(
                key=key,
                title=C.scenario_title(key),
                success=False,
                faults=["EXCEPTION"],
                metrics={},
                recorder=None,
                trace_lines=[],
                success_reason="raised {0}: {1}".format(type(exc).__name__, exc),
            )
    return results


def write_scenario_figures(
    result: ScenarioResult,
    outdir: Optional[str] = None,
    f_target_n: Optional[float] = None,
) -> List[str]:
    """Write the five time-series plots and the summary panel for one result.

    Parameters
    ----------
    result:
        A completed scenario.
    outdir:
        Output directory; defaults to ``data/results``.
    f_target_n:
        Force target to mark on the force plot, when known.

    Returns
    -------
    List[str]
        Paths of the files written, in writing order.
    """
    rec = result.recorder
    if rec is None or len(rec) == 0:
        return []
    dest = default_outdir(outdir)
    title = "Scenario {0}: {1}".format(result.key, result.title)
    fault_times = [
        float(rec.array("t")[i])
        for i, s in enumerate(rec.data["state"])
        if i > 0 and s in (10.0, 11.0) and rec.data["state"][i - 1] not in (10.0, 11.0)
    ]
    paths = [
        plot_position_vs_time(rec, outdir=dest, title=title),
        plot_speed_vs_time(rec, outdir=dest, title=title),
        plot_force_vs_time(rec, outdir=dest, title=title, f_target_n=f_target_n),
        plot_pid_output_vs_time(rec, outdir=dest, title=title),
        plot_state_vs_time(rec, outdir=dest, title=title, fault_times=fault_times),
        plot_scenario_summary(
            rec,
            result.key,
            faults=result.faults,
            metrics=result.metrics,
            f_target_n=f_target_n,
            outdir=dest,
        ),
    ]
    return paths


def write_comparison_figure(
    results: Dict[str, ScenarioResult],
    outdir: Optional[str] = None,
) -> str:
    """Write the cross-scenario comparison figure for ``--scenario all``.

    Parameters
    ----------
    results:
        Mapping produced by :func:`run_all`.
    outdir:
        Output directory; defaults to ``data/results``.

    Returns
    -------
    str
        Path of the PNG written, or an empty string when there is nothing to plot.
    """
    payload: Dict[str, Dict[str, float]] = {
        k: v.metrics for k, v in results.items() if v.metrics
    }
    if not payload:
        return ""
    return plot_scenario_comparison(payload, outdir=default_outdir(outdir))


def summary_table(results: Dict[str, ScenarioResult]) -> str:
    """Format the one-line-per-scenario summary table for ``--scenario all``.

    Parameters
    ----------
    results:
        Mapping produced by :func:`run_all`.

    Returns
    -------
    str
        A fixed-width table.  Every field is read from the run's own metrics, so
        the table cannot disagree with the recorded data.
    """
    header = (
        "Scenario  Result  PeakF[N]  RMSE[N]  Rise[s]  Settle[s]  Err[mrad]  "
        "Cycle[s]  Faults"
    )
    lines = [header, "-" * len(header)]
    for key in C.SCENARIO_IDS:
        r = results.get(key)
        if r is None:
            continue
        m = r.metrics
        settle = m.get("settling_time_s", 0.0)
        settle_txt = "n/a" if (settle != settle) else "{0:.3f}".format(settle)
        faults = ",".join(r.faults) if r.faults else "-"
        if len(faults) > 34:
            faults = faults[:31] + "..."
        lines.append(
            "{0:<9} {1:<7} {2:>8.3f}  {3:>7.4f}  {4:>7.3f}  {5:>9}  "
            "{6:>9.2f}  {7:>8.3f}  {8}".format(
                key,
                "PASS" if r.success else "FAIL",
                float(m.get("peak_force_n", 0.0)),
                float(m.get("force_rmse_n", 0.0)),
                float(m.get("rise_time_s", 0.0)),
                settle_txt,
                float(m.get("final_position_error_mrad", 0.0)),
                float(m.get("cycle_time_s", 0.0)),
                faults,
            )
        )
    return "\n".join(lines)
