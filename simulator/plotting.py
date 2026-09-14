"""
Figure generation for the tea-picking simulator.

What
----
Publication-quality PNG figures (150 dpi, grid, units on every axis, legend where
more than one series is drawn) written into ``data/results/``:

``plot_position_vs_time``, ``plot_speed_vs_time``, ``plot_force_vs_time``,
``plot_pid_output_vs_time``, ``plot_state_vs_time`` and
``plot_experiment_01_step_response``, plus :func:`plot_scenario_summary` (a 2x2
panel combining position, force, state and PWM for one scenario) and
:func:`plot_scenario_comparison` for the cross-scenario figure.

Why Agg first
-------------
``matplotlib.use("Agg")`` is selected *before* ``pyplot`` is imported.  Without
it, importing pyplot on a machine with no display (CI, a headless SSH session, a
Windows service) can fail or silently fall back to a blocking interactive
backend, and a batch run of seven scenarios would hang on the first figure.

Why a step plot for state
-------------------------
The FSM state is a categorical, zero-order-held signal.  Drawing it as a line
would interpolate between ``IDLE`` and ``ERROR`` and imply states that never
happened; ``drawstyle="steps-post"`` plus integer y-ticks labelled with the state
names is the only honest rendering.

What happens without it
-----------------------
Results would be numbers in a terminal, and the interaction between the force
loop and the state machine -- which is the entire subject of the experiments --
would not be visible at all.
"""

from __future__ import annotations

import os
from typing import Dict, Optional, Sequence

import matplotlib

matplotlib.use("Agg")  # must precede the pyplot import; see module docstring

import matplotlib.pyplot as plt
import numpy as np

from . import config as C
from .fsm import STATE_ORDER
from .metrics import metrics_from_recorder
from .telemetry import Recorder, ensure_dir, project_root

#: Figure width in inches; a two-column journal width at 150 dpi.
FIGSIZE = (7.2, 4.3)
#: Panel figure size for the scenario summary.
SUMMARY_FIGSIZE = (12.0, 7.5)


class NoDataError(RuntimeError):
    """Raised when a plot is requested for a run that recorded nothing.

    Plotting an empty recorder would otherwise produce a blank PNG that looks
    like a successful figure, so this is treated as an error rather than a
    warning.
    """


def _require_data(rec: Recorder, what: str = "channels") -> np.ndarray:
    """Return the time channel of ``rec``, raising when it is empty.

    Parameters
    ----------
    rec:
        Recorder to inspect.
    what:
        Label used in the error message.

    Returns
    -------
    numpy.ndarray
        The ``t`` channel.
    """
    t = rec.array("t")
    if t.size == 0:
        raise NoDataError("no {0} recorded; run a scenario before plotting".format(what))
    return t


def _finish(fig: "plt.Figure", path: str) -> str:
    """Save ``fig`` to ``path``, close it, and return the path.

    Parameters
    ----------
    fig:
        Figure to write.
    path:
        Destination; parent directories are created automatically.
    """
    ensure_dir(os.path.dirname(os.path.abspath(path)))
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def default_outdir(outdir: Optional[str] = None) -> str:
    """Resolve the figure output directory.

    Parameters
    ----------
    outdir:
        Explicit directory, or ``None`` for ``<project root>/data/results``.

    Returns
    -------
    str
        An existing directory path.
    """
    target = outdir if outdir else os.path.join(project_root(), C.RESULTS_DIR)
    return ensure_dir(target)


def _style(ax: "plt.Axes", title: str, xlabel: str, ylabel: str) -> None:
    """Apply the shared axis styling.

    Parameters
    ----------
    ax:
        Axes to style.
    title, xlabel, ylabel:
        Text; the label strings are expected to carry their units.
    """
    ax.set_title(title, fontsize=11)
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.grid(True, which="both", alpha=0.3, linewidth=0.6)
    ax.tick_params(labelsize=9)


def plot_position_vs_time(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Joint position tracking",
) -> str:
    """Plot the position reference and the measured joint angle.

    Parameters
    ----------
    rec:
        Populated recorder.
    path:
        Explicit output path; overrides ``outdir``.
    outdir:
        Output directory when ``path`` is not given.
    title:
        Figure title, normally the scenario name.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "position")
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.plot(t, rec.array("ref"), "k--", linewidth=1.4, label="Reference")
    ax.plot(t, rec.array("pos"), color="#1f77b4", linewidth=1.6, label="Measured")
    _style(ax, title, "Time [s]", "Joint position [mrad]")
    ax.legend(loc="best", fontsize=9)
    dest = path or os.path.join(default_outdir(outdir), "position_vs_time.png")
    return _finish(fig, dest)


def plot_speed_vs_time(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Joint speed",
) -> str:
    """Plot the measured joint velocity with the speed limit marked.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "speed")
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.plot(t, rec.array("vel"), color="#d62728", linewidth=1.5, label="Measured speed")
    ax.axhline(C.JOINT_MAX_SPEED_MRAD_S, color="grey", linestyle=":", linewidth=1.0,
               label="Speed limit +{0:.0f}".format(C.JOINT_MAX_SPEED_MRAD_S))
    ax.axhline(-C.JOINT_MAX_SPEED_MRAD_S, color="grey", linestyle=":", linewidth=1.0,
               label="Speed limit -{0:.0f}".format(C.JOINT_MAX_SPEED_MRAD_S))
    _style(ax, title, "Time [s]", "Joint speed [mrad/s]")
    ax.legend(loc="best", fontsize=8)
    dest = path or os.path.join(default_outdir(outdir), "speed_vs_time.png")
    return _finish(fig, dest)


def plot_force_vs_time(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Grip force",
    f_target_n: Optional[float] = None,
    show_limits: bool = True,
) -> str:
    """Plot the grip force with the target and the safety thresholds.

    Parameters
    ----------
    rec:
        Populated recorder.
    path, outdir, title:
        As for :func:`plot_position_vs_time`.
    f_target_n:
        Optional target force drawn as a dashed line.
    show_limits:
        Draw the contact, soft-limit and damage thresholds; these are what make
        Scenario D legible at a glance.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "force")
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.plot(t, rec.array("force"), color="#2ca02c", linewidth=1.5, label="Grip force")
    if f_target_n is not None:
        ax.axhline(f_target_n, color="black", linestyle="--", linewidth=1.2,
                   label="Target {0:.2f} N".format(f_target_n))
    if show_limits:
        ax.axhline(C.FORCE_CONTACT_N, color="#1f77b4", linestyle=":", linewidth=1.0,
                   label="Contact {0:.2f} N".format(C.FORCE_CONTACT_N))
        ax.axhline(C.FORCE_SOFT_LIMIT_N, color="#ff7f0e", linestyle="-.", linewidth=1.0,
                   label="Soft limit {0:.2f} N".format(C.FORCE_SOFT_LIMIT_N))
        ax.axhline(C.DAMAGE_FORCE_N, color="#d62728", linestyle="--", linewidth=1.0,
                   label="Damage {0:.2f} N".format(C.DAMAGE_FORCE_N))
    _style(ax, title, "Time [s]", "Grip force [N]")
    ax.legend(loc="best", fontsize=8, ncol=2)
    dest = path or os.path.join(default_outdir(outdir), "force_vs_time.png")
    return _finish(fig, dest)


def plot_pid_output_vs_time(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Controller output",
) -> str:
    """Plot the PWM command and the motor current, each on its own axis.

    The two are drawn on twin axes because their units are unrelated (per-mille
    versus amperes) and the interesting question -- did the command saturate, and
    did the current hit its limit while it did -- is answered by seeing both at
    the same instant.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "pid output")
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.plot(t, rec.array("pwm"), color="#9467bd", linewidth=1.5, label="PWM command")
    ax.axhline(C.MAX_PWM_PERMILLE, color="grey", linestyle=":", linewidth=1.0, label="PWM limit")
    ax.axhline(-C.MAX_PWM_PERMILLE, color="grey", linestyle=":", linewidth=1.0)
    _style(ax, title, "Time [s]", "PWM duty [per-mille]")
    ax2 = ax.twinx()
    ax2.plot(t, rec.array("current"), color="#8c564b", linewidth=1.2, label="Motor current")
    ax2.set_ylabel("Motor current [A]", fontsize=10)
    ax2.tick_params(labelsize=9)
    lines = ax.get_lines()[:1] + ax2.get_lines()[:1]
    ax.legend(lines, [ln.get_label() for ln in lines], loc="best", fontsize=9)
    dest = path or os.path.join(default_outdir(outdir), "pid_output_vs_time.png")
    return _finish(fig, dest)


def plot_state_vs_time(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Robot state",
    fault_times: Optional[Sequence[float]] = None,
) -> str:
    """Plot the FSM state as a step function with named y ticks.

    Parameters
    ----------
    rec:
        Populated recorder whose ``state`` channel holds state indices.
    path, outdir, title:
        As for :func:`plot_position_vs_time`.
    fault_times:
        Optional times at which a fault was latched; drawn as vertical lines so
        the reader can see the exact transition that the fault caused.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "state")
    states = rec.array("state")
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.plot(t, states, color="#17becf", linewidth=1.6, drawstyle="steps-post", label="FSM state")
    ax.set_yticks(list(range(len(STATE_ORDER))))
    ax.set_yticklabels([s.value for s in STATE_ORDER], fontsize=8)
    ax.set_ylim(-0.5, len(STATE_ORDER) - 0.5)
    if fault_times:
        for i, tf in enumerate(fault_times):
            ax.axvline(tf, color="#d62728", linestyle="--", linewidth=1.2,
                       label="Fault" if i == 0 else None)
        ax.legend(loc="best", fontsize=8)
    _style(ax, title, "Time [s]", "State")
    dest = path or os.path.join(default_outdir(outdir), "state_vs_time.png")
    return _finish(fig, dest)


def plot_scenario_summary(
    rec: Recorder,
    scenario_key: str,
    faults: Optional[Sequence[str]] = None,
    metrics: Optional[Dict[str, float]] = None,
    f_target_n: Optional[float] = None,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
) -> str:
    """Draw a 2x2 summary panel for one scenario.

    Panels: position tracking, grip force against its limits, FSM state as a step
    plot, and PWM with current on a twin axis.  The fault list is printed as a
    subtitle, so the figure is self-describing when it is opened on its own.

    Parameters
    ----------
    rec:
        Populated recorder.
    scenario_key:
        Scenario letter, used for the title.
    faults:
        Fault codes to show in the subtitle.
    metrics:
        Optional metrics dictionary; a short selection is printed in the corner.
    f_target_n:
        Optional force target for the force panel.
    path, outdir:
        Output location.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "scenario summary")
    fig, axes = plt.subplots(2, 2, figsize=SUMMARY_FIGSIZE)
    fault_txt = ", ".join(faults) if faults else "none"
    fig.suptitle(
        "Scenario {0}: {1}   |   faults: {2}".format(
            scenario_key, C.scenario_title(scenario_key), fault_txt
        ),
        fontsize=12,
    )

    ax = axes[0][0]
    ax.plot(t, rec.array("ref"), "k--", linewidth=1.3, label="Reference")
    ax.plot(t, rec.array("pos"), color="#1f77b4", linewidth=1.5, label="Measured")
    _style(ax, "Joint position", "Time [s]", "Position [mrad]")
    ax.legend(fontsize=8)

    ax = axes[0][1]
    ax.plot(t, rec.array("force"), color="#2ca02c", linewidth=1.4, label="Grip force")
    if f_target_n is not None:
        ax.axhline(f_target_n, color="black", linestyle="--", linewidth=1.1,
                   label="Target {0:.2f} N".format(f_target_n))
    ax.axhline(C.DAMAGE_FORCE_N, color="#d62728", linestyle="--", linewidth=1.0,
               label="Damage {0:.1f} N".format(C.DAMAGE_FORCE_N))
    _style(ax, "Grip force", "Time [s]", "Force [N]")
    ax.legend(fontsize=8)

    ax = axes[1][0]
    ax.plot(t, rec.array("state"), color="#17becf", linewidth=1.5, drawstyle="steps-post")
    ax.set_yticks(list(range(len(STATE_ORDER))))
    ax.set_yticklabels([s.value for s in STATE_ORDER], fontsize=7)
    ax.set_ylim(-0.5, len(STATE_ORDER) - 0.5)
    _style(ax, "FSM state", "Time [s]", "State")

    ax = axes[1][1]
    ax.plot(t, rec.array("pwm"), color="#9467bd", linewidth=1.4, label="PWM")
    _style(ax, "PWM command and motor current", "Time [s]", "PWM [per-mille]")
    ax2 = ax.twinx()
    ax2.plot(t, rec.array("current"), color="#8c564b", linewidth=1.1, label="Current")
    ax2.set_ylabel("Current [A]", fontsize=10)
    ax2.tick_params(labelsize=9)
    lines = ax.get_lines()[:1] + ax2.get_lines()[:1]
    ax.legend(lines, [ln.get_label() for ln in lines], fontsize=8, loc="best")

    if metrics:
        keys = ("rise_time_s", "overshoot_pct", "settling_time_s", "peak_force_n", "force_rmse_n")
        text = "\n".join(
            "{0}: {1:.4g}".format(k, float(metrics.get(k, 0.0))) for k in keys
        )
        fig.text(0.995, 0.01, text, fontsize=8, ha="right", va="bottom",
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    dest = path or os.path.join(default_outdir(outdir), "scenario_{0}_summary.png".format(scenario_key))
    return _finish(fig, dest)


def plot_experiment_01_step_response(
    rec: Recorder,
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Experiment 01: step response",
) -> str:
    """Plot a step response with rise time, overshoot and settling time annotated.

    The annotations are drawn from :func:`simulator.telemetry.metrics_from_recorder`
    rather than being placed by hand, so the arrows point at the values that the
    metrics table reports.  Dashed guides mark the 10 % and 90 % levels and the
    +/-2 % settling band.

    Parameters
    ----------
    rec:
        Recorder from a step-reference run.
    path, outdir, title:
        Output location and title.

    Returns
    -------
    str
        The path of the PNG written.
    """
    t = _require_data(rec, "step response")
    pos = rec.array("pos")
    ref = rec.array("ref")
    met = metrics_from_recorder(rec)
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(t, ref, "k--", linewidth=1.3, label="Reference")
    ax.plot(t, pos, color="#1f77b4", linewidth=1.7, label="Measured")

    start = float(pos[0])
    final = float(ref[-1])
    step = final - start
    if abs(step) > 1.0:
        lo = start + 0.10 * step
        hi = start + 0.90 * step
        for level, label, colour in ((lo, "10 %", "#7f7f7f"), (hi, "90 %", "#7f7f7f")):
            ax.axhline(level, color=colour, linestyle=":", linewidth=0.9, label=label)
        band = 0.02 * abs(step)
        ax.axhspan(final - band, final + band, color="#2ca02c", alpha=0.12,
                   label="+/-2 % band")

        rise = met.get("rise_time_s", 0.0)
        if rise and rise > 0.0:
            t_end = float(t[0]) + rise
            y_top = max(hi, lo)
            ax.annotate(
                "",
                xy=(t_end, y_top),
                xytext=(float(t[0]), y_top),
                arrowprops=dict(arrowstyle="<->", color="#1f77b4", linewidth=1.3),
            )
            ax.text(
                (float(t[0]) + t_end) * 0.5, y_top + 0.06 * abs(step),
                "rise time = {0:.3f} s".format(rise),
                color="#1f77b4", fontsize=9, ha="center",
            )
        peak = float(np.max(pos))
        overshoot = met.get("overshoot_pct", 0.0)
        t_peak = float(t[int(np.argmax(pos))])
        ax.plot([t_peak], [peak], marker="v", color="#d62728", markersize=7,
                label="Peak {0:.2f} mrad".format(peak))
        ax.annotate(
            "overshoot = {0:.2f} %".format(overshoot),
            xy=(t_peak, peak),
            xytext=(t_peak, peak + 0.18 * abs(step)),
            arrowprops=dict(arrowstyle="->", color="#d62728", linewidth=1.2),
            color="#d62728", fontsize=9,
        )
        settle = met.get("settling_time_s", 0.0)
        if settle and settle == settle:  # NaN-safe when it never settled
            ax.axvline(settle, color="#ff7f0e", linestyle="-.", linewidth=1.1,
                       label="settling = {0:.3f} s".format(settle))
    _style(ax, title, "Time [s]", "Joint position [mrad]")
    ax.legend(loc="lower right", fontsize=8)
    dest = path or os.path.join(default_outdir(outdir), "experiment_01_step_response.png")
    return _finish(fig, dest)


def plot_scenario_comparison(
    results: Dict[str, Dict[str, float]],
    path: Optional[str] = None,
    outdir: Optional[str] = None,
    title: str = "Scenario comparison",
) -> str:
    """Draw the cross-scenario comparison figure used by ``--scenario all``.

    Two panels: peak grip force per scenario with the damage threshold marked, and
    the settling/steady-state error per scenario.  These are the two quantities
    that distinguish "worked" from "faulted safely".

    Parameters
    ----------
    results:
        Mapping ``scenario key -> metrics dict``.
    path, outdir, title:
        Output location and title.

    Returns
    -------
    str
        The path of the PNG written.
    """
    keys = [k for k in C.SCENARIO_IDS if k in results]
    if not keys:
        raise NoDataError("no scenario results to compare")
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8))
    peaks = [float(results[k].get("peak_force_n", 0.0)) for k in keys]
    rmse = [float(results[k].get("force_rmse_n", 0.0)) for k in keys]
    ax = axes[0]
    bars = ax.bar(keys, peaks, color="#2ca02c", alpha=0.85)
    ax.axhline(C.DAMAGE_FORCE_N, color="#d62728", linestyle="--", linewidth=1.2,
               label="Damage {0:.1f} N".format(C.DAMAGE_FORCE_N))
    ax.axhline(C.FORCE_TARGET_N, color="black", linestyle=":", linewidth=1.1,
               label="Nominal target {0:.2f} N".format(C.FORCE_TARGET_N))
    for rect, val in zip(bars, peaks):
        ax.text(rect.get_x() + rect.get_width() / 2.0, val, "{0:.2f}".format(val),
                ha="center", va="bottom", fontsize=8)
    _style(ax, "Peak grip force by scenario", "Scenario", "Peak force [N]")
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.bar(keys, rmse, color="#1f77b4", alpha=0.85)
    _style(ax, "Grip-force RMS error by scenario", "Scenario", "Force RMSE [N]")
    fig.suptitle(title, fontsize=12)
    dest = path or os.path.join(default_outdir(outdir), "scenario_comparison.png")
    return _finish(fig, dest)
