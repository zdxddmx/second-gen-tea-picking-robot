"""
Scenario data types and the pass/fail criteria for the seven scenarios.

What
----
:class:`ScenarioSpec` describes everything that distinguishes one scenario from
another; :class:`ScenarioResult` carries the outcome.  :func:`verdict` decides
whether a run met its own pass criterion.

Why the criteria live here
--------------------------
A scenario's pass condition is *not* "no fault".  Scenarios D, E, F and G pass
precisely *because* a fault was detected and handled, so the criterion is
per-scenario and belongs next to the scenario definitions rather than inside the
control loop.  Keeping it a pure function of the recorded result also means a
verdict can be re-derived from a saved log without re-simulating.

What happens without it
-----------------------
Each scenario would grow its own ad-hoc success check, and the distinction
between "the machine worked" and "the machine failed safely" would be lost --
which is the single most important distinction in this test set.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from . import config as C
from .fsm import State
from .telemetry import Recorder


@dataclass
class ScenarioSpec:
    """Everything that distinguishes one scenario from another.

    Attributes
    ----------
    key:
        Scenario letter.
    title:
        Human-readable name.
    duration_s:
        Run length, seconds.
    noise_sigma_n:
        Standard deviation of the additive force-sensor noise, N.
    sensor_filter:
        Whether the force channel's low-pass filter is active.  Scenario C runs
        both arms so the benefit of filtering is measured rather than asserted.
    f_target_override_n:
        Explicit grip-force setpoint, N; ``None`` uses the configured nominal
        target (or the adaptive threshold when it is enabled).
    target_extra_mrad:
        Joint-position error injected into the vision target, mrad.
    lateral_error_mm:
        Reported lateral target error, mm.  Telemetry only: the plant is a single
        joint, so a lateral offset cannot be actuated, only accounted for.
    depth_error_mm:
        Reported depth target error, mm.
    link_drop_s:
        Time at which the host link is cut, s.
    clamp_joint_at_s:
        Time at which the joint is jammed, s.
    clamp_theta_mrad:
        Angle at which the joint is jammed, mrad.
    resist_torque_nm:
        Environmental resisting torque applied after the jam, N*m.
    estop_at_s:
        Time at which the E-stop is asserted, s.
    stress:
        When ``True`` the loop keeps integrating after a fault so the guarded
        error regime itself is exercised.  Used by the long divergence check.
    """

    key: str
    title: str
    duration_s: float = C.DEFAULT_DURATION_S
    noise_sigma_n: float = 0.0
    sensor_filter: bool = True
    f_target_override_n: Optional[float] = None
    target_extra_mrad: float = 0.0
    lateral_error_mm: float = 0.0
    depth_error_mm: float = 0.0
    link_drop_s: Optional[float] = None
    clamp_joint_at_s: Optional[float] = None
    clamp_theta_mrad: float = C.JOINT_HOME_MRAD
    resist_torque_nm: float = 0.0
    estop_at_s: Optional[float] = None
    stress: bool = False


@dataclass
class ScenarioResult:
    """Outcome of one scenario run.

    Attributes
    ----------
    key:
        Scenario letter.
    title:
        Human-readable name.
    success:
        Whether the scenario met its own pass criterion -- which is *not*
        "no fault": Scenarios D, E, F and G pass precisely because a fault was
        detected and handled.
    faults:
        Latched fault codes.
    metrics:
        Numeric results, from :func:`simulator.telemetry.metrics_from_recorder`
        plus scenario-specific entries.
    recorder:
        The full per-tick record.
    trace_lines:
        The ordered console trace produced by the run.  The CLI prints exactly
        these strings, and every number in them came from the simulation.
    success_reason:
        One-line explanation of why the pass/fail verdict was reached.
    """

    key: str
    title: str
    success: bool
    faults: List[str] = field(default_factory=list)
    metrics: Dict[str, float] = field(default_factory=dict)
    recorder: Optional[Recorder] = None
    trace_lines: List[str] = field(default_factory=list)
    success_reason: str = ""


def verdict(
    key: str,
    met: Dict[str, float],
    faults: List[str],
    halted: bool,
    state: State,
    cycle_completed: bool,
    damaged: bool,
) -> Tuple[bool, str]:
    """Decide whether a scenario passed, and why.

    Parameters
    ----------
    key:
        Scenario letter.
    met:
        The assembled metrics dictionary.
    faults:
        Latched fault codes.
    halted:
        ``True`` when the machine stopped on a fault or E-stop.
    state:
        Final FSM state.
    cycle_completed:
        ``True`` when the machine reached ``IDLE`` at the end of a pick.
    damaged:
        ``True`` when the stem damage flag latched.

    Returns
    -------
    Tuple[bool, str]
        ``(success, reason)``.  The reason is written to be readable in the CLI
        summary, so a reviewer can tell *why* a scenario failed without opening
        the log.
    """
    if key == "A":
        if cycle_completed and not faults:
            return True, "completed the full cycle with no faults"
        return False, "cycle incomplete (faults: {0})".format(faults or "none")
    if key == "B":
        err_pct = met.get("position_error_pct", 0.0)
        if cycle_completed and not faults and err_pct <= 5.0:
            return True, "completed despite the injected error ({0:.2f} % final error)".format(
                err_pct
            )
        if C.ERR_TARGET_UNREACHABLE in faults:
            return False, "target error exceeded the controller's authority"
        return False, "unexpected outcome (faults: {0})".format(faults or "none")
    if key == "C":
        rmse = met.get("force_rmse_n", 0.0)
        ok = met.get("force_ctrl_enabled", 0.0) > 0.5 and rmse < 0.25
        return (
            (True, "force tracking RMSE {0:.4f} N under noise".format(rmse))
            if ok
            else (False, "force tracking degraded (RMSE {0:.4f} N)".format(rmse))
        )
    if key == "D":
        # The question this scenario asks is "does the protection notice an
        # over-command, and does the machine stay safe?" -- not "did a nail
        # current ever exceed a number".  The command is deliberately larger than
        # the gripper can physically produce, so the pass condition is that the
        # force was pushed into the damage region *and* that the damage was
        # flagged and bounded.
        latched = damaged or met.get("damaged", 0.0) > 0.5
        peak = met.get("peak_force_n", 0.0)
        if latched and peak < C.FORCE_MAX_N:
            return True, (
                "protection triggered: soft limit at {0:.1f} N, damage latched at "
                "{1:.2f} N peak (bounded below the {2:.1f} N overload)".format(
                    C.FORCE_SOFT_LIMIT_N, peak, C.FORCE_MAX_N
                )
            )
        if not latched:
            return False, "no protection triggered at {0:.2f} N".format(peak)
        return False, "protection latched but the force exceeded the hard limit"
    if key == "E":
        if C.ERR_COMM_TIMEOUT in faults and met.get("pwm_after_halt_permille", 1.0) <= C.EPS:
            return True, "link timeout detected and the joint stopped (PWM 0)"
        return False, "timeout not handled (faults: {0})".format(faults or "none")
    if key == "F":
        if C.ERR_MOTOR_STALL in faults:
            delay_ms = max(0.0, met.get("halt_time_s", 0.0) - C.SCEN_F_CLAMP_AT_S) * 1000.0
            return True, "stall detected {0:.0f} ms after the jam (debounce {1} ms)".format(
                delay_ms, C.STALL_TRIP_MS
            )
        return False, "stall not detected"
    if key == "G":
        if halted and state is State.EMERGENCY_STOP:
            if met.get("pwm_after_halt_permille", 1.0) <= C.EPS:
                return True, "E-stop reached EMERGENCY_STOP with PWM at zero"
        return False, "E-stop did not reach EMERGENCY_STOP"
    return cycle_completed, "completed" if cycle_completed else "incomplete"
