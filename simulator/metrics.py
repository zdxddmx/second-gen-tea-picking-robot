"""
Response metrics derived from a recorded run.

What
----
:func:`metrics` computes the standard control and grasp figures of merit from the
recorded time series: rise time (10 % -> 90 %), overshoot, settling time (2 %
band), steady-state error, peak force, force RMSE and accumulated damage index.
:func:`metrics_from_recorder` is the convenience wrapper that also fills in the
PWM and current peaks, and :func:`force_band_entry_s` finds the moment the grip
enters its settle band.

Why
---
A plot is an argument; a number is evidence.  The same run must be summarisable so
that two controller variants can be compared without a human squinting at
overlaid curves.

Two definitions matter more than the rest, and both are chosen to measure what the
run is actually being asked about:

*Force RMSE* is evaluated only over the *regulation* window -- from the first
instant the force holds inside the settle band until detachment begins.  Before
contact the measured force is legitimately zero, the approach ramp is identical in
every run, and after the grip ends the jaw is opening on purpose.  Including any of
those makes a well-regulated grip indistinguishable from a wandering one.

*Settling time* is measured on the last departure from the 2 % band, so a response
that oscillates back out of the band is not credited with settling early.

What happens without it
-----------------------
Results would exist only as pictures, and a claim about, say, overshoot could not
be checked against the data it came from.

Units
-----
Time in s, position in mrad, force in N, current in A, PWM in per-mille.
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import numpy as np

from . import config as C
from .telemetry import Recorder


def _value_at_or_before(
    series: np.ndarray,
    times: np.ndarray,
    target: float,
    fallback: float,
) -> float:
    """Return the sample of ``series`` at ``target`` seconds.

    Parameters
    ----------
    series:
        Sampled channel.
    times:
        Sample times, same length as ``series``.
    target:
        Time of interest.
    fallback:
        Value returned when the series is empty or ``target`` precedes it.

    Returns
    -------
    float
        The nearest preceding sample, which is what a zero-order-hold signal
        actually holds at that instant.
    """
    if series.size == 0 or times.size == 0:
        return fallback
    idx = int(np.searchsorted(times, target, side="right")) - 1
    if idx < 0:
        return float(series[0])
    if idx >= series.size:
        return float(series[-1])
    return float(series[idx])


def metrics(
    t: Sequence[float],
    ref: Sequence[float],
    pos: Sequence[float],
    force: Sequence[float],
    window_start_s: Optional[float] = None,
    window_end_s: Optional[float] = None,
    force_rmse_ref_n: float = 0.0,
    force_settle_from_s: Optional[float] = None,
    force_window_end_s: Optional[float] = None,
) -> Dict[str, float]:
    """Compute the standard response metrics for one run.

    Parameters
    ----------
    t, ref, pos, force:
        Time (s), position reference (mrad), position (mrad) and force (N)
        channels.
    window_start_s, window_end_s:
        Optional evaluation window.  Scenario E needs it: once the link is lost
        the plant is frozen, and averaging over the frozen tail would report a
        flattering steady-state error for a run that actually faulted.
    force_rmse_ref_n:
        Reference force for the RMSE term, normally the achieved target.
    force_settle_from_s, force_window_end_s:
        Optional bounds on the force-RMSE window; see the module docstring.

    Returns
    -------
    Dict[str, float]
        ``rise_time_s``, ``overshoot_pct``, ``settling_time_s``,
        ``steady_state_error_mrad``, ``final_error_mrad``, ``peak_force_n``,
        ``force_rmse_n``, ``force_rmse_samples``, ``damage_index_ns``,
        ``pwm_peak_permille``, ``current_peak_a`` and ``n_samples``.
    """
    tt = np.asarray(t, dtype=float)
    rr = np.asarray(ref, dtype=float)
    pp = np.asarray(pos, dtype=float)
    ff = np.asarray(force, dtype=float)
    out: Dict[str, float] = {
        "rise_time_s": 0.0,
        "overshoot_pct": 0.0,
        "settling_time_s": 0.0,
        "steady_state_error_mrad": 0.0,
        "peak_force_n": 0.0,
        "force_rmse_n": 0.0,
        "force_rmse_samples": 0.0,
        "damage_index_ns": 0.0,
        "pwm_peak_permille": 0.0,
        "current_peak_a": 0.0,
        "final_error_mrad": 0.0,
        "n_samples": float(tt.size),
    }
    if tt.size < 2:
        return out

    mask = np.ones_like(tt, dtype=bool)
    if window_start_s is not None:
        mask &= tt >= float(window_start_s)
    if window_end_s is not None:
        mask &= tt <= float(window_end_s)
    if int(np.count_nonzero(mask)) < 2:
        mask = np.ones_like(tt, dtype=bool)

    t_w = tt[mask]
    ref_w = rr[mask]
    pos_w = pp[mask]
    force_w = ff[mask] if ff.size == tt.size else np.zeros_like(t_w)

    start_pos = float(pos_w[0])
    tail_n = max(1, int(0.1 * ref_w.size))
    final_ref = float(np.mean(ref_w[-tail_n:]))
    step = final_ref - start_pos
    out["final_error_mrad"] = float(ref_w[-1] - pos_w[-1])
    out["steady_state_error_mrad"] = float(
        np.mean(ref_w[-tail_n:] - pos_w[-max(1, int(0.1 * pos_w.size)):])
    )
    out["peak_force_n"] = float(np.max(force_w)) if force_w.size else 0.0
    if t_w.size > 1:
        dt_mean = float(np.mean(np.diff(t_w)))
        out["damage_index_ns"] = float(
            np.sum(np.maximum(0.0, force_w - C.DAMAGE_FORCE_N)) * dt_mean
        )

    if abs(step) > 1.0:
        rising = step > 0.0
        lo_thr = start_pos + 0.10 * step
        hi_thr = start_pos + 0.90 * step
        above_lo = pos_w >= lo_thr if rising else pos_w <= lo_thr
        above_hi = pos_w >= hi_thr if rising else pos_w <= hi_thr
        idx_lo = int(np.argmax(above_lo)) if bool(np.any(above_lo)) else -1
        idx_hi = int(np.argmax(above_hi)) if bool(np.any(above_hi)) else -1
        if idx_lo >= 0 and idx_hi >= 0:
            out["rise_time_s"] = float(max(0.0, t_w[idx_hi] - t_w[idx_lo]))
        peak = float(np.max(pos_w)) if rising else float(np.min(pos_w))
        if rising:
            out["overshoot_pct"] = float(max(0.0, (peak - final_ref) / step * 100.0))
        else:
            out["overshoot_pct"] = float(max(0.0, (final_ref - peak) / abs(step) * 100.0))
        band = 0.02 * abs(step)
        outside = np.abs(pos_w - final_ref) > band
        if bool(np.any(outside)):
            last_out = int(np.max(np.nonzero(outside)[0]))
            if last_out < t_w.size - 1:
                out["settling_time_s"] = float(t_w[last_out] - t_w[0])
            else:
                # Never settled inside the band; NaN records that honestly rather
                # than reporting a settling time equal to the whole run.
                out["settling_time_s"] = float("nan")
        else:
            out["settling_time_s"] = 0.0

    # Force RMSE is only meaningful once contact has occurred; before that the
    # controller is intentionally commanding zero force.
    if force_w.size and float(np.max(force_w)) > C.FORCE_CONTACT_N:
        contact_idx = int(np.argmax(force_w > C.FORCE_CONTACT_N))
        if force_settle_from_s is not None:
            idx = int(np.searchsorted(t_w, float(force_settle_from_s), side="left"))
            contact_idx = max(contact_idx, min(idx, force_w.size - 1))
        stop_idx = force_w.size
        if force_window_end_s is not None:
            stop_idx = int(np.searchsorted(t_w, float(force_window_end_s), side="left"))
            stop_idx = max(contact_idx + 1, min(stop_idx, force_w.size))
        tail = force_w[contact_idx:stop_idx]
        out["force_rmse_n"] = float(np.sqrt(np.mean((tail - float(force_rmse_ref_n)) ** 2)))
        out["force_rmse_samples"] = float(tail.size)
    return out


def metrics_from_recorder(
    rec: Recorder,
    window_start_s: Optional[float] = None,
    window_end_s: Optional[float] = None,
    force_rmse_ref_n: float = 0.0,
    force_settle_from_s: Optional[float] = None,
    force_window_end_s: Optional[float] = None,
) -> Dict[str, float]:
    """Compute :func:`metrics` plus the PWM and current peaks.

    Parameters
    ----------
    rec:
        Populated recorder.
    window_start_s, window_end_s:
        Optional evaluation window, forwarded to :func:`metrics`.
    force_rmse_ref_n:
        Reference force for the RMSE term.
    force_settle_from_s, force_window_end_s:
        Optional bounds on the force-regulation window, forwarded to
        :func:`metrics`.

    Returns
    -------
    Dict[str, float]
        The metrics dictionary, with ``pwm_peak_permille`` and ``current_peak_a``
        filled in from the corresponding channels over the same window.
    """
    out = metrics(
        rec.array("t"),
        rec.array("ref"),
        rec.array("pos"),
        rec.array("force"),
        window_start_s=window_start_s,
        window_end_s=window_end_s,
        force_rmse_ref_n=force_rmse_ref_n,
        force_settle_from_s=force_settle_from_s,
        force_window_end_s=force_window_end_s,
    )
    t = rec.array("t")
    pwm = rec.array("pwm")
    cur = rec.array("current")
    mask = np.ones_like(t, dtype=bool)
    if window_start_s is not None:
        mask &= t >= float(window_start_s)
    if window_end_s is not None:
        mask &= t <= float(window_end_s)
    if int(np.count_nonzero(mask)) < 1:
        mask = np.ones_like(t, dtype=bool)
    if pwm.size == t.size and pwm.size:
        out["pwm_peak_permille"] = float(np.max(np.abs(pwm[mask])))
    if cur.size == t.size and cur.size:
        out["current_peak_a"] = float(np.max(np.abs(cur[mask])))
    return out


def force_band_entry_s(
    t: Sequence[float],
    force: Sequence[float],
    target_n: float,
    band_n: float,
    hold_s: float,
    window_start_s: float = 0.0,
) -> Optional[float]:
    """Return the first time the force stays within a band for ``hold_s``.

    Parameters
    ----------
    t, force:
        Time (s) and measured force (N) channels.
    target_n:
        Setpoint the band is centred on, N.
    band_n:
        Half-width of the band, N.
    hold_s:
        How long the force must remain inside the band, seconds.
    window_start_s:
        Ignore samples before this time, s.  The search must not start at zero:
        before contact the measured force sits near zero, which counts as "inside
        the band" for a low target, and the window would then begin seconds before
        the gripper had even touched the stem.

    Returns
    -------
    float or None
        The start of the first qualifying window, or ``None`` when the force never
        stays inside the band that long.  This is the same criterion the gripper
        controller uses to declare the target reached, so the window it defines is
        exactly the regulation phase rather than the approach.
    """
    tt = np.asarray(t, dtype=float)
    ff = np.asarray(force, dtype=float)
    if tt.size == 0:
        return None
    inside = np.logical_and(
        np.abs(ff - float(target_n)) < float(band_n), tt >= float(window_start_s)
    )
    if tt.size > 1:
        dt_mean = float(np.mean(np.diff(tt)))
    else:
        dt_mean = C.CONTROL_DT_S
    needed = max(1, int(round(float(hold_s) / max(dt_mean, C.EPS))))
    run = 0
    start = 0
    for i in range(inside.size):
        if inside[i]:
            if run == 0:
                start = i
            run += 1
            if run >= needed:
                return float(tt[start])
        else:
            run = 0
    return None
