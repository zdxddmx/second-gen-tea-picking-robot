"""
Discrete PID controllers mirroring the C firmware implementation.

What
----
:class:`PID` is a sample-based proportional-integral-derivative controller with
the *same* arithmetic as the firmware's ``pid.c``:

1. **Trapezoidal integration** of the error,
   ``integral += 0.5 * (err + err_prev) * dt``.
2. **Derivative on measurement** (not on error) so that a step change of the
   setpoint does not produce a derivative kick: ``d_raw = -(meas - meas_prev)/dt``.
3. **First-order low-pass filter** on that derivative with time constant
   ``tau_d``: ``d_filt += (d_raw - d_filt) * dt / (tau_d + dt)``.
4. **Integral clamp** to ``+/- i_max`` *before* the output is formed.
5. **Back-calculation anti-windup**: after clamping the output,
   ``integral += (out_clamped - out_unclamped) * dt / max(ki, eps)``, applied
   only when ``ki != 0``.
6. **Output clamp** to ``[out_min, out_max]``.
7. Optional **feed-forward** term ``ff_gain * vel_ff`` added to the output.

Why these choices
-----------------
*Derivative on measurement* removes the kick that would otherwise slam the
motor whenever the trajectory generator steps the reference; the firmware does
this, so the simulator must too or the two would disagree on overshoot.  The
*low-pass* is required because differentiating a 1 kHz encoder signal amplifies
quantisation noise by a factor of ~1/dt.  *Back-calculation* is preferred over
simple integral clamping because the three loops here are cascaded: when the
inner loop saturates, the outer loop must be told, or it keeps winding up and
the response overshoots badly on recovery.

What happens without it
-----------------------
Without the derivative filter the velocity loop amplifies encoder noise into
audible PWM chatter; without back-calculation the position loop winds up during
PWM saturation and overshoots the target by tens of degrees; without the
trapezoidal term the integrator drifts with sample rate.

Units are whatever the caller uses -- the class is unit-agnostic, and the
cascade in :mod:`simulator.controller` documents which unit each loop carries.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from . import config as C


def _finite(value: float, fallback: float = 0.0) -> float:
    """Return ``value`` when it is a finite float, else ``fallback``.

    Parameters
    ----------
    value:
        Candidate number, typically produced by arithmetic on sensor data.
    fallback:
        Value returned when ``value`` is ``inf``, ``-inf`` or ``nan``.

    Returns
    -------
    float
        A finite float.  This is the last line of defence against a single
        ``nan`` sample poisoning the integrator forever.
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    if math.isfinite(v):
        return v
    return float(fallback)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into ``[low, high]`` with a safe, ordered bound pair.

    Parameters
    ----------
    value:
        Number to clamp.
    low, high:
        Bounds.  If they arrive reversed (a caller bug) they are swapped rather
        than silently producing a nonsensical result.
    Returns
    -------
    float
        The clamped value.
    """
    lo, hi = (low, high) if low <= high else (high, low)
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


class LowPassFilter:
    """First-order low-pass filter, ``y += (x - y) * dt / (tau + dt)``.

    This is the exact discrete form used by both the PID derivative filter and
    the firmware's force signal conditioning (``TP_FILTER_FORCE_LPF_TAU_S``).
    A ``tau`` of zero makes :meth:`update` a pass-through, which is how the
    simulator can emulate the *unfiltered* channel of Scenario C.
    """

    def __init__(self, tau_s: float, dt: float, initial: float = 0.0) -> None:
        """Create the filter.

        Parameters
        ----------
        tau_s:
            Time constant in seconds.  ``0.0`` means "no filtering".
        dt:
            Sample time in seconds.
        initial:
            Starting output value; seeding with the first sample avoids a
            startup transient from zero.
        """
        self.tau_s: float = max(0.0, _finite(tau_s))
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.y: float = _finite(initial)

    def reset(self, value: float = 0.0) -> None:
        """Reset the filter state to ``value``."""
        self.y = _finite(value)

    def update(self, x: float) -> float:
        """Feed one sample and return the filtered output."""
        xv = _finite(x, self.y)
        if self.tau_s <= 0.0:
            self.y = xv
        else:
            self.y = self.y + (xv - self.y) * self.dt / (self.tau_s + self.dt)
            self.y = _finite(self.y, xv)
        return self.y


class MovingAverage:
    """Fixed-length moving average, mirroring ``TP_FILTER_FORCE_MA_WINDOW``.

    Implemented as a circular buffer so that a 1 kHz run of 30 s allocates
    nothing after construction.
    """

    def __init__(self, window: int) -> None:
        """Create an ``window``-sample moving average (``window >= 1``)."""
        self.window: int = max(1, int(window))
        self._buf: List[float] = [0.0] * self.window
        self._sum: float = 0.0
        self._idx: int = 0
        self._count: int = 0
        self._primed: bool = False

    def reset(self) -> None:
        """Clear the buffer."""
        self._buf = [0.0] * self.window
        self._sum = 0.0
        self._idx = 0
        self._count = 0
        self._primed = False

    def update(self, x: float) -> float:
        """Feed one sample and return the current window mean.

        The buffer is *seeded with the first sample* rather than with zeros.  A
        zero-filled buffer makes the filter output ramp up from zero over the
        first ``window`` samples, which looks like a real 30 ms lag in the force
        signal and, on a 1 kHz loop, is long enough to delay contact detection and
        to bias the first force commands.
        """
        xv = _finite(x)
        if not self._primed:
            self._buf = [xv] * self.window
            self._sum = xv * self.window
            self._count = self.window
            self._idx = 0
            self._primed = True
            return xv
        if self._count < self.window:
            self._buf[self._idx] = xv
            self._sum += xv
            self._count += 1
        else:
            self._sum += xv - self._buf[self._idx]
            self._buf[self._idx] = xv
        self._idx = (self._idx + 1) % self.window
        denom = float(max(1, self._count))
        return self._sum / denom


class PID:
    """Sample-based PID with derivative-on-measurement and back-calculation.

    Attributes
    ----------
    kp, ki, kd:
        Gains.  Integral gain is per second, derivative gain in seconds.
    dt:
        Sample time, seconds.
    out_min, out_max:
        Output saturation limits.
    i_max:
        Symmetric integral clamp.
    tau_d:
        Derivative low-pass time constant, seconds.
    derivative_on_measurement:
        When ``True`` (the default, and what the firmware does) the derivative
        term is computed from the measurement so a setpoint step does not kick.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        out_min: float,
        out_max: float,
        i_max: float,
        tau_d: float,
        derivative_on_measurement: bool = True,
        ff_gain: float = 0.0,
        name: str = "pid",
    ) -> None:
        """Construct the controller and reset its state."""
        self.name: str = str(name)
        self.kp: float = _finite(kp)
        self.ki: float = _finite(ki)
        self.kd: float = _finite(kd)
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.out_min: float = _finite(out_min, -1.0)
        self.out_max: float = _finite(out_max, 1.0)
        if self.out_min > self.out_max:
            self.out_min, self.out_max = self.out_max, self.out_min
        self.i_max: float = abs(_finite(i_max))
        self.tau_d: float = max(0.0, _finite(tau_d))
        self.derivative_on_measurement: bool = bool(derivative_on_measurement)
        self.ff_gain: float = _finite(ff_gain)
        self.reset()

    # -- configuration ------------------------------------------------------
    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Replace the three gains without disturbing the integrator.

        Keeping the integrator is deliberate: a gain schedule applied mid-motion
        should not create a position step.
        """
        self.kp = _finite(kp, self.kp)
        self.ki = _finite(ki, self.ki)
        self.kd = _finite(kd, self.kd)

    def set_limits(
        self,
        out_min: float,
        out_max: float,
        i_max: Optional[float] = None,
    ) -> None:
        """Replace the output limits and, optionally, the integral clamp."""
        lo = _finite(out_min, self.out_min)
        hi = _finite(out_max, self.out_max)
        self.out_min, self.out_max = (lo, hi) if lo <= hi else (hi, lo)
        if i_max is not None:
            self.i_max = abs(_finite(i_max, self.i_max))
        # Re-clamp the live integrator so a tightened limit takes effect at once.
        self.integral = clamp(self.integral, -self.i_max, self.i_max)

    def reset(self) -> None:
        """Clear integrator, derivative filter and history.

        Called on every state-machine entry that changes the controlled
        variable, otherwise the stale integrator would produce a bump.
        """
        self.integral: float = 0.0
        self.d_filt: float = 0.0
        self.err_prev: float = 0.0
        self.meas_prev: float = 0.0
        self.err_prev_available: bool = False
        self.meas_prev_available: bool = False
        self.initialised: bool = False
        self.last_output: float = 0.0
        self.last_unclamped: float = 0.0
        self.last_p: float = 0.0
        self.last_i: float = 0.0
        self.last_d: float = 0.0
        self.last_ff: float = 0.0
        self.saturated: bool = False

    # -- runtime ------------------------------------------------------------
    def update(
        self,
        setpoint: float,
        measurement: float,
        vel_ff: float = 0.0,
    ) -> float:
        """Run one control tick.

        Parameters
        ----------
        setpoint:
            Desired value of the controlled variable.
        measurement:
            Measured value of the controlled variable.
        vel_ff:
            Feed-forward input.  It is multiplied by :attr:`ff_gain` and added
            to the controller output, so a velocity/torque feed-forward path can
            be enabled without touching the gains.

        Returns
        -------
        float
            Saturated controller output in ``[out_min, out_max]``.
        """
        sp = _finite(setpoint, self.last_output)
        meas = _finite(measurement, sp)
        err = sp - meas

        # First call has no history: seed it so the derivative starts at zero
        # instead of exploding because of an implied infinite slope.
        if not self.initialised:
            self.err_prev = err
            self.meas_prev = meas
            self.err_prev_available = True
            self.meas_prev_available = True
            self.initialised = True

        # --- (1) trapezoidal integration + (4) integral clamp ---------------
        self.integral += 0.5 * (err + self.err_prev) * self.dt
        if not math.isfinite(self.integral):
            self.integral = 0.0
        if self.ki != 0.0:
            self.integral = clamp(self.integral, -self.i_max, self.i_max)

        # --- (2)+(3) derivative on measurement, low-pass filtered -----------
        src_now = meas if self.derivative_on_measurement else err
        src_prev = self.meas_prev if self.derivative_on_measurement else self.err_prev
        d_raw = -(src_now - src_prev) / self.dt
        if self.tau_d > 0.0:
            self.d_filt += (d_raw - self.d_filt) * self.dt / (self.tau_d + self.dt)
        else:
            self.d_filt = d_raw
        d_filt = _finite(self.d_filt, 0.0)

        # --- proportional / integral / derivative / feed-forward ------------
        p_term = self.kp * err
        i_term = self.ki * self.integral
        d_term = self.kd * d_filt
        ff_term = self.ff_gain * _finite(vel_ff, 0.0)
        out_unclamped = p_term + i_term + d_term + ff_term

        # --- (6) output clamp and (5) back-calculation anti-windup ----------
        out_clamped = clamp(_finite(out_unclamped, 0.0), self.out_min, self.out_max)
        if self.ki != 0.0 and out_clamped != out_unclamped:
            self.integral += (out_clamped - out_unclamped) * self.dt / max(
                abs(self.ki), C.EPS
            )
            self.integral = clamp(_finite(self.integral, 0.0), -self.i_max, self.i_max)

        self.err_prev = err
        self.meas_prev = meas
        self.last_p = p_term
        self.last_i = self.ki * self.integral
        self.last_d = d_term
        self.last_ff = ff_term
        self.last_unclamped = out_unclamped
        self.last_output = out_clamped
        self.saturated = out_clamped != out_unclamped
        return out_clamped

    def terms(self) -> Sequence[float]:
        """Return ``(P, I, D, FF, out)`` from the most recent :meth:`update`.

        Used by the logger to identify which term dominates a poor response.
        """
        return (self.last_p, self.last_i, self.last_d, self.last_ff, self.last_output)

    def __repr__(self) -> str:
        """Return a compact debugging representation."""
        return (
            "PID({0}, kp={1:.4g}, ki={2:.4g}, kd={3:.4g}, "
            "out=[{4:.4g}, {5:.4g}], i_max={6:.4g}, tau_d={7:.4g})"
        ).format(
            self.name,
            self.kp,
            self.ki,
            self.kd,
            self.out_min,
            self.out_max,
            self.i_max,
            self.tau_d,
        )


class RateLimiter:
    """Symmetric slew-rate limiter, mirroring ``TP_MAX_DUTY_RATE_PER_S``.

    The firmware applies this to the PWM command so that a large step in the
    velocity loop cannot ask the H-bridge for an impossible dV/dt.
    """

    def __init__(self, rate_per_s: float, dt: float, initial: float = 0.0) -> None:
        """Create the limiter with a maximum slope of ``rate_per_s`` units/s."""
        self.rate_per_s: float = abs(_finite(rate_per_s))
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.value: float = _finite(initial)

    def reset(self, value: float = 0.0) -> float:
        """Reset the limiter state to ``value`` and return it."""
        self.value = _finite(value)
        return self.value

    def update(self, target: float) -> float:
        """Approach ``target`` without exceeding the configured rate."""
        tgt = _finite(target, self.value)
        max_step = self.rate_per_s * self.dt
        delta = tgt - self.value
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.value = _finite(self.value + delta, tgt)
        return self.value
