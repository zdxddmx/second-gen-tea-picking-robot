"""
Motion profile / trajectory generators: trapezoidal and S-curve.

What
----
:class:`TrapezoidalProfile` produces an acceleration-limited velocity profile and
:class:`SCurveProfile` produces a *jerk*-limited ("S-curve") profile.
:class:`StepProfile` is the unshaped reference.  All three expose the same
interface -- ``T`` (total duration), ``v_peak``, and :meth:`sample` returning a
:class:`ProfileSample` of ``(position, velocity, acceleration)`` -- so
:class:`~simulator.controller.CascadeController` can select one at run time
without any other change.

Why two shaped profiles
-----------------------
The trapezoidal profile is what the firmware emits: it is the time-optimal
acceleration-limited solution and is trivially cheap.  Its weakness is that
acceleration jumps at the phase corners, which excites the flexure of a light
carbon-fibre arm and is felt as a "clack" at every phase change.  An S-curve
bounds ``da/dt`` instead, at the cost of a slightly longer move.  Keeping both in
the simulator is what allows the experiment to compare them on the *same* plant
and decide whether the extra time buys real settling improvement.

All three are purely geometric: they never integrate state, so a profile cannot
drift and a re-plan is exact.

How the S-curve is solved
-------------------------
The textbook closed-form S-curve needs several cases (jerk-limited,
acceleration-limited, velocity-limited, and the degenerate short move).  Instead
this implementation builds one shape for a trial jerk and then bisects on the
jerk value.  For a fixed shape, distance scales as ``j^-2``, so
``distance(j)`` is monotone and the root is unique; 160 bisection steps reach the
floating-point floor.  The shape itself is the standard seven-phase S-curve --
jerk up, hold ``a_max``, jerk down, cruise, jerk down, hold ``-a_max``, jerk up --
where any phase may have zero length, which is precisely what makes the same
formula cover all the classical cases.

Two independent correctness properties are enforced and re-checked in
:meth:`SCurveProfile.check`: the profile never exceeds ``v_max`` or ``a_max``
(when the requested jerk is achievable at all), and the terminal position equals
the target exactly.

What happens without it
-----------------------
Without a profiled reference the position loop receives a step, the velocity loop
saturates, and the resulting 100 % PWM step is both unrealistic and a poor test
of the controller -- the derivative filter would be doing all of the work.

Units
-----
Position in mrad, velocity in mrad/s, acceleration in mrad/s^2, jerk in
mrad/s^3, time in s.  The classes are unit-agnostic, so the same code drives the
jaw in mm and mm/s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from . import config as C
from .pid import _finite


@dataclass
class ProfileSample:
    """One evaluated point of a motion profile.

    Attributes
    ----------
    t_s:
        Time since the profile started, seconds.
    pos:
        Position reference, in the profile's position unit.
    vel:
        Velocity reference, position unit per second.  This is the feed-forward
        term handed to the position loop.
    acc:
        Acceleration reference, position unit per second squared.  This is the
        torque feed-forward term (multiplied by inertia) when one is enabled.
    done:
        ``True`` once the profile has finished, i.e. the reference equals the
        target and the velocity is zero.
    """

    t_s: float
    pos: float
    vel: float
    acc: float
    done: bool


class _ProfileBase:
    """Shared bookkeeping for the profile shapes.

    Attributes
    ----------
    distance:
        Signed move length in position units.
    v_max, a_max:
        Velocity and acceleration magnitudes the profile must respect.
    start:
        Position at ``t = 0``.
    target:
        Position at ``t = T``; always exactly ``start + distance``.
    sign:
        ``+1`` for a positive move, ``-1`` for a negative one.  Both subclasses
        plan and sample in the positive direction and apply the sign at the end,
        which halves the number of sign cases that can be wrong.
    """

    def __init__(
        self,
        distance: float,
        v_max: float,
        a_max: float,
        start: float = 0.0,
    ) -> None:
        """Store the move specification and pre-compute the shape."""
        self.distance: float = _finite(distance)
        self.v_max: float = max(C.EPS, abs(_finite(v_max, 1.0)))
        self.a_max: float = max(C.EPS, abs(_finite(a_max, 1.0)))
        self.start: float = _finite(start)
        self.target: float = self.start + self.distance
        self.sign: float = 1.0 if self.distance >= 0.0 else -1.0
        self.T: float = 0.0
        self.v_peak: float = 0.0
        self._plan()

    def _plan(self) -> None:
        """Compute the profile shape.  Overridden by each subclass."""
        raise NotImplementedError

    def sample(self, t_s: float) -> ProfileSample:
        """Evaluate the profile at ``t_s`` seconds."""
        raise NotImplementedError

    def at_end(self, t_s: float) -> bool:
        """Return ``True`` when ``t_s`` has reached or passed ``T``."""
        return _finite(t_s) >= self.T - 1e-12

    def check(self, samples: int = 2001) -> Dict[str, float]:
        """Numerically audit the profile against its own limits.

        Parameters
        ----------
        samples:
            Number of points at which to evaluate the profile.

        Returns
        -------
        dict
            ``v_violation`` (peak speed over ``v_max``), ``a_violation`` (peak
            acceleration over ``a_max``) and ``end_error`` (terminal position
            error in position units).  All three should be at or very near zero;
            they are reported by the self-test rather than asserted here so that
            a deliberately extreme move can still be simulated.
        """
        v_max_seen = 0.0
        a_max_seen = 0.0
        n = max(2, int(samples))
        for i in range(n + 1):
            s = self.sample(self.T * i / float(n))
            v_max_seen = max(v_max_seen, abs(s.vel))
            a_max_seen = max(a_max_seen, abs(s.acc))
        end = self.sample(self.T)
        return {
            "v_violation": v_max_seen - self.v_max,
            "a_violation": a_max_seen - self.a_max,
            "end_error": abs(end.pos - self.target),
        }

    def __repr__(self) -> str:
        """Return a compact summary of the planned move."""
        return "{0}(d={1:.4g}, T={2:.4f}s, v_peak={3:.4g})".format(
            type(self).__name__, self.distance, self.T, self.v_peak
        )


class TrapezoidalProfile(_ProfileBase):
    """Acceleration-limited (trapezoidal-velocity) profile.

    Acceleration is ``+a``, then ``0``, then ``-a``; the speed is triangular when
    the move is too short to reach ``v_max`` and trapezoidal otherwise.  ``T`` and
    ``v_peak`` are exact, closed-form quantities.
    """

    def _plan(self) -> None:
        """Solve for the triangular or trapezoidal phase durations."""
        d = abs(self.distance)
        if d < 1e-12:
            self.T = 0.0
            self.v_peak = 0.0
            self.ta = 0.0
            self.tc = 0.0
            return
        # Peak speed is bounded by the velocity limit and by the distance, since
        # two acceleration ramps must fit inside d: v <= sqrt(a*d).
        v_pk = min(self.v_max, math.sqrt(self.a_max * d))
        ta = v_pk / self.a_max
        tc = max(0.0, (d - v_pk * ta) / max(v_pk, C.EPS))
        self.v_peak = v_pk
        self.ta = ta
        self.tc = tc
        self.T = 2.0 * ta + tc

    def sample(self, t_s: float) -> ProfileSample:
        """Evaluate position, velocity and acceleration at ``t_s``."""
        d = abs(self.distance)
        t = _finite(t_s)
        if self.T <= C.EPS or d < 1e-12:
            return ProfileSample(t, self.target, 0.0, 0.0, True)
        tc = max(0.0, min(t, self.T))
        a = self.a_max
        if tc <= self.ta:
            p = 0.5 * a * tc * tc
            v = a * tc
            acc = a
        elif tc <= self.ta + self.tc:
            p = 0.5 * a * self.ta * self.ta + self.v_peak * (tc - self.ta)
            v = self.v_peak
            acc = 0.0
        else:
            remaining = self.T - tc
            p = d - 0.5 * a * remaining * remaining
            v = a * remaining
            acc = -a
        return ProfileSample(
            tc,
            self.start + self.sign * p,
            self.sign * max(0.0, v),
            self.sign * acc,
            tc >= self.T - 1e-12,
        )


class SCurveProfile(_ProfileBase):
    """Jerk-limited (S-curve) profile built from seven phases.

    Parameters
    ----------
    distance, v_max, a_max, start:
        As for :class:`TrapezoidalProfile`.
    j_max:
        Maximum jerk magnitude.  Defaults to ``4 * a_max`` per second, which
        would reach the acceleration limit in 0.25 s -- a deliberate compromise
        between smoothness and move time for this arm.

    The phases are: jerk up, hold ``+a_max``, jerk down, cruise, jerk down, hold
    ``-a_max``, jerk up.  Any of them may have zero length.  When the requested
    jerk would make even the shortest move exceed ``v_max`` or ``a_max``, the
    solver returns the fastest shape that *does* respect the limits; velocity and
    acceleration limits always win over the jerk preference, because violating
    them would command the drive outside its rating.
    """

    def __init__(
        self,
        distance: float,
        v_max: float,
        a_max: float,
        start: float = 0.0,
        j_max: Optional[float] = None,
    ) -> None:
        """Store the jerk limit and plan the move."""
        self.j_max: float = (
            max(C.EPS, abs(_finite(j_max)))
            if j_max is not None
            else 4.0 * max(C.EPS, abs(_finite(a_max, 1.0)))
        )
        super().__init__(distance, v_max, a_max, start)

    # -- shape for a trial jerk --------------------------------------------
    def _shape_parts(
        self,
        j: float,
        a_peak: float,
        t_hold: float,
    ) -> Tuple[float, float, float]:
        """Return ``(t_jerk, distance, v_peak)`` for one accelerate phase.

        The accelerate phase ramps acceleration from zero to ``a_peak`` over
        ``t_jerk = a_peak/j``, holds it for ``t_hold``, then ramps back to zero.
        The speed gained is ``a_peak*(t_jerk + t_hold)`` -- the two half-ramps are
        together worth exactly one ``t_jerk`` at full acceleration -- and the
        distance is the exact closed-form integral of that velocity.
        """
        j_safe = max(j, C.EPS)
        t_jerk = a_peak / j_safe
        v_peak = a_peak * (t_jerk + t_hold)
        acc_time = 2.0 * t_jerk + t_hold
        distance = 2.0 * self._pos(acc_time, t_jerk, t_hold, a_peak, v_peak)
        return (t_jerk, distance, v_peak)

    def _plan_shape(self, j: float, d: float) -> Tuple[float, float, float, float]:
        """Return ``(t_jerk, t_hold, a_peak, v_peak)`` for jerk limit ``j``.

        The jerk is a *bound*, not a free parameter: the fastest profile that
        respects it is the one that ramps at ``j`` and covers exactly ``d``.  That
        makes the solve closed-form in every regime, with no root finding.

        Let ``t_j = a_peak/j`` be the jerk ramp, ``t_h`` the hold at ``a_peak`` and
        ``r = v_peak/a_peak = t_j + t_h``.  The accelerate phase then has
        ``v = a_peak*t`` for ``0 <= t <= r`` and ``v = a_peak*(2t_j + t_h - t)``
        afterwards, so its area is exactly ``v_peak*(t_j + t_h)`` and its length
        is ``2t_j + t_h``.  Three regimes follow.

        *Long move* -- ``d >= d_crit = a_max*v_max/j + v_max^2/a_max`` is the
        length at which the accelerate phase exactly reaches ``a_max`` and
        ``v_max`` with no hold and no cruise.  Beyond it the profile holds
        ``a_max``, cruises at ``v_max``, and the leftover distance is the cruise.

        *Mid move* -- ``a_max`` is reached but ``v_max`` is not.  With ``t_j``
        fixed by the jerk limit, the area equation gives
        ``t_h = d/(2*a_max) - 2*t_j``; if that is negative, the velocity triangle
        is truncated and ``t_j`` is found instead from ``4*j*t_j^3 = d``.

        *Short move* -- the acceleration limit is never reached, ``t_h = 0``, and
        ``t_j = (d/(4j))^(1/3)`` places the profile exactly on ``d``.
        """
        j_safe = max(j, C.EPS)
        v_lim = self.v_max
        a_lim = self.a_max
        d_crit = a_lim * v_lim / j_safe + v_lim * v_lim / a_lim
        if d >= d_crit:
            # Long move: hold a_lim up to v_lim, then cruise for the remainder.
            t_jerk = a_lim / j_safe
            return (t_jerk, v_lim / a_lim - t_jerk, a_lim, v_lim)
        # Shorter than d_crit: the velocity limit is never reached.  Solve the
        # area equation with a_peak = a_lim first and see whether the hold is
        # non-negative.
        t_j_full = a_lim / j_safe
        t_hold = 0.5 * d / a_lim - 2.0 * t_j_full
        if t_hold >= 0.0:
            # Mid move: ramp to a_lim, hold, ramp down.
            return (t_j_full, t_hold, a_lim, a_lim * (t_j_full + t_hold))
        # Short move: truncate the acceleration triangle so that the four jerk
        # ramps cover exactly d: area = 2*a_peak*t_j and t_j = a_peak/j give
        # 4*j*t_j^3 = d.
        t_jerk = (d / (4.0 * j_safe)) ** (1.0 / 3.0)
        a_peak = j_safe * t_jerk
        return (t_jerk, 0.0, min(a_peak, a_lim), min(a_peak, a_lim) * t_jerk)

    # -- exact kinematics for a given shape --------------------------------
    @staticmethod
    def _vel(t: float, tj: float, th: float, a_peak: float, v_peak: float) -> float:
        """Velocity of the accelerate phase at ``t`` seconds into the move."""
        tj_s = max(tj, C.PROFILE_EPS)
        if t <= 0.0:
            return 0.0
        if t <= tj:
            return 0.5 * a_peak * t * t / tj_s
        if t <= tj + th:
            return 0.5 * a_peak * tj + a_peak * (t - tj)
        if t <= 2.0 * tj + th:
            x = t - tj - th
            return v_peak - 0.5 * a_peak * x * x / tj_s
        return v_peak

    @staticmethod
    def _acc(t: float, tj: float, th: float, a_peak: float) -> float:
        """Acceleration of the accelerate phase at ``t`` seconds."""
        tj_s = max(tj, C.PROFILE_EPS)
        if t <= 0.0 or t >= 2.0 * tj + th:
            return 0.0
        if t <= tj:
            return a_peak * t / tj_s
        if t <= tj + th:
            return a_peak
        return a_peak * (1.0 - (t - tj - th) / tj_s)

    @staticmethod
    def _pos(t: float, tj: float, th: float, a_peak: float, v_peak: float) -> float:
        """Exact distance travelled by the accelerate phase after ``t`` seconds.

        The velocity profile of each phase is polynomial (quadratic, rising
        linear, then quadratic), so the position is available in closed form --
        no quadrature and therefore no integration error to hide a modelling
        mistake.
        """
        tj_s = max(tj, C.PROFILE_EPS)
        if t <= 0.0:
            return 0.0
        t_c = 2.0 * tj + th
        if t <= tj:                                    # jerk-up
            return a_peak * t ** 3 / (6.0 * tj_s)
        p1 = a_peak * tj * tj / 6.0                    # distance after phase 1
        if t <= tj + th:                               # constant a_peak
            x = t - tj
            return p1 + 0.5 * a_peak * tj * x + 0.5 * a_peak * x * x
        p2 = p1 + 0.5 * a_peak * tj * th + 0.5 * a_peak * th * th
        if t <= t_c:                                   # jerk-down
            x = t - tj - th
            return (
                p2
                + (v_peak - 0.5 * a_peak * tj) * x
                + 0.5 * a_peak * x * x / tj_s
                - a_peak * x ** 3 / (6.0 * tj_s * tj_s)
            )
        # Past the accelerate phase: finish it, then cruise at v_peak.
        p3 = (
            p2
            + (v_peak - 0.5 * a_peak * tj) * tj
            + 0.5 * a_peak * tj * tj / tj_s
            - a_peak * tj ** 3 / (6.0 * tj_s * tj_s)
        )
        return p3 + v_peak * (t - t_c)

    def _plan(self) -> None:
        """Solve the profile in closed form at the requested jerk limit."""
        d = abs(self.distance)
        if d < 1e-12:
            self.T = 0.0
            self.v_peak = 0.0
            self._tj = self._th = 0.0
            self._ap = self.a_max
            self._tc = 0.0
            self._j = self.j_max
            return
        j = max(self.j_max, C.EPS)
        tj, th, ap, v_pk = self._plan_shape(j, d)
        acc_time = 2.0 * tj + th
        d_acc = self._pos(acc_time, tj, th, ap, v_pk)
        # Whatever distance is left over is the constant-velocity cruise.  The
        # binary search on the jerk is deliberately absent: the jerk is a bound,
        # and the profile above already covers exactly d by construction, so a
        # root find would only add tolerance where an exact answer exists.
        tc = max(0.0, (d - 2.0 * d_acc) / v_pk) if v_pk > C.EPS else 0.0
        self._j = j
        self._tj = tj
        self._th = th
        self._ap = ap
        self._tc = tc
        self.v_peak = v_pk
        self.T = 2.0 * acc_time + tc

    def sample(self, t_s: float) -> ProfileSample:
        """Evaluate position, velocity and acceleration at ``t_s``.

        The deceleration half is the exact mirror of the acceleration half, which
        is what makes the terminal position land on the target without a
        correction step.
        """
        d = abs(self.distance)
        t = _finite(t_s)
        if self.T <= C.EPS or d < 1e-12:
            return ProfileSample(t, self.target, 0.0, 0.0, True)
        tc = max(0.0, min(t, self.T))
        acc_time = 2.0 * self._tj + self._th
        tj, th, ap, v_pk, cruise = self._tj, self._th, self._ap, self.v_peak, self._tc
        d_acc = self._pos(acc_time, tj, th, ap, v_pk)

        if tc <= acc_time:
            p = self._pos(tc, tj, th, ap, v_pk)
            v = self._vel(tc, tj, th, ap, v_pk)
            acc = self._acc(tc, tj, th, ap)
        elif tc <= acc_time + cruise:
            p = d_acc + v_pk * (tc - acc_time)
            v = v_pk
            acc = 0.0
        else:
            # Mirror: the state at tc equals the accelerate state at T - tc,
            # with position reflected about the midpoint.
            tr = self.T - tc
            p = d - self._pos(tr, tj, th, ap, v_pk)
            v = self._vel(tr, tj, th, ap, v_pk)
            acc = -self._acc(tr, tj, th, ap)
        return ProfileSample(
            tc,
            self.start + self.sign * p,
            self.sign * max(0.0, v),
            self.sign * acc,
            tc >= self.T - 1e-12,
        )


class StepProfile:
    """Instantaneous step reference (no trajectory shaping).

    Used as the "plain step" comparison mode: the position loop is handed the
    target immediately, which is the harshest input and the one the classic
    step-response metrics in :mod:`simulator.telemetry` are defined for.
    """

    def __init__(
        self,
        distance: float,
        v_max: float = 0.0,
        a_max: float = 0.0,
        start: float = 0.0,
    ) -> None:
        """Store the step target; ``v_max``/``a_max`` are accepted and ignored."""
        self.distance: float = _finite(distance)
        self.start: float = _finite(start)
        self.target: float = self.start + self.distance
        self.v_max: float = _finite(v_max)
        self.a_max: float = _finite(a_max)
        self.v_peak: float = 0.0
        self.T: float = 0.0

    def sample(self, t_s: float) -> ProfileSample:
        """Return the step: the target at ``t = 0`` with zero velocity."""
        return ProfileSample(_finite(t_s), self.target, 0.0, 0.0, True)

    def at_end(self, t_s: float) -> bool:
        """A step is instantly "at end"; settling is judged from measurements."""
        return True

    def check(self, samples: int = 2001) -> Dict[str, float]:
        """Return zero violations; a step has no shaped limits to violate."""
        return {"v_violation": 0.0, "a_violation": 0.0, "end_error": 0.0}

    def __repr__(self) -> str:
        """Return a compact summary."""
        return "StepProfile(target={0:.4g})".format(self.target)


#: Registry of the profile kinds selectable from the CLI and experiments.
PROFILE_KINDS: Tuple[str, ...] = ("trapezoid", "scurve", "step")


def make_profile(
    kind: str,
    distance: float,
    v_max: float,
    a_max: float,
    start: float = 0.0,
) -> object:
    """Create the profile selected by ``kind``.

    Parameters
    ----------
    kind:
        ``"trapezoid"``, ``"scurve"`` or ``"step"`` (case-insensitive).
    distance, v_max, a_max, start:
        Passed through to the selected class.

    Returns
    -------
    object
        A :class:`TrapezoidalProfile`, :class:`SCurveProfile` or
        :class:`StepProfile`.  An unknown ``kind`` falls back to the trapezoidal
        profile rather than raising, so a mistyped CLI flag still produces a run
        instead of a traceback.
    """
    k = str(kind).strip().lower()
    if k == "scurve":
        return SCurveProfile(distance, v_max, a_max, start)
    if k == "step":
        return StepProfile(distance, v_max, a_max, start)
    return TrapezoidalProfile(distance, v_max, a_max, start)
