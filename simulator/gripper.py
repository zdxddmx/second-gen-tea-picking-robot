"""
Flexible (force-controlled) gripper controller.

What
----
:class:FlexibleGripperController runs the force-controlled grasp: a fast
approach, a slow approach, contact detection, force settling, the detachment
motion and the release.  The force law itself is documented on
:meth:FlexibleGripperController._force_command, because the way it differs from
a plain PID is the substance of this module.

Why force control
-----------------
The stem force is a nonlinear stiffening spring, so a position-only gripper's
grip force varies by several newtons across a millimetre of stem-diameter
variation -- far more than the damage threshold allows.  The force loop turns that
stiffness into a non-issue: within its bandwidth the grip force depends on the
setpoint rather than on the stem.

What happens without it
-----------------------
The gripper would damage stems as soon as their diameter varied, and no
comparison against a position-only gripper would be meaningful because the
position-only arm would be failing for an unrelated reason.

Units
-----
Jaw gap in mm, jaw velocity in mm/s (negative closes), force in N.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from . import config as C
from .gripper_common import (
    PICK_DURATION_S,
    PICK_PULL_MM_S,
    PICK_SPEED_LIMIT_MM_S,
    PICK_TWIST_DEG_S,
    GripOutcome,
    GripperPhase,
)
from .gripper_position import FixedPositionGripperController  # noqa: F401 - re-exported
from .pid import PID, _finite, clamp


class FlexibleGripperController:
    """Force-controlled grasp with a fast/slow approach and safety limits.

    The sequence is ``FAST_APPROACH`` -> ``SLOW_APPROACH`` -> ``CONTACT_DETECT``
    -> ``FORCE_CONTROL`` -> ``PICK`` -> ``RELEASE``.  The switch from fast to slow
    is triggered by *distance to the stem surface*, which is known from the
    gripper geometry, and the switch to force control by the *measured force*,
    which is not -- that is why contact detection exists as its own phase rather
    than being inferred from position.

    Parameters
    ----------
    dt:
        Control period, seconds.
    f_target_n:
        Desired grip force, N.
    stem_gap_mm:
        Assumed stem surface position, mm, used for the approach switch.
    adaptive:
        When ``True``, the effective target tracks the observed contact force plus
        ``FORCE_ADAPT_MARGIN_N``, clamped into the valid range by ``observe``.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        f_target_n: float = C.FORCE_TARGET_N,
        stem_gap_mm: float = C.STEM_GAP_MM,
        adaptive: bool = False,
    ) -> None:
        """Create the force law and initialise the sub-state machine.

        Parameters
        ----------
        adaptive:
            Enables the adaptive force threshold.  It defaults to ``False``: the
            adaptive rule sets the target from the measured *contact* force
            (``observed + 0.60 N``, clamped to ``[0.80, 3.00]``), and because a tea
            stem first touches at only ~0.1 N that yields the 0.80 N floor -- a
            valid but very light grip which would make the nominal scenario a test
            of the floor rather than of the target.  Scenarios therefore run on the
            configured nominal target unless they explicitly ask for adaptation,
            and the clamp range is what bounds the grip in either case.
        """
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.f_target_n: float = _finite(f_target_n, C.FORCE_TARGET_N)
        self.stem_gap_mm: float = _finite(stem_gap_mm, C.STEM_GAP_MM)
        self.adaptive: bool = bool(adaptive)
        # The force loop is a *negative*-gain loop: closing the jaw is a negative
        # velocity but *increases* the force, so a positive force error must
        # produce a negative velocity command.  The inversion is applied by
        # negating the PID output (see :meth:`_force_command`) rather than by
        # inverting the feed-forward term: ``ff_term`` is *added* to
        # ``P + I + D``, so a negative ``ff_gain`` would only bias the result and
        # a large proportional term would still drive the jaw away from the stem.
        # Getting this wrong is not subtle in its consequence -- the jaw runs away
        # from the stem instead of into it.
        self.force_pid = PID(
            C.FORCE_PID_KP, C.FORCE_PID_KI, C.FORCE_PID_KD, self.dt,
            C.FORCE_PID_OUT_MIN, C.FORCE_PID_OUT_MAX, C.FORCE_PID_IMAX,
            C.FORCE_PID_TAU_D_S,
            derivative_on_measurement=True,
            ff_gain=0.0,
            name="force",
        )
        #: When ``False`` the jaw is held open; this is how the caller starts and
        #: stops the grasp without unmounting the controller.
        self.grip_active: bool = False
        self.reset()

    # -- lifecycle ----------------------------------------------------------
    def reset(self, f_target_n: Optional[float] = None) -> None:
        """Return to ``FAST_APPROACH`` and clear the force loop."""
        if f_target_n is not None:
            self.f_target_n = _finite(f_target_n, self.f_target_n)
        self.force_pid.reset()
        self.phase: GripperPhase = GripperPhase.FAST_APPROACH
        self.phase_s: float = 0.0
        self.settle_s: float = 0.0
        self.contact_s: Optional[float] = None
        self.grip_active = False
        self.f_observed_ema: float = 0.0
        self.f_target_adaptive: float = self.f_target_n
        self.f_achieved_n: float = 0.0
        self.peak_force_n: float = 0.0
        self.warning_raised: bool = False
        self.overload: bool = False
        self.released: bool = False
        #: Previous force error, used to detect the setpoint crossing at which the
        #: integrator is cleared.  ``None`` means "no previous sample yet".
        self._prev_force_err: Optional[float] = None
        #: Integral state of the force law (units: N*s), clamped in
        #: :meth:`_force_command`.
        self._force_integral: float = 0.0
        #: Jaw-measured deflection of the stem, mm; ``max(0, stem_gap - gap)``.
        #: Updated every tick from the gap reported by :meth:`step`, and used to
        #: normalise the force law's gain.
        self._stem_deflection_mm: float = 0.0
        #: ``True`` once the force loop has taken over from the approach; freezes
        #: the adaptive threshold estimate at its pre-contact value.
        self.force_control_started: bool = False
        #: ``(pid_output, proportional_term, integral_term)`` from the last force
        #: tick.  The PID output is recorded for comparison with the firmware gains
        #: even though the command is produced by the error-proportional law.
        self.last_terms: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.events: List[str] = []
        self.detach_pull_mm: float = 0.0
        self.detach_twist_deg: float = 0.0
        self.pick_s: float = 0.0
        self.backoff_mm: float = 0.0

    def effective_target(self, f_target_override: Optional[float] = None) -> float:
        """Return the grip-force setpoint actually used this tick.

        Parameters
        ----------
        f_target_override:
            When given (Scenario D sets 4.5 N), it is used verbatim and the
            adaptive term is bypassed -- an experiment that asks for a specific
            force must get exactly that force.
        """
        if f_target_override is not None:
            return _finite(f_target_override)
        if self.adaptive:
            return clamp(
                self.f_target_adaptive, C.FORCE_TARGET_MIN_N, C.FORCE_TARGET_MAX_N
            )
        return self.f_target_n

    def _switch(self, phase: GripperPhase) -> None:
        """Enter ``phase`` and reset its timer."""
        self.phase = phase
        self.phase_s = 0.0

    def _force_command(self, f_target: float, f_meas: float) -> float:
        """Run the grip force law and return a jaw velocity command.

        Why this is not simply the firmware PID
        ---------------------------------------
        The firmware's force gains are sized for the loop's *stroke* -- a 2 N error
        commanding a fast approach -- and cannot be used as-is against a stiff,
        gripped object.  Near the nominal grip the stem's local stiffness is
        ``k1 + 2*k2*d ~ 1.15 N/mm``, so the force changes at ``k_stem * v_jaw``: at
        the firmware's proportional gain of 14 mm/s per N a 0.5 N error commands
        7 mm/s, i.e. 8 N/s.  Measured on this plant that configuration overshoots
        the 2.2 N setpoint by 0.85 N and settles near 3.0 N, and the overshoot is
        *independent of the proportional gain* because what actually limits the
        loop is the integral term winding up while the jaw crosses the stem.

        The law used instead
        --------------------
        ``v_cmd = -G * error`` with ``G = FORCE_LAW_BANDWIDTH_PER_S /
        FORCE_LAW_STIFFNESS_N_PER_MM``.  Substituting the plant gives
        ``dF/dt = k_stem * G * error = FORCE_LAW_BANDWIDTH_PER_S * error``: a
        first-order force response whose rate is bounded by ``bandwidth * error``
        no matter how stiff the stem turns out to be.  That is the property the
        settle band needs, and it is the one fixed PID gains cannot provide because
        they do not know the object.  A small, clamped integral term removes the
        residual offset.

        The configured PID is still stepped every tick and its output kept in
        :attr:`last_terms`, so the firmware gains stay measurable and comparable in
        the logs even though the command no longer comes from them.

        Returns
        -------
        float
            Jaw velocity command in mm/s (negative closes), clamped to the force
            loop's output limits.
        """
        err = f_target - f_meas
        pid_out = self.force_pid.update(f_target, f_meas)

        # Local stem stiffness at the present deflection.  The force law wants a
        # chosen force *time constant*, and the deflection-to-force gain grows as
        # the stem is squeezed, so a fixed speed-per-error gain would over-drive the
        # loop at light contact.  The estimate is clamped so that a noisy channel,
        # or a stem that has already been released, cannot divide the command by
        # something tiny.
        k_local = C.STEM_K1_N_PER_MM + 2.0 * C.STEM_K2_N_PER_MM2 * max(
            0.0, self._stem_deflection_mm
        )
        k_local = clamp(k_local, C.STEM_K1_N_PER_MM, C.FORCE_LAW_K_EST_MAX_N_PER_MM)
        gain = C.FORCE_LAW_BANDWIDTH_PER_S / k_local

        # Integral term: removes the residual offset left by the proportional law.
        # Its contribution is clamped to FORCE_LAW_I_MAX_MM_S so it can never
        # dominate the command and re-create the wind-up overshoot.
        i_state_lim = C.FORCE_LAW_I_MAX_MM_S / max(C.FORCE_LAW_KI, C.EPS)
        self._force_integral = clamp(
            self._force_integral + err * self.dt, -i_state_lim, i_state_lim
        )
        # A zero-crossing of the error means the setpoint has just been passed:
        # clearing the integral stops stored energy from fighting the braking
        # phase, which is what caused the PID version's overshoot.
        if self._prev_force_err is not None and (err * self._prev_force_err) < 0.0:
            self._force_integral = 0.0
        self._prev_force_err = err
        i_term = C.FORCE_LAW_KI * self._force_integral

        command = -(gain * err + i_term)
        self.last_terms = (pid_out, gain * err, i_term)
        return clamp(command, -C.FORCE_PID_OUT_MAX, C.FORCE_PID_OUT_MAX)

    # -- runtime ------------------------------------------------------------
    def step(
        self,
        gap_mm: float,
        force_n: float,
        f_target_override: Optional[float] = None,
    ) -> GripOutcome:
        """Run one gripper control tick.

        Parameters
        ----------
        gap_mm:
            Measured jaw gap, mm.
        force_n:
            Measured (possibly filtered) contact force, N.
        f_target_override:
            Optional explicit force setpoint.

        Returns
        -------
        GripOutcome
            Phase, jaw velocity command, achieved force and any events.
        """
        self.phase_s += self.dt
        f_meas = _finite(force_n)
        self._stem_deflection_mm = max(0.0, self.stem_gap_mm - _finite(gap_mm))
        self.f_achieved_n = f_meas
        if f_meas > self.peak_force_n:
            self.peak_force_n = f_meas
        self.events = []

        # Adaptive threshold: an EMA of the observed force plus a fixed margin.
        #
        # It is only updated *before* closed-loop force control begins.  The
        # observed force is an EMA of the *measured* force, which once the loop is
        # closed is the same quantity the loop is driving -- so updating it during
        # force control makes the target chase itself upwards: 2.2 N raises the
        # estimate, which raises the target to 2.8 N, which raises the estimate
        # again, until the 3.0 N ceiling is reached.  Freezing it at the moment the
        # force loop takes over makes the adaptive value what it is meant to be: an
        # estimate of the stem's contact force from the pre-contact window, used to
        # pick a target that will not crush a soft bud.
        if self.adaptive and not self.force_control_started:
            self.f_observed_ema += C.FORCE_ADAPT_ALPHA * (f_meas - self.f_observed_ema)
            self.f_target_adaptive = clamp(
                self.f_observed_ema + C.FORCE_ADAPT_MARGIN_N,
                C.FORCE_TARGET_MIN_N,
                C.FORCE_TARGET_MAX_N,
            )
        f_target = self.effective_target(f_target_override)

        # Safety limits act in every phase, including the approach: an
        # obstruction must be able to abort a fast approach before it is crushed.
        if f_meas > C.FORCE_MAX_N:
            if not self.overload:
                self.overload = True
                self.events.append("Force overload: {0:.2f}N".format(f_meas))
                self._switch(GripperPhase.RELEASE)
            return GripOutcome(self.phase, C.GRIPPER_MAX_SPEED_MM_S, f_meas, self.events)
        if f_meas > C.FORCE_SOFT_LIMIT_N and not self.warning_raised:
            self.warning_raised = True
            self.events.append("Force soft limit: {0:.2f}N".format(f_meas))

        if not self.grip_active:
            # Idle: hold the jaw where it is.  Returning early (rather than
            # skipping the rest of the body) keeps the safety checks above live
            # even while the gripper is not grasping, and commanding zero keeps
            # the aperture available for the next approach.
            return GripOutcome(self.phase, 0.0, f_meas, self.events)

        if self.phase is GripperPhase.FAST_APPROACH:
            # Slow down when the jaw is within the configured distance of the
            # stem surface.  Closing below stem_gap_mm without this would arrive
            # at several newtons per millimetre of travel.
            if (gap_mm - self.stem_gap_mm) < C.APPROACH_SLOW_AT_MM:
                self._switch(GripperPhase.SLOW_APPROACH)
            else:
                return GripOutcome(
                    self.phase, -C.APPROACH_FAST_SPEED_MM_S, f_meas, self.events
                )

        if self.phase is GripperPhase.SLOW_APPROACH:
            if f_meas > C.FORCE_CONTACT_N:
                if self.contact_s is None:
                    self.contact_s = self.phase_s
                self.events.append("Contact detected")
                self.force_pid.reset()
                self.settle_s = 0.0
                self._switch(GripperPhase.CONTACT_DETECT)
            else:
                return GripOutcome(
                    self.phase, -C.APPROACH_SLOW_SPEED_MM_S, f_meas, self.events
                )

        if self.phase is GripperPhase.CONTACT_DETECT:
            # Hand over to closed-loop force control immediately; this phase
            # exists only to latch the contact event and reset the integrator, so
            # that the pre-contact history cannot bias the first force command.
            self.force_control_started = True
            self._switch(GripperPhase.FORCE_CONTROL)

        if self.phase is GripperPhase.FORCE_CONTROL:
            err = f_target - f_meas
            v_cmd = self._force_command(f_target, f_meas)
            if abs(err) < C.FORCE_BAND_N:
                self.settle_s += self.dt
            else:
                self.settle_s = 0.0
            if self.settle_s >= C.GRIPPER_SETTLE_S:
                self.events.append("Target force reached: {0:.2f}N".format(f_meas))
                self.pick_s = 0.0
                self._switch(GripperPhase.PICK)
            return GripOutcome(self.phase, v_cmd, f_meas, self.events)

        if self.phase is GripperPhase.PICK:
            # Detachment: pull up at PICK_PULL_MM_S while twisting at
            # PICK_TWIST_DEG_S, so the stem is worked free of the bud rather than
            # torn straight out.  The grip force is still held by the same force
            # loop, which is what stops the stem slipping out during the pull;
            # the command is additionally limited to a gentle 3 mm/s because a
            # sudden correction during detachment is how a picked bud is lost.
            self.pick_s += self.dt
            self.detach_pull_mm += PICK_PULL_MM_S * self.dt
            self.detach_twist_deg += PICK_TWIST_DEG_S * self.dt
            err = f_target - f_meas
            v_cmd = clamp(
                self._force_command(f_target, f_meas),
                -PICK_SPEED_LIMIT_MM_S,
                PICK_SPEED_LIMIT_MM_S,
            )
            # The detachment runs for its full duration.  It must *not* exit early
            # on "the force error is small": pulling a stem free is a position
            # action (a twist plus an upward pull), and the force error is expected
            # to grow as the stem resists.  Leaving early would put the machine
            # into RELEASE while the state machine was still waiting for the pick
            # to finish, and the jaw would open on a stem that was never detached.
            if self.pick_s >= PICK_DURATION_S:
                self.events.append(
                    "Detachment complete: pull {0:.2f}mm, twist {1:.1f}deg".format(
                        self.detach_pull_mm, self.detach_twist_deg
                    )
                )
                self._switch(GripperPhase.RELEASE)
            return GripOutcome(self.phase, v_cmd, f_meas, self.events)

        if self.phase is GripperPhase.RELEASE:
            if self.backoff_mm < C.GRIPPER_BACKOFF_MM:
                self.backoff_mm += C.GRIPPER_MAX_SPEED_MM_S * self.dt
                return GripOutcome(
                    self.phase, C.GRIPPER_MAX_SPEED_MM_S, f_meas, self.events
                )
            self.released = True
            self.events.append("Released")
            self._switch(GripperPhase.DONE)
            return GripOutcome(self.phase, 0.0, f_meas, self.events)

        return GripOutcome(self.phase, 0.0, f_meas, self.events)

    def snapshot(self) -> Dict[str, float]:
        """Return the gripper-controller state as a plain ``dict``."""
        return {
            "phase_index": float(
                [
                    GripperPhase.FAST_APPROACH,
                    GripperPhase.SLOW_APPROACH,
                    GripperPhase.CONTACT_DETECT,
                    GripperPhase.FORCE_CONTROL,
                    GripperPhase.PICK,
                    GripperPhase.RELEASE,
                    GripperPhase.DONE,
                ].index(self.phase)
            ),
            "f_target_n": self.effective_target(),
            "f_target_adaptive_n": self.f_target_adaptive,
            "f_observed_ema_n": self.f_observed_ema,
            "f_achieved_n": self.f_achieved_n,
            "peak_force_n": self.peak_force_n,
            "overload": 1.0 if self.overload else 0.0,
            "warning": 1.0 if self.warning_raised else 0.0,
        }
