"""
Position-only gripper controller: the comparison arm for the force loop.

What
----
:class:`FixedPositionGripperController` closes the jaw to a fixed gap and does not
use force feedback at all.  It is the baseline the flexible controller is measured
against.

Why it exists
-------------
Experiment 03 needs a control group.  "Force feedback is better" is an assertion
until the same stem, the same approach and the same jaw dynamics are run without
it -- and then the difference in grip force becomes a measurement.  Keeping the
position-only controller as a separate class (rather than as a flag on the
flexible one) is what makes that comparison honest: there is no force term present
at all, so it cannot accidentally be enabled.

What happens without it
-----------------------
There would be no baseline, and the benefit of force control would remain a claim
rather than a figure.

Units
-----
Jaw gap in mm, jaw velocity in mm/s (negative closes), force in N.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from . import config as C
from .gripper import (
    PICK_DURATION_S,
    PICK_PULL_MM_S,
    PICK_TWIST_DEG_S,
    GripOutcome,
    GripperPhase,
)
from .pid import _finite


class FixedPositionGripperController:
    """Position-only gripper: close to a fixed gap, no force feedback.

    This is the control group for Experiment 03.  It exists to make the value of
    force feedback measurable rather than asserted: given the same stem, the same
    approach and the same jaw dynamics, the only difference is that the grip force
    here is whatever the stem stiffness happens to produce at the commanded gap.

    Parameters
    ----------
    dt:
        Control period, seconds.
    gap_target_mm:
        Gap to close to, mm.  A real position-only gripper would size this from
        the nominal stem diameter, which is exactly the source of the error.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        gap_target_mm: float = C.STEM_GAP_MM - 2.0,
    ) -> None:
        """Store the gap setpoint and start closing."""
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.gap_target_mm: float = _finite(gap_target_mm, C.STEM_GAP_MM - 2.0)
        self.reset()

    def reset(self, gap_target_mm: Optional[float] = None) -> None:
        """Reset to a fresh closing motion."""
        if gap_target_mm is not None:
            self.gap_target_mm = _finite(gap_target_mm, self.gap_target_mm)
        self.phase: GripperPhase = GripperPhase.SLOW_APPROACH
        self.phase_s: float = 0.0
        self.f_achieved_n: float = 0.0
        self.peak_force_n: float = 0.0
        self.overload: bool = False
        self.released: bool = False
        self.backoff_mm: float = 0.0
        self.reached: bool = False
        self.events: List[str] = []
        self.detach_pull_mm: float = 0.0
        self.detach_twist_deg: float = 0.0
        self.pick_s: float = 0.0

    def step(self, gap_mm: float, force_n: float) -> GripOutcome:
        """Run one position-only control tick.

        Parameters
        ----------
        gap_mm:
            Measured jaw gap, mm.
        force_n:
            Measured contact force, N.  It is *measured* but not *controlled* --
            reading it is what lets the comparison report the resulting force.

        Returns
        -------
        GripOutcome
            Same structure as the flexible controller's, so the two are
            interchangeable in a scenario.
        """
        self.phase_s += self.dt
        f_meas = _finite(force_n)
        self.f_achieved_n = f_meas
        if f_meas > self.peak_force_n:
            self.peak_force_n = f_meas
        self.events = []

        if self.phase is GripperPhase.RELEASE:
            if self.backoff_mm < C.GRIPPER_BACKOFF_MM:
                self.backoff_mm += C.GRIPPER_MAX_SPEED_MM_S * self.dt
                return GripOutcome(self.phase, C.GRIPPER_MAX_SPEED_MM_S, f_meas, self.events)
            self.released = True
            self.phase = GripperPhase.DONE
            self.events.append("Released")
            return GripOutcome(self.phase, 0.0, f_meas, self.events)

        if self.phase is GripperPhase.PICK:
            self.pick_s += self.dt
            self.detach_pull_mm += PICK_PULL_MM_S * self.dt
            self.detach_twist_deg += PICK_TWIST_DEG_S * self.dt
            if self.pick_s >= PICK_DURATION_S:
                self.phase = GripperPhase.RELEASE
                self.events.append(
                    "Detachment complete: pull {0:.2f}mm, twist {1:.1f}deg".format(
                        self.detach_pull_mm, self.detach_twist_deg
                    )
                )
            return GripOutcome(self.phase, -C.APPROACH_SLOW_SPEED_MM_S, f_meas, self.events)

        # Pure position control: drive at a fixed speed until the gap setpoint.
        if gap_mm > self.gap_target_mm + 0.05:
            return GripOutcome(self.phase, -C.APPROACH_SLOW_SPEED_MM_S, f_meas, self.events)
        if not self.reached:
            self.reached = True
            self.events.append("Gap setpoint reached: {0:.2f}mm".format(gap_mm))
            self.pick_s = 0.0
            self.phase = GripperPhase.PICK
        return GripOutcome(self.phase, 0.0, f_meas, self.events)

    def snapshot(self) -> Dict[str, float]:
        """Return the controller state as a plain ``dict`` of floats."""
        return {
            "gap_target_mm": self.gap_target_mm,
            "f_achieved_n": self.f_achieved_n,
            "peak_force_n": self.peak_force_n,
            "reached": 1.0 if self.reached else 0.0,
        }
