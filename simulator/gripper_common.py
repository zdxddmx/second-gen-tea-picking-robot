"""
Shared gripper definitions: phase enumeration, outcome record and pick constants.

What
----
:class:`GripperPhase`, :class:`GripOutcome` and the detachment constants
(``PICK_PULL_MM_S``, ``PICK_TWIST_DEG_S``, ``PICK_DURATION_S``,
``PICK_SPEED_LIMIT_MM_S``) that both gripper controllers use.

Why it is separate
------------------
Both :mod:`simulator.gripper` (force control) and
:mod:`simulator.gripper_position` (position-only comparison) need the same phase
vocabulary and the same outcome record, and the comparison arm also needs the
detachment constants so the two arms perform an identical pick.  Defining those in
either controller would make the two modules import each other, so they live here
where nothing depends on a controller.

What happens without it
-----------------------
The position-only controller could not share the flexible controller's phase
enumeration without a circular import, and the two arms of the Experiment 03
comparison would end up with divergent pick motions -- which would make any
difference in grip force uninterpretable.

Units
-----
Jaw velocity in mm/s (negative closes), force in N, times in s.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List

#: Detachment pull speed during PICK, mm/s.
PICK_PULL_MM_S: float = 0.25
#: Detachment twist rate during PICK, deg/s.
PICK_TWIST_DEG_S: float = 90.0
#: Nominal PICK duration, s.
PICK_DURATION_S: float = 0.25
#: Symmetric jaw-speed cap during detachment, mm/s.  Deliberately small: a large
#: correction while the stem is being worked free would throw the bud.
PICK_SPEED_LIMIT_MM_S: float = 3.0


class GripperPhase(enum.Enum):
    """Sub-states of the gripper controllers."""

    FAST_APPROACH = "FAST_APPROACH"
    SLOW_APPROACH = "SLOW_APPROACH"
    CONTACT_DETECT = "CONTACT_DETECT"
    FORCE_CONTROL = "FORCE_CONTROL"
    PICK = "PICK"
    RELEASE = "RELEASE"
    DONE = "DONE"

    def __str__(self) -> str:
        """Return the bare phase name."""
        return self.value


@dataclass
class GripOutcome:
    """Result of one gripper control tick.

    Attributes
    ----------
    phase:
        Current sub-state.
    v_cmd_mm_s:
        Jaw velocity command, mm/s (negative closes).
    achieved_force_n:
        Force actually measured at the jaw, N.
    events:
        Human-readable events produced this tick, e.g. ``"Contact detected"``.
    """

    phase: GripperPhase
    v_cmd_mm_s: float
    achieved_force_n: float
    events: List[str] = field(default_factory=list)
