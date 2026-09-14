"""
Pick-cycle sequencer: the jaw command and the robot-level handshakes.

What
----
:class:`GripSequencer` owns the part of the pick cycle that sits *between* the
gripper controller and the robot state machine: it turns the controller's phase
into a jaw velocity, reports the controller's events, and generates the events
that move the FSM from ``GRIP`` through ``PICK``, ``RELEASE`` and ``RETURN``.

Why it is its own class
-----------------------
The handshakes are neither gripper control nor joint control: they are the
robot's *choreography*.  Putting them in the simulation loop made that loop
unreadable and, more importantly, made them untestable in isolation -- and this
is exactly the logic that has to be right, because a missing handshake parks the
machine in one state forever.  Keeping it here means the FSM can be driven
through a whole cycle with no plant at all.

What happens without it
-----------------------
The FSM reaches ``GRIP`` and stops: the gripper controller's job is to hold the
force, not to decide that the grip has been established, so nothing would ever
advance the machine to detachment.
"""

from __future__ import annotations

from typing import Callable, List, Optional

from . import config as C
from .fsm import Event, RobotFSM, State
from .gripper import FlexibleGripperController, FixedPositionGripperController

#: Time the grip is held at the target force before detachment begins, s.  The
#: stem must be given a moment to settle into the jaws, otherwise the pick starts
#: while the stem is still creeping.
GRIP_HOLD_S: float = 0.10
#: Time from the start of the detachment to the release handshake, s.  Matches the
#: gripper controller's ``PICK_DURATION_S``.
GRIPPER_RELEASE_HANDOFF_S: float = 0.30
#: Time from the start of the release to the return handshake, s.
RELEASE_HANDOFF_S: float = 0.20
#: Time from the start of the return to the home handshake, s.
RETURN_HANDOFF_S: float = 0.30
#: Jaw speed used to re-open the grip at the end of a pick, mm/s.  Deliberately
#: below the fast-approach speed: flinging a detached stem open throws it out of
#: the basket.
RELEASE_OPEN_SPEED_MM_S: float = 30.0


class GripSequencer:
    """Drives the jaw and the pick-cycle handshakes for one simulation.

    Parameters
    ----------
    controller:
        The gripper controller, flexible or position-only -- both expose
        ``step(gap_mm, force_n[, target])``, ``released`` and ``grip_active``.
    fsm:
        The robot state machine.  The sequencer fires events on it; it never sets
        the state directly.
    on_trace:
        Callback invoked with each console-trace line, so the caller owns
        formatting and de-duplication.
    on_event:
        Callback invoked as ``on_event(kind, message)`` for the event log.
    on_fault:
        Callback invoked with a fault code to latch.
    """

    def __init__(
        self,
        controller: object,
        fsm: RobotFSM,
        on_trace: Callable[[str], None],
        on_event: Callable[[str, str], None],
        on_fault: Callable[[str], None],
        f_target_override_n: Optional[float] = None,
    ) -> None:
        """Store the collaborators and reset the handshake timers."""
        self.controller = controller
        self.fsm = fsm
        self.on_trace = on_trace
        self.on_event = on_event
        self.on_fault = on_fault
        self.f_target_override_n = f_target_override_n
        self.reset()

    def reset(self) -> None:
        """Clear every handshake timer and status flag."""
        self.grip_hold_t_s: float = 0.0
        self.pick_t_s: float = 0.0
        self.release_t_s: float = 0.0
        self.return_t_s: float = 0.0
        self.contact_t_s: float = 0.0
        self.force_reached_t_s: float = 0.0
        self.force_hold_end_s: float = 0.0
        self.force_controlled: bool = False
        self.cycle_completed: bool = False
        self.completed_cycle_time_s: float = 0.0

    # -- per-tick entry point ----------------------------------------------
    def step(self, t: float, gap_mm: float, force_meas_n: float) -> float:
        """Compute the jaw velocity command and advance the handshakes.

        Parameters
        ----------
        t:
            Simulation time, seconds.
        gap_mm:
            Jaw aperture, mm.
        force_meas_n:
            Measured (conditioned) grip force, N.

        Returns
        -------
        float
            Jaw velocity command in mm/s (negative closes).
        """
        state = self.fsm.state
        gripping = (State.APPROACH, State.CONTACT, State.GRIP)
        if state not in gripping:
            # The mission handshakes must run in *every* state, so they are driven
            # before the jaw is dealt with: PICK, RELEASE and RETURN are precisely
            # the states in which the jaw is no longer under force control, and
            # running the sequence only while gripping would leave the machine
            # parked in PICK forever.
            self._sequence(t)
            if state is State.PICK:
                # The grip is maintained by holding the jaw still: the stem's own
                # stiffness sustains the force, so commanding zero velocity keeps
                # the force loop latched at its last output without re-planning.
                return 0.0
            if state is State.RELEASE:
                if gap_mm < C.GRIPPER_MAX_MM - 0.2:
                    return RELEASE_OPEN_SPEED_MM_S
                if not self.controller.released:
                    self.controller.released = True
                if self.release_t_s <= 0.0:
                    self.release_t_s = t
                return 0.0
            # Idle / RETURN / fault: hold the jaw at its current aperture.  A
            # positive command here would drive the jaw into the 40 mm hard stop
            # and leave the next approach with no travel to measure, so the
            # command is explicitly zero rather than "fast approach".
            return 0.0

        self.controller.grip_active = True
        outcome = self.controller.step(gap_mm, force_meas_n, self.f_target_override_n)
        self._handle_events(t, list(outcome.events), force_meas_n)
        self._sequence(t)
        return float(outcome.v_cmd_mm_s)

    # -- event plumbing ----------------------------------------------------
    def _handle_events(self, t: float, events: List[str], force_meas_n: float) -> None:
        """Log the controller's events and fire the matching FSM transitions."""
        for message in events:
            self.on_event("grip", message)
            if message.startswith("Contact detected"):
                self.contact_t_s = t
                self.on_trace("Contact detected")
                self.fsm.fire(Event.CONTACT_DETECTED)
            elif message.startswith("Target force reached"):
                self.force_reached_t_s = t
                self.on_trace("Target force reached: {0:.2f}N".format(force_meas_n))
                self.force_controlled = True
                self.fsm.fire(Event.FORCE_REACHED)
            elif message.startswith("Force overload"):
                self.on_fault(C.ERR_FORCE_OVERLOAD)
                self.on_event("fault", C.ERR_FORCE_OVERLOAD)
                self.on_trace("Force overload protection triggered")
            elif message.startswith("Force soft limit"):
                self.on_fault(C.ERR_FORCE_SOFT_LIMIT)
                self.on_event("warn", C.ERR_FORCE_SOFT_LIMIT)

    def _sequence(self, t: float) -> None:
        """Advance the grip/pick/release/return handshakes.

        Each FSM state is entered by an event that the *plant* can actually
        witness, and each is left by an event the *phase machine* can witness:

        ==================  ==========================  ============================
        FSM state           entered on                  left on
        ==================  ==========================  ============================
        ``CONTACT``         ``CONTACT_DETECTED``        force settled
        ``GRIP``            ``FORCE_REACHED``           the grip has been held
        ``PICK``            ``PICK_DONE``               detachment finished
        ``RELEASE``         ``RELEASE_DONE``            jaws re-opened
        ``RETURN``          ``HOME_REACHED``            home reached
        ==================  ==========================  ============================

        The ``GRIP`` -> ``PICK`` event is generated here rather than by the gripper
        controller, because "the grip is established" is a decision made by the
        robot, not by the jaws: the controller's job is to hold the force, and it
        keeps holding it through the detachment.
        """
        state = self.fsm.state
        if state is State.GRIP:
            if self.grip_hold_t_s <= 0.0:
                self.grip_hold_t_s = t
            if t - self.grip_hold_t_s >= GRIP_HOLD_S:
                self.fsm.fire(Event.PICK_DONE)
                self.grip_hold_t_s = 0.0
        elif state is State.PICK:
            if self.pick_t_s <= 0.0:
                # First entry into PICK: log it once and start its timer.
                self.pick_t_s = t
                self.force_hold_end_s = t
                self.on_trace("Picking...")
            if t - self.pick_t_s >= GRIPPER_RELEASE_HANDOFF_S:
                self.fsm.fire(Event.RELEASE_DONE)
                self.on_trace("Tea picked successfully")
        elif state is State.RELEASE:
            if self.release_t_s > 0.0 and t - self.release_t_s >= RELEASE_HANDOFF_S:
                self.fsm.fire(Event.HOME_REACHED)
        elif state is State.RETURN:
            if self.return_t_s <= 0.0:
                self.return_t_s = t
                self.on_trace("Returning home")
            if t - self.return_t_s >= RETURN_HANDOFF_S:
                self.fsm.fire(Event.HOME_REACHED)
                if self.fsm.state is State.IDLE:
                    self.cycle_completed = True
                    self.completed_cycle_time_s = t
                    self.on_trace("Cycle completed")
