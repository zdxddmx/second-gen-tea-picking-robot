"""
Table-driven finite state machine for the picking cycle.

What
----
:class:`RobotFSM` owns a single dispatch table keyed by ``(State, Event)`` whose
values are :class:`Transition` records carrying the target state and an optional
action callback.  The machine exposes :meth:`RobotFSM.fire`, which performs one
transition (or reports that the event was ignored) and appends a row to a trace
that can be plotted as "State vs Time".

Why a table
-----------
A hand-written ``if/elif`` ladder over eleven states makes illegal transitions
easy to introduce and impossible to review: reordering two branches changes
behaviour silently.  With a table the *interaction matrix* is a single literal
that can be read top to bottom, and adding a state means adding rows rather than
searching for the right place to insert a branch.  It also makes the FSM
directly comparable to a statechart, which is how the firmware's behaviour is
specified.

What happens without it
-----------------------
Branch-order bugs would let, for example, ``CONTACT_DETECTED`` be handled while
in ``RELEASE``, driving the jaws shut on an already-detached stem.

Trace
-----
Every accepted transition appends ``(t_s, from_state, event, to_state)`` to
:attr:`RobotFSM.trace`.  This is the data source for
:func:`simulator.plotting.plot_state_vs_time`.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import config as C


class State(enum.Enum):
    """Discrete operating states of the picking robot."""

    INIT = "INIT"
    SELF_CHECK = "SELF_CHECK"
    IDLE = "IDLE"
    TARGET_RECEIVED = "TARGET_RECEIVED"
    APPROACH = "APPROACH"
    CONTACT = "CONTACT"
    GRIP = "GRIP"
    PICK = "PICK"
    RELEASE = "RELEASE"
    RETURN = "RETURN"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"

    def __str__(self) -> str:
        """Return the bare state name, e.g. ``"APPROACH"``."""
        return self.value


class Event(enum.Enum):
    """Events that can drive a state transition."""

    SELF_CHECK_OK = "SELF_CHECK_OK"
    SELF_CHECK_FAIL = "SELF_CHECK_FAIL"
    TARGET_RECEIVED = "TARGET_RECEIVED"
    START = "START"
    CONTACT_DETECTED = "CONTACT_DETECTED"
    FORCE_REACHED = "FORCE_REACHED"
    PICK_DONE = "PICK_DONE"
    RELEASE_DONE = "RELEASE_DONE"
    HOME_REACHED = "HOME_REACHED"
    STOP = "STOP"
    FAULT = "FAULT"
    E_STOP = "E_STOP"
    RESET = "RESET"

    def __str__(self) -> str:
        """Return the bare event name, e.g. ``"CONTACT_DETECTED"``."""
        return self.value


@dataclass
class Transition:
    """One entry of the FSM dispatch table.

    Attributes
    ----------
    target:
        State entered when the transition is taken.  ``None`` means "stay in the
        current state" (used for self-looping actions such as a RESET that only
        clears fault bookkeeping).
    action:
        Optional ``callable(fsm)`` invoked *before* the state changes, so it can
        still read the old state from ``fsm.state``.
    description:
        Short human-readable note; surfaced in the printed trace for review.
    """

    target: Optional[State]
    action: Optional[Callable[["RobotFSM"], None]] = None
    description: str = ""


@dataclass
class TraceEntry:
    """A single recorded transition.

    Attributes
    ----------
    t_s:
        Simulation time of the transition, seconds.
    from_state, to_state:
        States before and after.
    event:
        Event that caused it.
    """

    t_s: float
    from_state: State
    to_state: State
    event: Event


def _act_log_fault(fsm: "RobotFSM") -> None:
    """Default FAULT action: latch the pending fault code into the error list."""
    code = fsm.pending_fault or C.ERR_NONE
    if code and code not in fsm.faults:
        fsm.faults.append(code)
    fsm.messages.append("Fault latched: {0}".format(code))


def _act_clear_faults(fsm: "RobotFSM") -> None:
    """RESET action: clear the latched faults and the pending code."""
    fsm.faults = []
    fsm.pending_fault = None
    fsm.messages.append("Faults cleared by RESET")


#: The dispatch table.  Keyed by ``(state, event)``.
TRANSITION_TABLE: Dict[Tuple[State, Event], Transition] = {
    # -- power-up ---------------------------------------------------------
    (State.INIT, Event.START): Transition(State.SELF_CHECK, None, "begin self check"),
    (State.SELF_CHECK, Event.SELF_CHECK_OK): Transition(State.IDLE, None, "self check passed"),
    (State.SELF_CHECK, Event.SELF_CHECK_FAIL): Transition(
        State.ERROR,
        lambda fsm: fsm.faults.append(C.ERR_SELF_CHECK) or None,
        "self check failed",
    ),
    # -- mission ----------------------------------------------------------
    (State.IDLE, Event.TARGET_RECEIVED): Transition(
        State.TARGET_RECEIVED, None, "vision target accepted"
    ),
    (State.TARGET_RECEIVED, Event.START): Transition(State.APPROACH, None, "start approach"),
    (State.APPROACH, Event.CONTACT_DETECTED): Transition(
        State.CONTACT, None, "stem contact detected"
    ),
    (State.CONTACT, Event.FORCE_REACHED): Transition(State.GRIP, None, "grip force reached"),
    (State.GRIP, Event.PICK_DONE): Transition(State.PICK, None, "detachment complete"),
    (State.PICK, Event.RELEASE_DONE): Transition(State.RELEASE, None, "stem released"),
    (State.RELEASE, Event.HOME_REACHED): Transition(State.RETURN, None, "returning to home"),
    (State.RETURN, Event.HOME_REACHED): Transition(State.IDLE, None, "home reached"),
    # -- controlled stop --------------------------------------------------
    (State.APPROACH, Event.STOP): Transition(State.IDLE, None, "operator stop"),
    (State.TARGET_RECEIVED, Event.STOP): Transition(State.IDLE, None, "operator stop"),
    # -- faults -----------------------------------------------------------
    (State.IDLE, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault in IDLE"),
    (State.TARGET_RECEIVED, Event.FAULT): Transition(
        State.ERROR, _act_log_fault, "fault before approach"
    ),
    (State.APPROACH, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault while approaching"),
    (State.CONTACT, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault in contact"),
    (State.GRIP, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault while gripping"),
    (State.PICK, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault while picking"),
    (State.RELEASE, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault while releasing"),
    (State.RETURN, Event.FAULT): Transition(State.ERROR, _act_log_fault, "fault while returning"),
    (State.ERROR, Event.FAULT): Transition(None, _act_log_fault, "additional fault latched"),
    # -- emergency stop: available from every operational state -----------
    (State.IDLE, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.TARGET_RECEIVED, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.APPROACH, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.CONTACT, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.GRIP, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.PICK, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.RELEASE, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.RETURN, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop"),
    (State.INIT, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop during boot"),
    (State.SELF_CHECK, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop during boot"),
    (State.ERROR, Event.E_STOP): Transition(State.EMERGENCY_STOP, None, "E-stop while faulted"),
    (State.EMERGENCY_STOP, Event.E_STOP): Transition(None, None, "E-stop re-asserted"),
    # -- recovery ---------------------------------------------------------
    (State.ERROR, Event.RESET): Transition(State.IDLE, _act_clear_faults, "reset from ERROR"),
    (State.EMERGENCY_STOP, Event.RESET): Transition(
        State.IDLE, _act_clear_faults, "reset from E-stop"
    ),
}


@dataclass
class FSMSnapshot:
    """Immutable view of the FSM state, used for telemetry.

    Attributes
    ----------
    state:
        Current state.
    state_index:
        Integer index of the state in :data:`STATE_ORDER`, for plotting.
    fault_code:
        Most recent latched fault code, or ``None``.
    """

    state: State
    state_index: int
    fault_code: Optional[str]


#: Stable ordering used for the y axis of the state plot.
STATE_ORDER: Tuple[State, ...] = (
    State.INIT,
    State.SELF_CHECK,
    State.IDLE,
    State.TARGET_RECEIVED,
    State.APPROACH,
    State.CONTACT,
    State.GRIP,
    State.PICK,
    State.RELEASE,
    State.RETURN,
    State.ERROR,
    State.EMERGENCY_STOP,
)

#: State index lookup, kept in sync with :data:`STATE_ORDER`.
STATE_INDEX: Dict[State, int] = {s: i for i, s in enumerate(STATE_ORDER)}


class RobotFSM:
    """Table-driven state machine with a transition trace.

    Parameters
    ----------
    initial:
        State the machine starts in.  ``INIT`` is correct for a real boot; a
        scenario that begins mid-mission may pass another state.
    autostart:
        When ``True`` the first :meth:`step` fires ``START`` so that a caller
        does not have to model the power-up event explicitly.
    """

    def __init__(
        self,
        initial: State = State.INIT,
        autostart: bool = True,
    ) -> None:
        """Create the machine and reset its bookkeeping."""
        self.initial: State = initial
        self.autostart: bool = bool(autostart)
        self.table: Dict[Tuple[State, Event], Transition] = dict(TRANSITION_TABLE)
        self.trace: List[TraceEntry] = []
        self.faults: List[str] = []
        self.messages: List[str] = []
        self.pending_fault: Optional[str] = None
        self.t_s: float = 0.0
        self.last_change_s: float = 0.0
        self.state: State = initial
        self.recent_events: List[Event] = []
        self._started: bool = False

    # -- query --------------------------------------------------------------
    def available_events(self, state: Optional[State] = None) -> List[Event]:
        """Return the events accepted in ``state`` (default: current state)."""
        st = self.state if state is None else state
        return [ev for (s, ev) in self.table.keys() if s is st]

    def is_terminal_fault(self) -> bool:
        """Return ``True`` when the machine is in a stopped-by-fault state."""
        return self.state in (State.ERROR, State.EMERGENCY_STOP)

    def time_in_state_s(self) -> float:
        """Return how long the machine has been in its current state, seconds."""
        return max(0.0, self.t_s - self.last_change_s)

    def state_index(self) -> int:
        """Return the integer plotting index of the current state."""
        return STATE_INDEX.get(self.state, len(STATE_ORDER) - 1)

    def last_fault(self) -> Optional[str]:
        """Return the most recently latched fault code, if any."""
        return self.faults[-1] if self.faults else None

    def snapshot(self) -> FSMSnapshot:
        """Return an immutable snapshot for logging."""
        return FSMSnapshot(self.state, self.state_index(), self.last_fault())

    # -- mutation -----------------------------------------------------------
    def fire(self, event: Event) -> bool:
        """Attempt one transition on ``event``.

        Parameters
        ----------
        event:
            Event to deliver.

        Returns
        -------
        bool
            ``True`` when a transition was taken, ``False`` when the event is
            not accepted in the current state.  An ignored event is *not* an
            error: it is how the machine ignores, for example, a duplicate
            contact report.
        """
        key = (self.state, event)
        trans = self.table.get(key)
        if trans is None:
            self.recent_events.append(event)
            return False
        if trans.action is not None:
            trans.action(self)
        from_state = self.state
        if trans.target is not None:
            self.state = trans.target
            self.last_change_s = self.t_s
        self.trace.append(TraceEntry(self.t_s, from_state, self.state, event))
        self.recent_events.append(event)
        if trans.description:
            self.messages.append(
                "{0:.3f}s {1} -> {2} ({3})".format(
                    self.t_s, from_state.value, self.state.value, trans.description
                )
            )
        return True

    def fault(self, code: str) -> bool:
        """Report a fault code and transition to ``ERROR``.

        Parameters
        ----------
        code:
            Fault identifier, e.g. :data:`config.ERR_MOTOR_STALL`.

        Returns
        -------
        bool
            ``True`` when the machine moved to ``ERROR``.
        """
        self.pending_fault = code
        return self.fire(Event.FAULT)

    def e_stop(self) -> bool:
        """Assert the emergency stop."""
        return self.fire(Event.E_STOP)

    def reset(self, t_s: Optional[float] = None) -> bool:
        """Clear faults and return to ``IDLE``.

        Parameters
        ----------
        t_s:
            Optional new time base value; the trace is always cleared because it
            describes a completed episode.
        """
        if t_s is not None:
            self.t_s = float(t_s)
        ok = self.fire(Event.RESET)
        if ok:
            self.trace = [TraceEntry(self.t_s, self.state, self.state, Event.RESET)]
            self.last_change_s = self.t_s
        return ok

    def step(self, t_s: float) -> FSMSnapshot:
        """Advance the machine's time base by one tick.

        Only the time base moves here.  Transitions are always caused by an
        explicit event so that the FSM is a pure function of
        ``(state, event)`` and can be exercised without a plant.

        Parameters
        ----------
        t_s:
            Current simulation time, seconds.

        Returns
        -------
        FSMSnapshot
            The state after the tick.
        """
        self.t_s = float(t_s)
        if self.autostart and not self._started:
            self._started = True
            self.fire(Event.START)
        return self.snapshot()

    # -- reporting ----------------------------------------------------------
    def trace_rows(self) -> List[Tuple[float, str, str, str]]:
        """Return the trace as ``(t_s, from, event, to)`` string tuples."""
        return [
            (e.t_s, e.from_state.value, e.event.value, e.to_state.value) for e in self.trace
        ]

    def format_trace(self) -> str:
        """Return the trace as a multi-line, human-readable table."""
        lines = ["   time[s]  from                event              to"]
        for t_s, src, ev, dst in self.trace_rows():
            lines.append("{0:9.3f}  {1:<18}  {2:<16}  {3}".format(t_s, src, ev, dst))
        return "\n".join(lines)
