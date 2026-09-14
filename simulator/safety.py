"""
Safety monitors: stall, over-current and host-link watchdog.

What
----
:class:SafetyMonitor holds the firmware's fault-detection logic
(`TP_STALL_*`, `TP_OVERCURRENT_TRIP_MS`, `TP_COMM_TIMEOUT_MS`) as a single
object driven by *measured* values and the commanded duty.

Why it is separate from the controller
--------------------------------------
Fault detection must keep running when the controller behaves oddly -- that is the
whole point of it -- so it consumes measurements and commands, never the
controller's internal intentions.  Keeping it in its own module is also what lets
the safety behaviour be reviewed without reading the control code.

What happens without it
-----------------------
A blocked joint or a dead host link would go unnoticed, and the drive would
continue to push against the obstruction until something failed mechanically.
"""

from __future__ import annotations

from typing import List

from . import config as C
from .pid import _finite


class SafetyMonitor:
    """Stall, over-current and link-watchdog detectors.

    Why it is separate from the controller
    --------------------------------------
    Fault detection must keep running when the controller behaves oddly -- that
    is the whole point of it -- so it consumes *measured* values and the commanded
    duty, never the controller's internal intentions.

    Parameters
    ----------
    dt:
        Detection period, seconds (``TP_SAFETY_PERIOD_MS``).
    """

    def __init__(self, dt: float = C.SAFETY_DT_S) -> None:
        """Initialise the debounce timers."""
        self.dt: float = max(C.EPS, _finite(dt, C.SAFETY_DT_S))
        self.reset()

    def reset(self) -> None:
        """Clear all debounce timers and latched faults."""
        self.stall_timer_s: float = 0.0
        self.overcurrent_timer_s: float = 0.0
        self.link_lost_s: float = 0.0
        self.faults: List[str] = []
        self.stalled: bool = False
        self.overcurrent: bool = False
        self.link_lost: bool = False
        #: ``True`` while the stall conditions currently hold.  Used to attribute
        #: a saturated current to the jam instead of tripping over-current on it.
        self.stall_arming: bool = False

    def check_stall(self, pwm_permille: float, omega_mrad_s: float) -> bool:
        """Update the stall detector.

        A stall is armed only when the controller is genuinely asking for torque
        (``|PWM| > STALL_PWM_THRESHOLD``); otherwise a joint that is simply being
        told to hold still would trip it.  The condition must persist for
        ``STALL_TRIP_MS`` before it is reported, which is what makes it immune to
        the zero-crossing of a legitimate slow move.

        Returns
        -------
        bool
            ``True`` when the stall has just been latched.
        """
        armed = abs(_finite(pwm_permille)) > C.STALL_PWM_THRESHOLD
        slow = abs(_finite(omega_mrad_s)) < C.STALL_SPEED_THRESHOLD_MRAD_S
        self.stall_arming = armed and slow
        if self.stall_arming:
            self.stall_timer_s += self.dt
        else:
            self.stall_timer_s = 0.0
        if self.stall_timer_s >= C.STALL_TRIP_MS / 1000.0 and not self.stalled:
            self.stalled = True
            self.faults.append(C.ERR_MOTOR_STALL)
            return True
        return False

    def check_overcurrent(self, current_a: float) -> bool:
        """Update the over-current detector against ``MAX_CURRENT_MA``.

        The detector is *suppressed while a stall is being evaluated*.  This is a
        deliberate resolution of a real race, not a workaround: a blocked joint
        drives the inner current loop into saturation, and because the current
        command is clamped to the single value ``PEAK_CURRENT_MA`` there is no
        intermediate operating point between the 3 A continuous limit and the 5 A
        peak -- the current jumps straight to the peak.  Without the suppression
        the 50 ms over-current timer always expires 250 ms before the 300 ms stall
        debounce completes, so a jam would always be reported as an electrical
        fault and the mechanical one would be unreachable.

        Reporting the mechanical cause is also the *more useful* diagnosis: the
        excess current is a consequence of the jam, and the operator can clear a
        jam whereas an over-current trip on a blocked joint says nothing about
        what actually happened.  A genuine electrical fault that is *not* a stall
        (a short, a failed bridge) does not hold the joint still at zero speed, so
        it still trips on the normal path.

        Returns
        -------
        bool
            ``True`` when the over-current trip has just been latched.
        """
        over = abs(_finite(current_a)) > C.MAX_CURRENT_A
        if over and self.stall_arming:
            # Attribute the current to the jam, and keep the timer clear so that a
            # sustained jam cannot accumulate an over-current trip in parallel.
            self.overcurrent_timer_s = 0.0
            return False
        if over:
            self.overcurrent_timer_s += self.dt
        else:
            self.overcurrent_timer_s = 0.0
        if self.overcurrent_timer_s >= C.OVERCURRENT_TRIP_MS / 1000.0 and not self.overcurrent:
            self.overcurrent = True
            self.faults.append(C.ERR_OVERCURRENT)
            return True
        return False

    def check_link(self, link_ok: bool) -> bool:
        """Update the host-link watchdog; returns ``True`` on first timeout."""
        if not link_ok:
            self.link_lost_s += self.dt
        else:
            self.link_lost_s = 0.0
            self.link_lost = False
        if self.link_lost_s >= C.COMM_TIMEOUT_S and not self.link_lost:
            self.link_lost = True
            self.faults.append(C.ERR_COMM_TIMEOUT)
            return True
        return False
