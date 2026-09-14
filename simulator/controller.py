"""
Controllers: the cascade joint controller and the flexible gripper controller.

What
----
:class:`CascadeController` runs the firmware's two nested loops -- a position PID
(``mrad`` error -> ``mrad/s`` velocity reference) driving a velocity PID
(``mrad/s`` error -> PWM per-mille) -- with velocity and torque feed-forward and a
selectable trajectory generator.

:class:`FlexibleGripperController` runs the force-controlled grasp: a fast
approach, a slow approach, contact detection, force-settling, the detachment
motion and the release.  :class:`FixedPositionGripperController` is the
position-only comparison used to show what force feedback buys.

:class:`SafetyMonitor` implements the firmware's stall and link-watchdog logic
side by side with the controllers, so that every scenario sees the same fault
semantics.

Why a cascade
-------------
A single loop from position to PWM has to cover two very different dynamics: a
heavy mechanical mode and a fast electrical one.  Splitting them lets the inner
loop be tuned for bandwidth (its disturbance rejection is what makes the
back-EMF and friction nonlinearities invisible from outside) and the outer loop
for accuracy, and it gives the velocity limit a natural place to be enforced --
the position loop's output *is* the speed command, so clipping it there is
exactly "do not exceed ``JOINT_MAX_SPEED_MRAD_S``".

Why force control for the gripper
---------------------------------
The stem force is a nonlinear stiffening spring, so a position-only gripper's
grip force varies by several newtons across a millimetre of stem-diameter
variation -- far more than the 4 N damage threshold allows.  The force loop turns
that stiffness into a non-issue: within its bandwidth the grip force depends on
the setpoint rather than on the stem.

Friction feed-forward
---------------------
Because the velocity loop's plant gain is high (``Kt`` is the *reflected* torque
constant, 2 N*m/A), the loop reaches a useful duty at very low error, where the
Coulomb term dominates.  :data:`config.VEL_PID_FF_GAIN` is therefore wired to a
*sign-of-command Coulomb feed-forward* rather than to the trajectory
acceleration: the duty needed to break stiction is computed from the measured
friction torque instead of being earned by the integrator, which removes most of
the stick-slip limit cycle a plain PI would show here.

What happens without it
-----------------------
Without the inner loop the position response is badly under-damped; without
back-calculation the outer loop winds up while the PWM is saturated; without the
force loop the gripper damages stems as soon as their diameter varies.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from . import config as C
from .gripper import (
    GripperPhase,
    GripOutcome,
    FlexibleGripperController,
    FixedPositionGripperController,
)
from .pid import PID, _finite, clamp
from .profiles import ProfileSample, make_profile
from .safety import SafetyMonitor

#: Detachment pull speed during PICK, mm/s.
PICK_PULL_MM_S: float = 0.25
#: Detachment twist rate during PICK, deg/s.
PICK_TWIST_DEG_S: float = 90.0
#: Nominal PICK duration, s.
PICK_DURATION_S: float = 0.25
#: Symmetric jaw-speed cap during detachment, mm/s.  Deliberately small: a large
#: correction while the stem is being worked free would throw the bud.
PICK_SPEED_LIMIT_MM_S: float = 3.0


def friction_feed_forward(omega_cmd_mrad_s: float) -> float:
    """Return the PWM feed-forward that cancels Coulomb friction.

    Parameters
    ----------
    omega_cmd_mrad_s:
        Commanded joint velocity, mrad/s.

    Returns
    -------
    float
        PWM duty in per-mille that must be added to break stiction in the
        commanded direction.  The value is derived from the plant constants
        rather than tuned: ``Tc/Kt`` is the current needed to hold against
        friction, and ``R/V_bus`` converts that current to duty.  It is scaled by
        ``tanh(omega/omega_eps)`` so it fades out smoothly at zero speed.
    """
    omega = _finite(omega_cmd_mrad_s)
    if abs(omega) < C.EPS:
        return 0.0
    i_break = C.MOTOR_TC_NM / max(C.MOTOR_KT_NM_PER_A, C.EPS)
    duty = i_break * C.MOTOR_R_OHM / C.MOTOR_VBUS_V * C.MAX_PWM_PERMILLE
    return duty * math.tanh(omega / C.MOTOR_OMEGA_EPS_MRAD_S)


class CascadeController:
    """Nested position/velocity controller with feed-forward.

    Parameters
    ----------
    dt:
        Control period, seconds.
    profile_kind:
        ``"trapezoid"``, ``"scurve"`` or ``"step"``; see
        :func:`simulator.profiles.make_profile`.
    v_max_mrad_s, a_max_mrad_s2:
        Motion limits handed to the trajectory generator.
    use_velocity_ff:
        Add the trajectory velocity to the position loop's output.
    use_torque_ff:
        Add the friction feed-forward to the velocity loop's output.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        profile_kind: str = "trapezoid",
        v_max_mrad_s: float = C.JOINT_MAX_SPEED_MRAD_S,
        a_max_mrad_s2: float = C.JOINT_MAX_ACCEL_MRAD_S2,
        use_velocity_ff: bool = True,
        use_torque_ff: bool = True,
    ) -> None:
        """Create the two PIDs and the trajectory generator."""
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.profile_kind: str = str(profile_kind)
        self.v_max_mrad_s: float = _finite(v_max_mrad_s, C.JOINT_MAX_SPEED_MRAD_S)
        self.a_max_mrad_s2: float = _finite(a_max_mrad_s2, C.JOINT_MAX_ACCEL_MRAD_S2)
        self.use_velocity_ff: bool = bool(use_velocity_ff)
        self.use_torque_ff: bool = bool(use_torque_ff)
        self.pos_pid = PID(
            C.POS_PID_KP, C.POS_PID_KI, C.POS_PID_KD, self.dt,
            C.POS_PID_OUT_MIN, C.POS_PID_OUT_MAX, C.POS_PID_IMAX,
            C.POS_PID_TAU_D_S,
            derivative_on_measurement=True,
            ff_gain=C.POS_PID_FF_GAIN if use_velocity_ff else 0.0,
            name="pos",
        )
        self.vel_pid = PID(
            C.VEL_PID_KP, C.VEL_PID_KI, C.VEL_PID_KD, self.dt,
            C.VEL_PID_OUT_MIN, C.VEL_PID_OUT_MAX, C.VEL_PID_IMAX,
            C.VEL_PID_TAU_D_S,
            derivative_on_measurement=True,
            ff_gain=1.0 if use_torque_ff else 0.0,
            name="vel",
        )
        self.profile = make_profile(self.profile_kind, 0.0, self.v_max_mrad_s, self.a_max_mrad_s2)
        self.t_s: float = 0.0
        self.paused: bool = False
        self.ref_pos_mrad: float = 0.0
        self.ref_vel_mrad_s: float = 0.0
        self.ref_acc_mrad_s2: float = 0.0
        self.pwm_cmd: float = 0.0
        self.saturated: bool = False
        self.goal_reached: bool = False

    # -- configuration ------------------------------------------------------
    def pause(self) -> None:
        """Freeze the trajectory clock and hold the present reference.

        The loop keeps running -- so the joint holds position under closed-loop
        control -- but the profile stops advancing.  This is what the pick cycle
        needs while the gripper works: the arm must stay put without being
        re-planned, because a re-plan every tick would reset the profile and
        produce a staircase reference.
        """
        self.paused = True

    def resume(self) -> None:
        """Release the trajectory clock, continuing from where it stopped."""
        self.paused = False

    def set_target(self, target_mrad: float, current_mrad: float) -> None:
        """Plan a move from the present position to ``target_mrad``.

        Parameters
        ----------
        target_mrad:
            Desired joint angle, mrad.  Clamped into the software limits.
        current_mrad:
            Present joint angle, mrad; the profile starts here so there is no
            position jump on re-planning mid-motion.
        """
        tgt = clamp(_finite(target_mrad), C.JOINT_MIN_MRAD, C.JOINT_MAX_MRAD)
        start = _finite(current_mrad)
        self.profile = make_profile(
            self.profile_kind, tgt - start, self.v_max_mrad_s, self.a_max_mrad_s2, start
        )
        self.t_s = 0.0
        self.ref_pos_mrad = start
        self.ref_vel_mrad_s = 0.0
        self.ref_acc_mrad_s2 = 0.0
        self.goal_reached = False

    def reset(self, position_mrad: float = C.JOINT_HOME_MRAD) -> None:
        """Clear the PIDs and hold the present position as the reference."""
        pos = _finite(position_mrad)
        self.pos_pid.reset()
        self.vel_pid.reset()
        self.t_s = 0.0
        self.ref_pos_mrad = pos
        self.ref_vel_mrad_s = 0.0
        self.ref_acc_mrad_s2 = 0.0
        self.pwm_cmd = 0.0
        self.saturated = False
        self.goal_reached = False

    # -- runtime ------------------------------------------------------------
    def tick(self, meas_pos_mrad: float, meas_vel_mrad_s: float) -> float:
        """Run one cascade iteration.

        Parameters
        ----------
        meas_pos_mrad:
            Measured (or encoder-reported) joint angle, mrad.
        meas_vel_mrad_s:
            Measured joint velocity, mrad/s.

        Returns
        -------
        float
            PWM command in per-mille.  The caller is responsible for applying it
            to the plant, because the same value also feeds the safety monitor.
        """
        self.t_s += self.dt
        sample = self.profile.sample(self.t_s)
        self.ref_pos_mrad = sample.pos if not self.paused else self.ref_pos_mrad
        self.ref_vel_mrad_s = sample.vel if not self.paused else 0.0
        self.ref_acc_mrad_s2 = sample.acc if not self.paused else 0.0

        # Outer loop: position error -> velocity reference.
        vel_cmd = self.pos_pid.update(self.ref_pos_mrad, _finite(meas_pos_mrad), self.ref_vel_mrad_s)
        vel_cmd = clamp(vel_cmd, -self.v_max_mrad_s, self.v_max_mrad_s)

        # Inner loop: velocity error -> duty.  The friction feed-forward is the
        # only term that can move the joint from rest, because at zero speed the
        # proportional term of a well-scaled loop is still tiny.
        ff_duty = friction_feed_forward(vel_cmd) if self.use_torque_ff else 0.0
        pwm = self.vel_pid.update(vel_cmd, _finite(meas_vel_mrad_s), ff_duty)
        self.pwm_cmd = clamp(pwm, -C.MAX_PWM_PERMILLE, C.MAX_PWM_PERMILLE)
        self.saturated = self.vel_pid.saturated or self.pos_pid.saturated
        self.goal_reached = sample.done and abs(self.ref_pos_mrad - meas_pos_mrad) < 50.0
        return self.pwm_cmd

    def reference(self) -> ProfileSample:
        """Return the trajectory reference at the current time."""
        return self.profile.sample(self.t_s)

    def snapshot(self) -> Dict[str, float]:
        """Return the controller state as a plain ``dict`` of floats.

        The individual PID terms are included because they are what identifies
        *which* term is responsible for a poor response: a large ``pos_i`` with a
        saturated PWM means wind-up, whereas a large ``vel_p`` with a small
        ``pos_i`` means the inner loop gain is too high.
        """
        return {
            "ref_pos_mrad": self.ref_pos_mrad,
            "ref_vel_mrad_s": self.ref_vel_mrad_s,
            "ref_acc_mrad_s2": self.ref_acc_mrad_s2,
            "pwm_cmd": self.pwm_cmd,
            "pos_p": self.pos_pid.last_p,
            "pos_i": self.pos_pid.last_i,
            "pos_d": self.pos_pid.last_d,
            "vel_p": self.vel_pid.last_p,
            "vel_i": self.vel_pid.last_i,
            "vel_d": self.vel_pid.last_d,
            "saturated": 1.0 if self.saturated else 0.0,
            "paused": 1.0 if self.paused else 0.0,
        }

