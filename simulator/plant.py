"""
Plant models: motor-joint dynamics and the flexible tea-stem gripper.

What
----
:class:`MotorJointPlant` integrates the joint in motor-joint space (mrad,
mrad/s, A, N*m, V) at the firmware control rate of 1 kHz.  :class:`GripperPlant`
integrates the linear jaw (mm, N) together with the nonlinear stem-contact force
and the two damage indicators.

Why the current loop is a first-order lag instead of the raw electrical ODE
--------------------------------------------------------------------------
The honest electrical model is ``L di/dt = V - R*i - Ke*w``.  With the real
hardware values ``L = 1.5 mH`` and ``R = 2.0 ohm`` the electrical time constant
is ``tau_e = L/R = 0.75 ms``, which is *shorter than the 1 ms control period*.
An explicit (forward) Euler step of that ODE has the amplification factor
``1 - dt/tau_e = 1 - 1.333 = -0.333``, i.e. it is on the edge of the stability
region (|1 - dt/tau| < 1 requires ``dt < 2*tau_e``; here we are at 1.33*tau_e)
and any reduction of ``L``, or a faster loop, would make the simulation blow up
with a growing oscillation at the Nyquist frequency.  That failure mode is a
property of the *integrator*, not of the machine, and it would be dishonest to
hide it by silently shrinking the step.

The standard, physically faithful fix -- and the one used here -- is to treat the
inner current loop as what it actually is on this drive: a fast first-order
tracking loop with time constant ``tau_e``.  The model becomes
``di/dt = (i_cmd - i)/tau_e``, solved in closed form per tick
(``i <- i_cmd + (i - i_cmd) * exp(-dt/tau_e)``).  This is stable for any ``dt``,
reproduces the 0.75 ms electrical time constant exactly, and still shows the
back-EMF term through the current-command limit that ``V_bus`` imposes.  The
exponential form is used rather than Euler so the loop gain is not silently
weakened: at ``dt = 1 ms`` a plain Euler step would only move the current 74 %
of the way per tick and would mis-state the effective bandwidth.

What happens without it
-----------------------
Using the raw stiff ODE at ``dt = 1 ms`` produces a divergent alternating
current oscillation within a few hundred ticks, which then contaminates the
torque and, through the encoder, the whole cascade.

Units
-----
``theta`` in mrad, ``omega`` in mrad/s, ``i_a`` in A, torque in N*m, PWM in
per-mille (-1000..1000, 1000 == 100 % duty), ``V_bus = 24 V``.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

from . import config as C
from .pid import _finite, clamp

#: Quantisation of the incremental encoder, mrad/count (0.005 deg).
ENCODER_MRAD_PER_COUNT: float = 0.0873


class MotorJointPlant:
    """Rigid single joint driven through a current-controlled PMDC motor.

    Parameters
    ----------
    dt:
        Integration step, seconds.  Defaults to the firmware control period.
    vbus_v:
        DC bus voltage.
    encoder_quantised:
        When ``True``, :meth:`sense_theta` returns the quantised position that a
        real incremental encoder would report.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        vbus_v: float = C.MOTOR_VBUS_V,
        encoder_quantised: bool = True,
    ) -> None:
        """Construct the plant and reset it to the home position."""
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.vbus_v: float = _finite(vbus_v, C.MOTOR_VBUS_V)
        self.encoder_quantised: bool = bool(encoder_quantised)
        #: Closed-form per-tick current-loop decay factor exp(-dt/tau_e).
        self._alpha_e: float = math.exp(-self.dt / C.MOTOR_TAU_E_S)
        #: Clamped state, frozen when the joint is jammed (Scenario F).
        self.clamp_theta_mrad: Optional[float] = None
        #: Commanded PWM after the slew limiter, per-mille.
        self.pwm_cmd: float = 0.0
        self.reset()

    # -- lifecycle ----------------------------------------------------------
    def reset(self, theta_mrad: float = C.JOINT_HOME_MRAD) -> None:
        """Reset every state to rest at ``theta_mrad``.

        Parameters
        ----------
        theta_mrad:
            Initial joint angle.  Angles are clamped into the software limits so
            a scenario cannot start outside the machine's range.
        """
        self.theta_mrad: float = clamp(
            _finite(theta_mrad, C.JOINT_HOME_MRAD), C.JOINT_MIN_MRAD, C.JOINT_MAX_MRAD
        )
        self.omega_mrad_s: float = 0.0
        self.i_a: float = 0.0
        self.i_cmd_a: float = 0.0
        self.pwm_cmd = 0.0
        self.torque_nm: float = 0.0
        self.friction_nm: float = 0.0
        self.back_emf_v: float = 0.0
        self.voltage_v: float = 0.0
        self.voltage_saturated: bool = False
        self.current_limited: bool = False
        self.torque_ext_nm: float = 0.0
        self.clamp_theta_mrad = None

    # -- helpers ------------------------------------------------------------
    def apply_pwm_slew(self, pwm_target: float) -> float:
        """Limit ``pwm_target`` to the maximum PWM slew rate.

        Parameters
        ----------
        pwm_target:
            Requested duty in per-mille.

        Returns
        -------
        float
            The slew-limited duty actually applied to the bridge.
        """
        tgt = clamp(_finite(pwm_target), -C.MAX_PWM_PERMILLE, C.MAX_PWM_PERMILLE)
        step = C.MAX_DUTY_STEP_PERMILLE
        delta = tgt - self.pwm_cmd
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        self.pwm_cmd = clamp(self.pwm_cmd + delta, -C.MAX_PWM_PERMILLE, C.MAX_PWM_PERMILLE)
        return self.pwm_cmd

    def _electrical_step(self, voltage_v: float) -> None:
        """Advance the current state by one tick using the closed-form lag.

        The current command is the value that the applied voltage can support at
        the present speed, ``(V - Ke*w)/R``, bounded by the continuous and peak
        current limits.  Modelling the limit here -- rather than after the fact
        -- is what makes the drive behave correctly near full speed, where the
        back-EMF leaves almost no voltage headroom.

        Parameters
        ----------
        voltage_v:
            Applied terminal voltage after saturation, volts.
        """
        back_emf = C.MOTOR_KE_V_PER_MRAD_S * self.omega_mrad_s
        if abs(C.MOTOR_R_OHM) < C.EPS:
            i_from_voltage = 0.0
        else:
            i_from_voltage = (voltage_v - back_emf) / C.MOTOR_R_OHM
        i_ceiling = C.MAX_CURRENT_A if abs(i_from_voltage) <= C.MAX_CURRENT_A else C.PEAK_CURRENT_A
        i_cmd = clamp(i_from_voltage, -i_ceiling, i_ceiling)
        # Latch the fact that the drive was current- or voltage-limited, both for
        # telemetry and for the over-current / stall detectors.
        self.current_limited = abs(i_from_voltage) > C.MAX_CURRENT_A
        self.i_cmd_a = i_cmd
        self.i_a = i_cmd + (self.i_a - i_cmd) * self._alpha_e
        self.i_a = _finite(self.i_a, i_cmd)

    def _mechanical_step(self, external_torque_nm: float) -> None:
        """Advance angle and speed by one tick.

        Parameters
        ----------
        external_torque_nm:
            Torque applied to the joint by the environment (positive assists
            motion).  A large negative value models a jam.
        """
        omega = self.omega_mrad_s
        # Coulomb friction, smoothed with tanh so that the discontinuity at
        # omega = 0 does not chatter the 1 kHz integrator.  omega_eps = 200
        # mrad/s is narrow compared with any commanded speed, so the model is
        # still essentially "static friction" while remaining Lipschitz.
        friction = C.MOTOR_TC_NM * math.tanh(omega / C.MOTOR_OMEGA_EPS_MRAD_S)
        viscous = C.MOTOR_B_NM_S_RAD * omega
        self.friction_nm = friction + viscous

        # J * domega/dt = Kt*i - b*w - Tc*tanh(w/eps) + Text ; omega in mrad/s so
        # the derivative also carries the 1e-3 factor from mrad -> rad.
        accel_rad_s2 = (
            C.MOTOR_KT_NM_PER_A * self.i_a - self.friction_nm + _finite(external_torque_nm)
        ) / C.MOTOR_J_KG_M2
        self.torque_nm = C.MOTOR_KT_NM_PER_A * self.i_a
        self.torque_ext_nm = _finite(external_torque_nm)

        omega = omega + accel_rad_s2 * 1000.0 * self.dt
        theta = self.theta_mrad + omega * self.dt

        # Jam model: hold the shaft.  Absorbing the momentum is what a real
        # obstruction does, so the speed is zeroed instead of being integrated
        # through the obstacle.
        if self.clamp_theta_mrad is not None:
            theta = self.clamp_theta_mrad
            omega = 0.0

        # Soft end-stops: the joint cannot leave its travel.
        if theta < C.JOINT_MIN_MRAD:
            theta = C.JOINT_MIN_MRAD
            omega = max(0.0, omega)
        elif theta > C.JOINT_MAX_MRAD:
            theta = C.JOINT_MAX_MRAD
            omega = min(0.0, omega)

        self.theta_mrad = _finite(theta, self.theta_mrad)
        self.omega_mrad_s = _finite(omega, 0.0)

    # -- public API ---------------------------------------------------------
    def step(
        self,
        pwm_permille: float,
        external_torque_nm: float = 0.0,
    ) -> Dict[str, float]:
        """Advance the joint by one control tick.

        Parameters
        ----------
        pwm_permille:
            Requested PWM duty in per-mille (slew-limited internally).
        external_torque_nm:
            Environmental torque, N*m.

        Returns
        -------
        dict
            Snapshot with keys ``theta_mrad``, ``omega_mrad_s``, ``i_a``,
            ``torque_nm``, ``pwm``, ``voltage_v``, ``voltage_saturated``,
            ``current_limited``, ``back_emf_v``, ``friction_nm``.
        """
        pwm = self.apply_pwm_slew(pwm_permille)
        voltage = pwm / C.MAX_PWM_PERMILLE * self.vbus_v
        self.voltage_saturated = abs(voltage) > self.vbus_v
        voltage = clamp(voltage, -self.vbus_v, self.vbus_v)
        self.voltage_v = voltage
        self.back_emf_v = C.MOTOR_KE_V_PER_MRAD_S * self.omega_mrad_s

        self._electrical_step(voltage)
        self._mechanical_step(external_torque_nm)
        return self.snapshot()

    def sense_theta(self) -> float:
        """Return the joint angle as the encoder would report it, in mrad."""
        if not self.encoder_quantised:
            return self.theta_mrad
        counts = round(self.theta_mrad / ENCODER_MRAD_PER_COUNT)
        return counts * ENCODER_MRAD_PER_COUNT

    def snapshot(self) -> Dict[str, float]:
        """Return the current state as a plain ``dict`` of floats."""
        return {
            "theta_mrad": self.theta_mrad,
            "omega_mrad_s": self.omega_mrad_s,
            "i_a": self.i_a,
            "torque_nm": self.torque_nm,
            "pwm": self.pwm_cmd,
            "voltage_v": self.voltage_v,
            "voltage_saturated": 1.0 if self.voltage_saturated else 0.0,
            "current_limited": 1.0 if self.current_limited else 0.0,
            "back_emf_v": self.back_emf_v,
            "friction_nm": self.friction_nm,
        }


class GripperPlant:
    """Parallel-jaw gripper with a nonlinear, damageable tea-stem contact.

    The jaw is commanded in velocity and follows a first-order lag with time
    constant ``tau_jaw = 8 ms``.  The tea stem is modelled as a nonlinear
    stiffening spring whose surface sits at :data:`config.STEM_GAP_MM`;
    contact begins only when the jaw gap is smaller than that, and the
    deflection is ``d = stem_gap_mm - gap_mm``.

    Force law
    ---------
    ``F = k1*d + k2*d*d + c*|v_closing|`` with ``k1 = 0.30 N/mm`` and
    ``k2 = 0.06 N/mm^2``.  The quadratic term makes the stem stiffen as it is
    squeezed, which is what turns a modest grip-force error into a large force
    error and therefore what makes force feedback necessary at all.

    Damage model
    ------------
    ``damaged`` latches permanently once ``F > damage_force_n``; additionally an
    accumulated ``damage_index`` integrates ``max(0, F - damage_force_n) * dt``
    so that a short overload and a long mild overload are distinguishable.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        stem_gap_mm: float = C.STEM_GAP_MM,
        damage_force_n: float = C.DAMAGE_FORCE_N,
    ) -> None:
        """Construct the gripper and open it fully."""
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.stem_gap_mm: float = _finite(stem_gap_mm, C.STEM_GAP_MM)
        self.damage_force_n: float = _finite(damage_force_n, C.DAMAGE_FORCE_N)
        self._alpha_jaw: float = math.exp(-self.dt / C.GRIPPER_TAU_JAW_S)
        self.reset()

    def reset(self, gap_mm: float = C.GRIPPER_OPEN_MM) -> None:
        """Reset gap, force, damage flags and the pick travel integrator."""
        self.gap_mm: float = clamp(
            _finite(gap_mm, C.GRIPPER_OPEN_MM), C.GRIPPER_MIN_MM, C.GRIPPER_MAX_MM
        )
        self.v_cmd_mm_s: float = 0.0
        self.v_mm_s: float = 0.0
        self.force_n: float = 0.0
        self.peak_force_n: float = 0.0
        self.damaged: bool = False
        self.damage_index: float = 0.0
        self.in_contact: bool = False
        self.deflection_mm: float = 0.0
        #: Vertical travel of the tool centre point, mm (negative == lifting).
        self.z_mm: float = 0.0
        #: Accumulated twist applied during detachment, deg.
        self.twist_deg: float = 0.0

    def contact_force(self, gap_mm: float, v_mm_s: float) -> float:
        """Evaluate the stem contact force for a candidate gap.

        Parameters
        ----------
        gap_mm:
            Jaw gap, mm.
        v_mm_s:
            Jaw velocity, mm/s (negative while closing).

        Returns
        -------
        float
            Contact force in newtons; ``0.0`` when there is no contact.
        """
        if gap_mm >= self.stem_gap_mm:
            return 0.0
        d = self.stem_gap_mm - gap_mm
        elastic = C.STEM_K1_N_PER_MM * d + C.STEM_K2_N_PER_MM2 * d * d
        # Damping opposes closure only, so opening the jaw is not braked by a
        # stem that is no longer being compressed.
        closing = max(0.0, -v_mm_s)
        damped = C.STEM_DAMP_N_S_PER_MM * closing
        return _finite(elastic + damped)

    def step(self, v_cmd_mm_s: float) -> Dict[str, float]:
        """Advance the jaw by one control tick.

        Parameters
        ----------
        v_cmd_mm_s:
            Commanded jaw velocity in mm/s.  Negative closes the jaw.

        Returns
        -------
        dict
            ``gap_mm``, ``force_n``, ``v_mm_s``, ``contact``, ``damaged``,
            ``damage_index``, ``peak_force_n``, ``deflection_mm``.
        """
        cmd = clamp(
            _finite(v_cmd_mm_s),
            -C.GRIPPER_MAX_SPEED_MM_S,
            C.GRIPPER_MAX_SPEED_MM_S,
        )
        self.v_cmd_mm_s = cmd
        # Closed-form first-order lag, same rationale as the current loop.
        self.v_mm_s = cmd + (self.v_mm_s - cmd) * self._alpha_jaw
        self.v_mm_s = _finite(self.v_mm_s, cmd)

        gap = self.gap_mm + self.v_mm_s * self.dt
        # The jaw is rigid against its own end-stops; the spring lives in the
        # stem, not in the mechanism.
        if gap < C.GRIPPER_MIN_MM:
            gap = C.GRIPPER_MIN_MM
        elif gap > C.GRIPPER_MAX_MM:
            gap = C.GRIPPER_MAX_MM
        self.gap_mm = gap

        self.deflection_mm = max(0.0, self.stem_gap_mm - self.gap_mm)
        self.force_n = self.contact_force(self.gap_mm, self.v_mm_s)
        self.in_contact = self.force_n > 0.0
        if self.force_n > self.peak_force_n:
            self.peak_force_n = self.force_n
        if self.force_n > self.damage_force_n:
            self.damaged = True
            self.damage_index += (self.force_n - self.damage_force_n) * self.dt
        # A detached (or absent) stem stops the tool from advancing in z: the
        # jaw gap no longer pushes the mechanism down.
        self.z_mm += 0.25 * max(0.0, -self.v_mm_s) * self.dt
        self.z_mm -= 0.10 * max(0.0, self.v_mm_s) * self.dt
        return self.snapshot()

    def advance_detach(self, pull_mm_s: float, twist_deg_s: float) -> None:
        """Apply the detachment motion (a pull in z together with a twist).

        Parameters
        ----------
        pull_mm_s:
            Upward (negative z) tool velocity, mm/s.
        twist_deg_s:
            Twist rate, deg/s.  A real pick rotates the stem free of the bud.
        """
        self.z_mm += _finite(pull_mm_s) * self.dt
        self.twist_deg += _finite(twist_deg_s) * self.dt

    def snapshot(self) -> Dict[str, float]:
        """Return the current gripper state as a plain ``dict`` of floats."""
        return {
            "gap_mm": self.gap_mm,
            "force_n": self.force_n,
            "v_mm_s": self.v_mm_s,
            "contact": 1.0 if self.in_contact else 0.0,
            "damaged": 1.0 if self.damaged else 0.0,
            "damage_index": self.damage_index,
            "peak_force_n": self.peak_force_n,
            "deflection_mm": self.deflection_mm,
            "z_mm": self.z_mm,
            "twist_deg": self.twist_deg,
        }
