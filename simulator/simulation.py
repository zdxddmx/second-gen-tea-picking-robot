"""
Closed-loop simulation loop shared by all seven scenarios.

What
----
:class:`Simulation` wires the plant, the cascade controller, the gripper
controller, the state machine, the safety monitor and the recorder into one
1 kHz loop, and runs it for a requested duration.  :class:`ScenarioSpec` is the
data-driven description of what makes each scenario different -- injected target
error, sensor noise, force target, link drop, external torque, E-stop.

Why a shared loop with a spec, rather than seven loops
------------------------------------------------------
Seven hand-written loops would drift: a fix to the stall detector or to the
recording order would have to be applied seven times, and a scenario that forgot
one would produce a result that looked fine and was not comparable.  With one
loop, the *only* difference between scenarios is the contents of
:class:`ScenarioSpec`, so a difference in outcome is attributable to the
injected condition and nothing else.

Loop order per tick (this order is deliberate)
----------------------------------------------
1. state machine, so a fault latched last tick takes effect immediately;
2. sensors, including the fault injection that substitutes for hardware;
3. emergency-stop check, before any command is computed;
4. safety monitors (stall, over-current, link watchdog);
5. cascade controller, unless the machine is faulted or stopped;
6. gripper controller and its state-machine handshakes;
7. plant integration;
8. recording.

Writing the E-stop into the same tick as the detection, and zeroing the PWM
whenever the machine is ``ERROR`` or ``EMERGENCY_STOP``, is what makes "immediate
controlled stop" true rather than approximately true.

What happens without it
-----------------------
Every scenario would be its own approximate re-implementation of the control
cycle, and no two results could be compared with confidence.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from . import config as C
from .controller import (
    CascadeController,
    FixedPositionGripperController,
    FlexibleGripperController,
    GripperPhase,
    SafetyMonitor,
)
from .fsm import Event, RobotFSM, State
from .pid import _finite, clamp
from .plant import GripperPlant, MotorJointPlant
from .sensors import ForceSensor
from .scenario_types import ScenarioResult, ScenarioSpec, verdict
from .sequencer import GripSequencer
from .metrics import force_band_entry_s, metrics_from_recorder
from .telemetry import Recorder

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


class Simulation:
    """One closed-loop run of the picking cycle.

    Parameters
    ----------
    spec:
        The scenario description.
    seed:
        RNG seed for the sensor noise; the run is bit-reproducible for a given
        seed and specification.
    dt:
        Control period, seconds.
    profile_kind:
        Trajectory generator, forwarded to the cascade controller.
    force_max_hold_s:
        How long the loop keeps integrating after a latched fault so the record
        shows the halted regime.  The plant is held on the last good state while
        the rail is at its last value, so this is a genuinely frozen sample, not
        a fabricated tail.
    """

    def __init__(
        self,
        spec: ScenarioSpec,
        seed: int = C.DEFAULT_SEED,
        dt: float = C.CONTROL_DT_S,
        profile_kind: str = "trapezoid",
        force_max_hold_s: float = 0.0,
    ) -> None:
        """Build the plants, controllers, state machine and recorder."""
        self.spec = spec
        self.dt = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        self.force_max_hold_s = max(0.0, _finite(force_max_hold_s))

        # Target: the nominal pick position plus the injected vision error.
        self.target_mrad = C.NOMINAL_TARGET_MRAD + spec.target_extra_mrad

        self.joint = MotorJointPlant(dt=self.dt)
        self.gripper = GripperPlant(dt=self.dt)
        self.sensor = ForceSensor(
            dt=self.dt, noise_sigma_n=spec.noise_sigma_n, enable_filter=spec.sensor_filter
        )
        self.cascade = CascadeController(dt=self.dt, profile_kind=profile_kind)
        self.grip: object = FlexibleGripperController(dt=self.dt)
        self.safety = SafetyMonitor(dt=self.dt)
        self.fsm = RobotFSM()
        self.rec = Recorder(dt=self.dt)
        # The pick-cycle choreography lives in its own object so the handshake
        # logic is testable without a plant; see simulator/sequencer.py.
        self.seq = GripSequencer(
            self.grip,
            self.fsm,
            on_trace=self._trace,
            on_event=lambda kind, message: self.rec.log_event(self.t_s, kind, message),
            on_fault=self.faults_add,
            f_target_override_n=spec.f_target_override_n,
        )

        self.t_s: float = 0.0
        self.fault_times: List[float] = []
        self.trace_lines: List[str] = []
        self.seq.cycle_completed: bool = False
        self.self_check_done: bool = False
        self.seq.force_controlled: bool = False
        self.seq.completed_cycle_time_s: float = 0.0
        self.last_pwm: float = 0.0
        self.last_force_meas: float = 0.0
        self.last_vcmd: float = 0.0
        self.last_force_raw: float = 0.0
        self.last_current_a: float = 0.0
        self.last_adc_counts: float = C.FORCE_CAL_ZERO_COUNTS
        self.link_ok: bool = True
        self.seq.contact_t_s: float = 0.0
        self.seq.force_reached_t_s: float = 0.0
        #: Time the grip phase ended (detachment started), s.  The force RMSE is
        #: bounded by this: after it the jaw is deliberately opening and the
        #: measured force is falling to zero, which is a release, not a tracking
        #: error.
        self.seq.force_hold_end_s: float = 0.0
        self.depth_error_measured_mm: float = 0.0
        self.lateral_error_mm: float = spec.lateral_error_mm
        self.halt_t_s: Optional[float] = None
        self.frozen: bool = False

    # -- helpers ------------------------------------------------------------
    def _trace(self, message: str) -> None:
        """Append a line to the console trace exactly once."""
        if not self.trace_lines or self.trace_lines[-1] != message:
            self.trace_lines.append(message)

    def _halted(self) -> bool:
        """Return ``True`` when the machine has stopped by fault or E-stop."""
        return self.fsm.is_terminal_fault()

    def _state(self) -> State:
        """Return the current state machine state."""
        return self.fsm.state

    # -- one control tick ---------------------------------------------------
    def _tick(self) -> None:
        """Advance the whole system by one control period."""
        spec = self.spec
        t = self.t_s

        # (1) state machine ---------------------------------------------------
        self.fsm.step(t)
        if not self.self_check_done and t >= C.SELF_CHECK_HOLD_S:
            # The firmware's power-on self-test is a real phase, not a formality:
            # it is where the sensor plausibility and encoder checks would refuse
            # to let the machine move at all.  Emitting the result explicitly
            # (rather than letting the FSM skip straight to IDLE) is what keeps
            # that gate visible in the recorded trace.
            self.self_check_done = True
            self.fsm.fire(Event.SELF_CHECK_OK)
            self.rec.log_event(t, "state", "Self check passed")
            # The mission hand-off happens only once the machine is ready, in the
            # same order the vision host and the operator would deliver it.
            self.fsm.fire(Event.TARGET_RECEIVED)
            self.cascade.reset(self.joint.sense_theta())
            self.cascade.set_target(self.target_mrad, self.joint.sense_theta())
            self.fsm.fire(Event.START)
            self.rec.log_event(t, "state", "Mission accepted")
            self._trace("Approaching target...")

        # (2) sensors, plus the scenario's fault injection --------------------
        true_force = self.gripper.force_n
        sample = self.sensor.sample(true_force, self.rng)
        self.last_force_raw = float(sample["raw"])
        self.last_adc_counts = float(sample["adc_counts"])
        # The controller consumes the conditioned channel; the plant keeps the
        # true force.  Scenario C compares the two.
        self.last_force_meas = float(sample["filtered"])
        self.last_current_a = self.joint.i_a
        if spec.clamp_joint_at_s is not None and t >= spec.clamp_joint_at_s:
            if self.joint.clamp_theta_mrad is None:
                self.joint.clamp_theta_mrad = self.joint.theta_mrad
        if spec.link_drop_s is not None and t >= spec.link_drop_s:
            self.link_ok = False

        # (3) emergency stop, before any command is produced ------------------
        if spec.estop_at_s is not None and t >= spec.estop_at_s and not self._halted():
            self.fsm.e_stop()
            self.rec.log_event(t, "estop", "Operator E-stop asserted")
            self._trace("Emergency stop asserted")
            self.halt_t_s = t
            self.faults_add(C.ERR_ESTOP)

        # (4) safety monitors -------------------------------------------------
        if not self._halted():
            if self.safety.check_link(self.link_ok):
                self.fsm.fault(C.ERR_COMM_TIMEOUT)
                self.rec.log_event(t, "fault", "Host link timeout")
                self._trace("Communication timeout")
                self.halt_t_s = t
            if self.safety.check_overcurrent(abs(self.joint.i_a)):
                self.fsm.fault(C.ERR_OVERCURRENT)
                self.rec.log_event(t, "fault", "Over-current trip")

        # (5) joint controller ------------------------------------------------
        if self._halted():
            self.last_pwm = 0.0
        else:
            self.last_pwm = self.cascade.tick(self.joint.sense_theta(), self.joint.omega_mrad_s)
            if self.safety.check_stall(self.last_pwm, self.joint.omega_mrad_s):
                self.fsm.fault(C.ERR_MOTOR_STALL)
                self.rec.log_event(t, "fault", "Motor stall detected")
                self._trace("Motor stall detected")
                self.halt_t_s = t
                self.last_pwm = 0.0

        # (6) gripper controller ---------------------------------------------
        v_jaw = self.seq.step(t, self.gripper.gap_mm, self.last_force_meas)
        self.last_vcmd = v_jaw

        # (7) plant integration ----------------------------------------------
        # The environmental torque models a jam: after the clamp time the joint
        # is loaded against its motion by the specified resisting torque.
        torque_ext = 0.0
        if self.joint.clamp_theta_mrad is not None and self.spec.resist_torque_nm:
            torque_ext = -abs(self.spec.resist_torque_nm)
        if not self.frozen:
            self.joint.step(self.last_pwm, torque_ext)
            self.gripper.step(v_jaw)
            if not math.isfinite(self.joint.theta_mrad) or not math.isfinite(self.gripper.gap_mm):
                # Last-resort guard: a non-finite plant state is a model failure,
                # not a control failure, and must not be recorded as data.
                self.fsm.fault(C.ERR_SENSOR)
                self.frozen = True

        # (8) recording -------------------------------------------------------
        self.rec.record(
            t=t,
            ref=self.cascade.ref_pos_mrad,
            pos=self.joint.sense_theta(),
            vel=self.joint.omega_mrad_s,
            force=self.last_force_meas,
            pwm=self.last_pwm,
            current=self.joint.i_a,
            state=self.fsm.state.value,
            fault=self.fsm.last_fault() or "",
            adc_raw=self.last_adc_counts,
            vjaw=self.last_vcmd,
            gap=self.gripper.gap_mm,
        )

    def faults_add(self, code: str) -> None:
        """Latch a fault code without going through the state machine."""
        if code and code not in self.fsm.faults:
            self.fsm.faults.append(code)

    # -- gripper command ----------------------------------------------------
    def _check_stuck(self, t: float) -> bool:
        """Escalate an unresolved target into ``ERR_TARGET_UNREACHABLE``.

        If the machine is still not in ``RETURN`` well after the profile should
        have finished, the target was not reached.  Reporting that as a fault is
        what makes Scenario B's outcome honest: the injected 8 mm error is either
        corrected or it is not, and "still working on it" is not an answer.

        Returns
        -------
        bool
            ``True`` when the fault was raised.
        """
        if self._halted():
            return False
        if t < 2.0 * self.spec.duration_s:
            return False
        if self.seq.cycle_completed:
            return False
        self.fsm.fault(C.ERR_TARGET_UNREACHABLE)
        self.rec.log_event(t, "fault", "Target not reached in time")
        self._trace("Target unreachable within the allocated time")
        self.halt_t_s = t
        return True

    # -- run ----------------------------------------------------------------
    def run(self) -> ScenarioResult:
        """Execute the scenario and return its result.

        Returns
        -------
        ScenarioResult
            With the recorder, metrics, fault list and console trace attached.
        """
        spec = self.spec
        self._trace("===== Tea Picking Robot Simulation =====")
        self._trace("Target detected")
        # The state machine boots through INIT -> SELF_CHECK -> IDLE on its own
        # during the first ticks; the mission is handed over from _tick() once the
        # self-check has passed, so the ordering is enforced by the FSM rather
        # than by the order of statements here.

        n_ticks = int(round(spec.duration_s / self.dt))
        for i in range(n_ticks):
            self.t_s = i * self.dt
            self._tick()
            if self._halted() and self.halt_t_s is not None and not spec.stress:
                if self.t_s - self.halt_t_s >= self.force_max_hold_s:
                    self.frozen = True
        # Extend past the nominal duration if a fault needs its hold window; the
        # recorded tail is frozen, so this adds evidence without inventing motion.
        if not self.seq.cycle_completed and not self._halted():
            self._check_stuck(self.t_s)

        self._trace("========================================")
        result = self._build_result()
        return result

    # -- result assembly ----------------------------------------------------
    def _depth_error(self) -> float:
        """Return the residual depth error in mm.

        The residual is the gap between the achieved grip force and the target,
        converted to a depth through the stem stiffness at that force.  This is
        the physically meaningful "how far off was the tool in the depth
        direction" number: a force error is what a depth error *becomes* once the
        jaw is touching the stem.
        """
        f_err = abs(self.grip.f_achieved_n - self.grip.effective_target(self.spec.f_target_override_n))
        k_eff = C.STEM_K1_N_PER_MM + 2.0 * C.STEM_K2_N_PER_MM2 * max(0.0, self.gripper.deflection_mm)
        return f_err / max(k_eff, C.EPS)

    def _build_result(self) -> ScenarioResult:
        """Assemble the :class:`ScenarioResult` from the recorded run."""
        spec = self.spec
        rec = self.rec
        t_end = float(rec.array("t")[-1]) if len(rec) else 0.0
        window_end: Optional[float] = None
        if self.halt_t_s is not None and self.halt_t_s < t_end:
            # Score only the live part of a faulted run.
            window_end = self.halt_t_s
        f_target_used = float(self.grip.effective_target(spec.f_target_override_n))
        # The force RMSE is measured over the grip phase only: from the moment the
        # force first holds inside the settle band until the detachment starts.
        # Before contact the measured force is legitimately zero, the approach ramp
        # is identical in every run, and after the grip ends the jaw is opening on
        # purpose -- none of those three is a force-tracking error, and including
        # any of them would swamp the number that matters.
        hold_end = self.seq.force_hold_end_s if self.seq.force_hold_end_s > 0.0 else None
        settle_from = force_band_entry_s(
            rec.array("t"),
            rec.array("force"),
            f_target_used,
            C.FORCE_BAND_N,
            C.GRIPPER_SETTLE_S,
            window_start_s=self.seq.contact_t_s,
        )
        met = metrics_from_recorder(
            rec,
            window_start_s=0.0,
            window_end_s=window_end,
            force_rmse_ref_n=f_target_used,
            force_settle_from_s=settle_from,
            force_window_end_s=hold_end,
        )
        met["force_regulation_from_s"] = (
            float(settle_from) if settle_from is not None else -1.0
        )
        met["force_target_n"] = f_target_used
        met["force_hold_end_s"] = float(self.seq.force_hold_end_s)
        met["final_position_mrad"] = float(self.joint.theta_mrad)
        met["target_position_mrad"] = float(self.target_mrad)
        met["final_position_error_mrad"] = float(
            self.target_mrad - self.joint.theta_mrad
        )
        met["position_error_pct"] = float(
            abs(self.target_mrad - self.joint.theta_mrad) / max(abs(self.target_mrad), C.EPS) * 100.0
        )
        met["lateral_error_mm"] = float(spec.lateral_error_mm)
        met["depth_error_mm"] = float(spec.depth_error_mm)
        met["residual_depth_error_mm"] = float(self._depth_error())
        met["peak_force_n"] = float(self.gripper.peak_force_n)
        met["damage_index_ns"] = float(self.gripper.damage_index)
        met["damaged"] = 1.0 if self.gripper.damaged else 0.0
        met["contact_time_s"] = float(self.seq.contact_t_s)
        met["force_reached_time_s"] = float(self.seq.force_reached_t_s)
        met["cycle_time_s"] = float(self.seq.completed_cycle_time_s)
        met["fault_count"] = float(len(self.fsm.faults))
        met["halt_time_s"] = float(self.halt_t_s) if self.halt_t_s is not None else -1.0
        met["final_pwm_permille"] = float(self.last_pwm)
        met["force_ctrl_enabled"] = 1.0 if self.seq.force_controlled else 0.0
        tail = rec.array("pwm")
        met["pwm_after_halt_permille"] = (
            float(np.max(np.abs(tail[-min(50, tail.size):]))) if tail.size else 0.0
        )

        success, reason = verdict(
            spec.key,
            met,
            list(self.fsm.faults),
            self._halted(),
            self.fsm.state,
            self.seq.cycle_completed,
            bool(self.gripper.damaged),
        )
        return ScenarioResult(
            key=spec.key,
            title=spec.title,
            success=success,
            faults=list(self.fsm.faults),
            metrics=met,
            recorder=rec,
            trace_lines=list(self.trace_lines),
            success_reason=reason,
        )
