"""
Plant physics and scenario defaults.

One of the three value tables that make up :mod:simulator.config.  This part
holds the constants the *simulator* needs and the firmware does not have: the
motor electrical and mechanical parameters, the nonlinear stem-contact law, the
damage threshold, and the fault-injection defaults for the seven scenarios.

`DAMAGE_FORCE_N` deliberately differs from the firmware and is documented in
place, next to the firmware value it replaces.
"""

from __future__ import annotations

from typing import Dict, Tuple

from ._core import CONTROL_DT_S, GRIPPER_MAX_MM  # noqa: F401 - derived defaults


# =============================================================================
# 9. Plant physics  (simulator-only; the firmware has no plant model)
# =============================================================================

#: Stator inductance, H.
MOTOR_L_H: float = 1.5e-3
#: Stator resistance, ohm.
MOTOR_R_OHM: float = 2.0
#: Electrical time constant tau_e = L/R, s.  See plant.py docstring: the current
#: loop is integrated as this first-order lag rather than as the raw stiff ODE.
MOTOR_TAU_E_S: float = MOTOR_L_H / MOTOR_R_OHM          # 0.75 ms
#: Back-EMF constant, V*s/rad (post-gearbox effective value).
MOTOR_KE_V_S_RAD: float = 0.05
#: Back-EMF constant in V per (mrad/s): 0.05/1000.
MOTOR_KE_V_PER_MRAD_S: float = MOTOR_KE_V_S_RAD / 1000.0
#: Effective torque constant, N*m/A (post-gearbox).
MOTOR_KT_NM_PER_A: float = 2.0
#: Reflected rotor inertia, kg*m^2.
MOTOR_J_KG_M2: float = 2.5e-3
#: Viscous friction coefficient, N*m*s/rad.
MOTOR_B_NM_S_RAD: float = 3.0e-5
#: Coulomb friction torque, N*m.
MOTOR_TC_NM: float = 0.02
#: Velocity scale of the tanh smoothing applied to Coulomb friction, mrad/s.
MOTOR_OMEGA_EPS_MRAD_S: float = 200.0
#: Motor bus voltage, V.
MOTOR_VBUS_V: float = 24.0
#: Numerically smallest denominator accepted by the PID etc., to avoid /0.
EPS: float = 1.0e-12

# --- Gripper plant ----------------------------------------------------------
#: Jaw velocity-loop time constant, s.
GRIPPER_TAU_JAW_S: float = 8.0e-3
#: Gap at which the tea stem surface sits, mm.  Contact starts below this.
STEM_GAP_MM: float = 6.0
#: Linear term of the nonlinear stiffening stem stiffness, N/mm.
STEM_K1_N_PER_MM: float = 0.30
#: Quadratic term of the nonlinear stiffening stem stiffness, N/mm^2.
STEM_K2_N_PER_MM2: float = 0.06
#: Damping coefficient added while the jaw closes onto a stem, N*s/mm.
STEM_DAMP_N_S_PER_MM: float = 0.010
#: Latching damage threshold, N.
#:
#: The value is bounded above by physics and below by usefulness, and 3.8 N sits
#: in that window.  With ``k1 = 0.30 N/mm``, ``k2 = 0.06 N/mm^2`` and the stem
#: surface at 6.0 mm, a fully closed jaw (gap = 0) generates
#: ``0.30*6 + 0.06*36 = 3.96 N`` -- *measured* as 3.9941 N in Scenario D, which
#: drives the jaw to its hard stop.  That is the largest force this gripper can
#: ever produce, so the firmware's 4.0 N is unreachable by about 0.006 N and any
#: test of the damage path would fail for a geometric reason rather than a control
#: one.  3.8 N leaves a ~0.19 N margin, keeps the 4.8 N Scenario D command above
#: it, and still leaves the 5.0 N overload trip as the level the jaw cannot reach
#: in normal operation.  The firmware value is retained below for traceability.
DAMAGE_FORCE_N: float = 3.8
#: Firmware value of the stem damage threshold, N (``TP_FORCE_*``/config.h 4.0).
DAMAGE_FORCE_N_FIRMWARE: float = 4.0
#: Jaw gap at which the gripper starts, mm (fully open).
GRIPPER_OPEN_MM: float = GRIPPER_MAX_MM

#: Knee of the soft-closing approximation used for the trapezoidal profile.
PROFILE_EPS: float = 1.0e-9
# =============================================================================
# 10. Scenario defaults
# =============================================================================

#: Time the power-on self-test takes before the machine may move, s.
SELF_CHECK_HOLD_S: float = 0.010
#: Default run length of a single scenario, s.
DEFAULT_DURATION_S: float = 6.0
#: Default RNG seed (deterministic runs unless overridden).
DEFAULT_SEED: int = 20240517
#: Nominal target joint position for a pick, mrad (30 deg).
NOMINAL_TARGET_MRAD: float = 30000.0
#: Scenario B lateral target error, mm.
SCEN_B_LATERAL_ERROR_MM: float = 8.0
#: Scenario B depth target error, mm.
SCEN_B_DEPTH_ERROR_MM: float = 5.0
#: Scenario C force-measurement noise standard deviation, N.
SCEN_C_FORCE_NOISE_SIGMA_N: float = 0.08
#: Scenario D grip-force target, N.  Above DAMAGE_FORCE_N, and high enough to be
#: an over-command: at a fully closed jaw the stem law can produce at most
#: ``0.30*6 + 0.06*36 = 4.99 N``, so asking for 4.8 N drives the grip into the
#: damage region and tests whether the protection notices.
SCEN_D_FORCE_TARGET_N: float = 4.8
#: Scenario E time at which the host link is dropped, s.
SCEN_E_LINK_DROP_S: float = 0.300
#: Scenario F external resisting torque applied to the stalled joint, N*m.
SCEN_F_RESIST_TORQUE_NM: float = 30.0
#: Scenario F time at which the joint is clamped, s.
SCEN_F_CLAMP_AT_S: float = 0.400
#: Scenario G time at which the E-stop is asserted, s.
SCEN_G_ESTOP_AT_S: float = 1.200

#: Fault code strings (mirror the firmware's status.h error codes).
ERR_NONE: str = "ERR_NONE"
ERR_COMM_TIMEOUT: str = "ERR_COMM_TIMEOUT"
ERR_MOTOR_STALL: str = "ERR_MOTOR_STALL"
ERR_FORCE_OVERLOAD: str = "ERR_FORCE_OVERLOAD"
ERR_FORCE_SOFT_LIMIT: str = "ERR_FORCE_SOFT_LIMIT"
ERR_TARGET_UNREACHABLE: str = "ERR_TARGET_UNREACHABLE"
ERR_SENSOR: str = "ERR_SENSOR"
ERR_OVERCURRENT: str = "ERR_OVERCURRENT"
ERR_ESTOP: str = "ERR_ESTOP"
ERR_SELF_CHECK: str = "ERR_SELF_CHECK"

#: Directory (relative to the project root) for PNG figures.
RESULTS_DIR: str = "data/results"
#: Directory (relative to the project root) for CSV/JSON logs.
LOGS_DIR: str = "data/logs"

#: Human-readable scenario titles, used by plots and the CLI summary table.
SCENARIO_TITLES: Dict[str, str] = {
    "A": "Normal pick",
    "B": "Target position error",
    "C": "Force-sensor noise",
    "D": "Excessive grip force",
    "E": "Communication timeout",
    "F": "Motor stall",
    "G": "Emergency stop",
}

#: Ordered scenario identifiers.
SCENARIO_IDS: Tuple[str, ...] = ("A", "B", "C", "D", "E", "F", "G")


def scenario_title(key: str) -> str:
    """Return a human-readable title for a scenario identifier.

    Parameters
    ----------
    key:
        Scenario letter, e.g. ``"A"``.  Case-insensitive.

    Returns
    -------
    str
        The title, or ``"Scenario <key>"`` when the key is unknown, so that a
        caller can never crash merely because it printed a label.
    """
    k = str(key).strip().upper()
    return SCENARIO_TITLES.get(k, "Scenario {0}".format(k))

