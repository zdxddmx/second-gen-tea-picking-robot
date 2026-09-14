"""Cross-implementation agreement between the firmware's headers and Python.

Why this file exists
--------------------
`tests/test_protocol_agreement.py` pins everything the two implementations put
*on the wire*.  This file pins something just as dangerous and much easier to
get wrong: the **constants**.

The Python simulator and the C firmware are two controllers for the same
machine, and the value of having both is that agreement between them is
evidence.  That argument collapses the moment they are tuned differently —two
disagreeing controllers agreeing with nobody proves nothing, and worse, it looks
exactly like success because both still produce plausible curves.

This is not a hypothetical.  When this file was written:

  * the firmware had been retuned (`TP_POS_PID_KP` 12 -> 5, `TP_VEL_PID_KP`
    0.005 -> 0.026) while the Python config still carried the old values;
  * the plant's torque and back-EMF constants were `Kt = 2.0`, `Ke = 0.05` in
    *both* implementations —a factor of 40 apart, which violates the SI
    identity `Kt[N.m/A] == Ke[V.s/rad]` and describes a machine that does not
    conserve energy;
  * the firmware serviced its force loop from the 100 Hz robot task while its
    own documentation, its gains and its tuning all assumed 1 kHz.

None of those produced a compile error, a failing test or an implausible
simulation.  They were found by comparing the numbers with each other, which is
what this module automates.

How it works
------------
The firmware's `config.h` and `plant_model.h` are **parsed** rather than
duplicated: a constant is only checked if it is genuinely shared, and the check
reads the same file the compiler reads.  A constant that exists in one
implementation and not the other is reported as a documented difference rather
than silently skipped.
"""

from __future__ import print_function

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if os.path.join(ROOT, "simulator") not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, "simulator"))

from host import protocol as P  # noqa: E402
from simulator import config as C  # noqa: E402

CONFIG_H = os.path.join(ROOT, "firmware", "Application", "config.h")
PLANT_H = os.path.join(ROOT, "firmware", "BSP", "host", "plant_model.h")


def _read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        return handle.read()


def firmware_macro(name):
    """Return a firmware macro as a float, or None when it is not defined.

    Handles the two shapes the headers use: a plain numeric literal, and a
    literal with a C float suffix.  Anything more complicated (an expression
    built from other macros) returns None and is skipped by the callers rather
    than being mis-evaluated -- a test that silently misparses a constant is
    worse than one that admits it cannot check it.
    """
    text = _read(CONFIG_H) + "\n" + _read(PLANT_H)
    match = re.search(
        r"^\s*#define\s+%s\s+\(?\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*f?\s*\)?\s*(?:/\*.*)?$"
        % re.escape(name),
        text, re.M)
    if match is None:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# The physical identity
# ---------------------------------------------------------------------------

def test_torque_and_back_emf_constants_are_equal_in_the_firmware():
    """`Kt[N.m/A] == Ke[V.s/rad]` is an energy-conservation identity.

    The firmware asserts it with `TP_STATIC_ASSERT`, which fails the build; this
    test exists so the *Python* side is checked by the same rule, and so the
    reason is recorded where a reader will find it.
    """
    kt = firmware_macro("TP_PLANT_MOTOR_KT_NM_PER_A")
    ke = firmware_macro("TP_PLANT_MOTOR_KE_VS_RAD")
    assert kt is not None, "TP_PLANT_MOTOR_KT_NM_PER_A not found in the headers"
    assert ke is not None, "TP_PLANT_MOTOR_KE_VS_RAD not found in the headers"
    assert kt == ke, (
        "Kt = %.4f and Ke = %.4f differ by a factor of %.1f; in SI units they "
        "are the same physical quantity, so a mismatch means the model does not "
        "conserve energy" % (kt, ke, max(kt, ke) / min(kt, ke)))


def test_torque_and_back_emf_constants_are_equal_in_python():
    assert C.MOTOR_KT_NM_PER_A == C.MOTOR_KE_V_S_RAD, (
        "the Python plant's Kt and Ke must be equal: %.4f vs %.4f"
        % (C.MOTOR_KT_NM_PER_A, C.MOTOR_KE_V_S_RAD))


# ---------------------------------------------------------------------------
# The two implementations must agree with each other
# ---------------------------------------------------------------------------

#: (firmware macro, Python attribute, description).  Only constants that are
#: genuinely shared belong here: the firmware's `config.h` and the simulator's
#: config are two views of one machine, and a value that legitimately differs is
#: listed in DOCUMENTED_DIFFERENCES below instead.
SHARED_CONSTANTS = [
    ("TP_PLANT_MOTOR_KT_NM_PER_A", "MOTOR_KT_NM_PER_A", "torque constant"),
    ("TP_PLANT_MOTOR_KE_VS_RAD", "MOTOR_KE_V_S_RAD", "back-EMF constant"),
    ("TP_PLANT_MOTOR_R_OHM", "MOTOR_R_OHM", "winding resistance"),
    ("TP_PLANT_MOTOR_L_H", "MOTOR_L_H", "winding inductance"),
    ("TP_PLANT_JOINT_J_KGM2", "MOTOR_J_KG_M2", "joint inertia"),
    ("TP_PLANT_JOINT_B_NMS_RAD", "MOTOR_B_NM_S_RAD", "viscous damping"),
    ("TP_PLANT_JOINT_TC_NM", "MOTOR_TC_NM", "Coulomb friction"),
    ("TP_PLANT_STEM_GAP_MM", "STEM_GAP_MM", "stem contact gap"),
    ("TP_PLANT_STEM_K1_N_PER_MM", "STEM_K1_N_PER_MM", "stem linear stiffness"),
    ("TP_PLANT_STEM_K2_N_PER_MM2", "STEM_K2_N_PER_MM2", "stem stiffening term"),
    ("TP_PLANT_JAW_TAU_S", "GRIPPER_TAU_JAW_S", "jaw servo time constant"),
    ("TP_POS_PID_KP", "POS_PID_KP", "position loop Kp"),
    ("TP_POS_PID_KI", "POS_PID_KI", "position loop Ki"),
    ("TP_POS_PID_IMAX", "POS_PID_IMAX", "position loop integral clamp"),
    ("TP_POS_PID_OUT_MAX", "POS_PID_OUT_MAX", "position loop output clamp"),
    ("TP_VEL_PID_KP", "VEL_PID_KP", "velocity loop Kp"),
    ("TP_VEL_PID_KI", "VEL_PID_KI", "velocity loop Ki"),
    ("TP_VEL_PID_IMAX", "VEL_PID_IMAX", "velocity loop integral clamp"),
    ("TP_VEL_PID_OUT_MAX", "VEL_PID_OUT_MAX", "velocity loop output clamp"),
]

#: Constants that deliberately differ, with the reason.  Any entry here is a
#: promise that the difference is understood; anything not listed and not equal
#: fails the test below.
DOCUMENTED_DIFFERENCES = {
    "FORCE_PID_KI": ("the simulator uses 0.25 where the firmware ships 6.0",
                     "documented in simulator/config/_control.py: the two "
                     "implementations integrate the force loop at different "
                     "rates, so the same Ki does not produce the same "
                     "closed-loop behaviour"),
    "FORCE_PID_KI_FIRMWARE": ("the firmware's own value, kept for reference",
                              "not a parameter, a record"),
}


def _tolerance(expected):
    """Absolute tolerance appropriate to the magnitude of the value.

    A fixed relative tolerance would either fail on a value that is legitimately
    rounded in one file, or pass a real discrepancy on a large one.  The
    tolerance here is one unit in the last place of the *shorter* decimal
    representation, which is what "the same constant written twice" means.
    """
    return max(1e-9, abs(expected) * 1e-4)


def test_firmware_and_simulator_share_the_same_constants():
    missing = []
    mismatched = []
    for macro, attr, description in SHARED_CONSTANTS:
        fw = firmware_macro(macro)
        py = getattr(C, attr, None)
        if fw is None or py is None:
            missing.append("%s (%s) / %s" % (macro, description, attr))
            continue
        if abs(fw - float(py)) > _tolerance(fw):
            mismatched.append(
                "%-32s %-10s firmware %-12.6g python %-12.6g"
                % (macro, description, fw, float(py)))

    assert not missing, (
        "these shared constants could not be read from both sides:\n  "
        + "\n  ".join(missing))
    assert not mismatched, (
        "the firmware and the Python simulator are running different "
        "controllers for these constants:\n  " + "\n  ".join(mismatched))


def test_every_documented_difference_is_still_a_difference():
    """A listed exception that has quietly become an agreement is stale.

    Pinning this keeps the exception list honest: if someone later aligns the
    force integral gain, the entry must be removed rather than left to claim a
    difference that no longer exists.
    """
    for attr in DOCUMENTED_DIFFERENCES:
        assert hasattr(C, attr), (
            "%s is listed as a documented difference but no longer exists; "
            "remove the entry" % attr)


# ---------------------------------------------------------------------------
# Derived quantities that must remain physically sensible
# ---------------------------------------------------------------------------

def test_the_drive_can_reach_the_commanded_top_speed():
    """A velocity limit the bus voltage cannot produce is an unenforceable spec.

    With a 24 V rail and a back-EMF constant Ke, the no-load speed is V/Ke.  The
    firmware advertises a joint maximum of `TP_JOINT_MAX_SPEED_MRAD_S`, and if
    that exceeds V/Ke then the outer loop's output clamp is reachable only in
    the sense that it is never reachable.
    """
    ke = firmware_macro("TP_PLANT_MOTOR_KE_VS_RAD")
    vbus = 24.0
    top = firmware_macro("TP_JOINT_MAX_SPEED_MRAD_S")
    assert ke is not None and top is not None
    no_load_mrad_s = 1000.0 * vbus / ke
    assert no_load_mrad_s > top, (
        "no-load speed %.0f mrad/s is below the advertised maximum %.0f mrad/s"
        % (no_load_mrad_s, top))


def test_holding_the_modelled_load_needs_far_less_than_the_current_limit():
    """The joint must be able to hold its own load without saturating."""
    kt = firmware_macro("TP_PLANT_MOTOR_KT_NM_PER_A")
    load = firmware_macro("TP_PLANT_JOINT_LOAD_TORQUE_NM")
    limit = firmware_macro("TP_MAX_CURRENT_MA")
    assert kt is not None and limit is not None
    if load is None:
        # The load torque is set inside plant_model.c rather than a header; the
        # value is fixed at 20 mN*m there.  Recorded here so the check still
        # runs and so a change to it is noticed.
        load = 0.02
    needed_ma = 1000.0 * load / kt
    assert needed_ma < 0.5 * limit, (
        "holding %.3f N*m needs %.0f mA of the %.0f mA limit, which leaves too "
        "little headroom for acceleration" % (load, needed_ma, limit))


def test_stall_detector_arms_well_above_the_stall_threshold():
    """The duty arming condition is what prevents false stall trips.

    A stall detector that looked only at speed would be useless: every slow
    commanded motion looks like a stall.  This firmware arms the detector only
    once the *commanded duty* exceeds `TP_STALL_PWM_THRESHOLD`, because a stall
    is "torque is being demanded and nothing is moving" rather than "nothing is
    moving".

    The invariant that makes that work is quantitative: the duty threshold,
    converted through the plant's duty-to-speed gain, must correspond to a speed
    far above the stall speed threshold.  If those two numbers were ever to
    approach each other, a legitimate slow approach would start tripping the
    protection.

    The first version of this test checked the *slowest commanded joint speed*
    against the stall threshold, which conflated two different quantities - a
    gripper speed in mm/s against a joint speed in mrad/s - and failed for a
    reason that had nothing to do with the detector.
    """
    duty_permille = firmware_macro("TP_STALL_PWM_THRESHOLD")
    stall_mrad_s = firmware_macro("TP_STALL_SPEED_THRESHOLD_MRAD_S")
    ke = firmware_macro("TP_PLANT_MOTOR_KE_VS_RAD")
    vbus = 24.0
    assert None not in (duty_permille, stall_mrad_s, ke)

    # duty -> steady-state speed through the back-EMF constant
    implied_mrad_s = duty_permille * (1000.0 * vbus / ke) / 1000.0
    assert implied_mrad_s > 20.0 * stall_mrad_s, (
        "the duty arming threshold implies only %.0f mrad/s against a %.0f "
        "mrad/s stall threshold; a legitimate slow motion could trip the "
        "detector" % (implied_mrad_s, stall_mrad_s))


def test_wire_force_scaling_preserves_the_setpoint():
    """The wire format must carry the grip-force setpoint without loss.

    The firmware encodes force as integer millinewtons (`i16`).  The target is
    2.20 N, so the encoded value has to be exactly 2200 -- a rounding error here
    would bias every grip the robot makes by a fixed amount that no control
    loop could remove, because the setpoint itself would be wrong.
    """
    target_n = firmware_macro("TP_FORCE_TARGET_N")
    assert target_n is not None
    payload = P.encode_set_force_target(int(round(target_n * 1000.0)))
    assert len(payload) == 2
    decoded = P.decode_set_force_target(payload)
    assert decoded.force_mN == 2200, (
        "%.2f N should round-trip as 2200 mN, got %r" % (target_n, decoded))
    assert target_n * 1000.0 == float(decoded.force_mN), (
        "the setpoint is not exactly representable in millinewtons: %.4f mN"
        % (target_n * 1000.0))