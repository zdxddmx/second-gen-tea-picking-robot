"""
Central configuration for the tea-picking robot simulator.

Every tunable number used by the simulator lives in this package.  The values are
a direct, number-for-number mirror of the C firmware header
`firmware/Application/config.h` (firmware version 2.0.0), so that a simulation
result can be compared against bench measurements without any unit conversion or
mental re-scaling.

The tables are split by domain purely to keep every module inside the project's
line budget, and are re-exported here, so `from . import config as C` followed by
`C.FORCE_TARGET_N` behaves exactly as it did when this was a single module:

`_core`
    Build identity, task rates and periods, physical and actuator limits.
`_control`
    PID gain tables, flexible gripping, communication, safety, sensor conditioning.
`_plant`
    Motor and stem physics, damage threshold, scenario defaults.

Why one authoritative table
---------------------------
If the simulator kept its own private gains the two implementations would drift
apart silently and the simulator would stop being evidence about the firmware.
Keeping the values in one place also lets a reviewer diff them against
`config.h` mechanically.

What happens without it
-----------------------
Magic numbers would be scattered through the plant and controller; changing the
grip force target would require editing several files, and a mismatch with the
firmware would only be discovered on hardware.

Unit convention (identical to the firmware, see docs/units_convention.md)
------------------------------------------------------------------------
=========================  ====================  ==========================
Quantity                   Internal unit         Wire unit
=========================  ====================  ==========================
Joint angle                mrad (float)          mrad (int32)
Joint velocity             mrad/s (float)        mrad/s (int16)
Gripper jaw gap            mm (float)            um (uint16)
Force                      N (float)             mN (int16)
Motor current              A  (float)            mA (int16)
PWM duty                   per-mille (-1000..1000)  per-mille (int16)
Cartesian target           mm (float)            mm (int16)
=========================  ====================  ==========================

Note the deliberate difference from the firmware: the firmware stores current in
**mA** (`TP_MAX_CURRENT_MA`) because its ADC reports milliamps, while the plant
model integrates in **A** because the torque constant `Kt` is N*m/A.
`MAX_CURRENT_A` is therefore derived from the mirrored mA constant rather than
being a second, independent number.

Three values deliberately differ from the firmware, each documented in place next
to the firmware value it replaces: `FORCE_PID_KI`, `DAMAGE_FORCE_N` and the
force-loop law constants in `_control`.
"""

from __future__ import annotations

from typing import List

from ._core import *  # noqa: F401,F403 - re-exported value table
from ._control import *  # noqa: F401,F403 - re-exported value table
from ._plant import *  # noqa: F401,F403 - re-exported value table

__all__ = [
    name
    for name in dir()
    if name.isupper() or name in ('scenario_title', 'validate')
]



def validate() -> List[str]:
    """Re-check the firmware's compile-time consistency assertions in Python.

    The C header enforces these with ``TP_STATIC_ASSERT``.  Reproducing them
    here means a bad edit to :mod:`simulator.config` fails immediately and
    loudly in the simulator rather than producing plausible-looking but
    physically meaningless results.

    Returns
    -------
    List[str]
        Human-readable descriptions of every violated invariant; empty when the
        configuration is consistent.
    """
    problems: List[str] = []
    rates = [
        ("SENSOR_FREQ_HZ", SENSOR_FREQ_HZ),
        ("COMM_FREQ_HZ", COMM_FREQ_HZ),
        ("ROBOT_FREQ_HZ", ROBOT_FREQ_HZ),
        ("SAFETY_FREQ_HZ", SAFETY_FREQ_HZ),
        ("TELEMETRY_FREQ_HZ", TELEMETRY_FREQ_HZ),
    ]
    for name, hz in rates:
        if hz <= 0 or (CONTROL_FREQ_HZ % hz) != 0:
            problems.append(
                "{0}={1} must divide CONTROL_FREQ_HZ={2}".format(name, hz, CONTROL_FREQ_HZ)
            )
    if not (FORCE_TARGET_N < FORCE_SOFT_LIMIT_N < FORCE_MAX_N):
        problems.append("require FORCE_TARGET_N < FORCE_SOFT_LIMIT_N < FORCE_MAX_N")
    if not (FORCE_TARGET_MIN_N <= FORCE_TARGET_N <= FORCE_TARGET_MAX_N):
        problems.append("FORCE_TARGET_N must lie within [MIN, MAX]")
    if JOINT_MAX_SPEED_MRAD_S > POS_PID_OUT_MAX:
        problems.append("POS_PID_OUT_MAX must allow the full joint speed range")
    if not (GRIPPER_MIN_MM <= STEM_GAP_MM <= GRIPPER_MAX_MM):
        problems.append("STEM_GAP_MM must lie inside the jaw travel")
    if CONTROL_DT_S <= 0.0:
        problems.append("CONTROL_DT_S must be positive")
    if PROTO_MAX_FRAME > 128:
        problems.append("PROTO_MAX_FRAME must stay below the RX ring size")
    return problems
