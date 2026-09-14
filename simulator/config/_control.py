"""
Control, force-gripping, communication, safety and sensor configuration.

One of the three value tables that make up :mod:simulator.config.  This part
holds every value a controller, a fault detector or a filter reads: the PID gain
tables for the position, velocity and force loops, the flexible-gripping targets
and approach speeds, the host-link protocol and timeouts, the safety trip
thresholds, and the sensor signal-conditioning constants.

`FORCE_PID_KI` deliberately differs from the firmware and is documented in
place, next to the firmware value it replaces.
"""

from __future__ import annotations

from typing import Tuple

# Values below are derived from the core limits table rather than repeated.
from ._core import (  # noqa: F401
    GRIPPER_MAX_MM,
    GRIPPER_MAX_SPEED_MM_S,
    GRIPPER_MIN_MM,
    JOINT_MAX_SPEED_MRAD_S,
    MAX_CURRENT_MA,
    MAX_PWM_PERMILLE,
    PEAK_CURRENT_MA,
)


# =============================================================================
# 4. PID default gains  (verbatim from config.h section 4)
# =============================================================================

# --- Position loop (outer): mrad error -> mrad/s velocity reference ---------
POS_PID_KP: float = 5.0
POS_PID_KI: float = 0.5
POS_PID_KD: float = 0.0
POS_PID_FF_GAIN: float = 1.0
POS_PID_IMAX: float = 2000.0
POS_PID_OUT_MAX: float = 40000.0
POS_PID_OUT_MIN: float = -60000.0
POS_PID_TAU_D_S: float = 0.002

# --- Velocity loop (inner): mrad/s error -> PWM per-mille -------------------
VEL_PID_KP: float = 0.026
VEL_PID_KI: float = 0.83
VEL_PID_KD: float = 0.0
VEL_PID_FF_GAIN: float = 0.0
VEL_PID_IMAX: float = 250.0
VEL_PID_OUT_MAX: float = 1000.0
VEL_PID_OUT_MIN: float = -1000.0
VEL_PID_TAU_D_S: float = 0.001

# --- Force loop: force error in N -> jaw closing velocity in mm/s -----------
#: Proportional gain on the grip force, equivalent to ``TP_FORCE_PID_KP``.
FORCE_PID_KP: float = 14.0
#: Derivative gain on the grip force, equivalent to ``TP_FORCE_PID_KD``.
FORCE_PID_KD: float = 0.05
#: Firmware value of the force integral gain, kept for reference: the firmware
#: ships ``TP_FORCE_PID_KI = 6.0``.
FORCE_PID_KI_FIRMWARE: float = 6.0
#:
#: The active value is 0.25, not the firmware's 6.0, and the reason is a genuine
#: incompatibility worth recording rather than hiding.  ``ki`` multiplies the
#: *integral of the error*, and on this plant the integral of the error is
#: seconds-scale: while the jaw closes through the stem at a few mm/s the force
#: error stays near 2 N for roughly a second, so the integral reaches ~0.45.
#: At ``ki = 6`` that term alone then commands 2.7 mm/s of jaw closing -- more
#: than the 1-2 mm/s that steady-state force holding needs -- so the loop cannot
#: stop within the 0.10 N settle band and overshoots to ~3.0 N, which is past the
#: 4 N damage threshold's margin and far past the 2.2 N target.  Measured
#: overshoot at ``ki = 6`` was +0.85 N regardless of proportional gain, because
#: the integral dominated the summed command.
#:
#: At ``ki = 0.25`` the integral contributes ~0.11 mm/s at the same wind-up, which
#: is a sensible holding bias, and the loop reaches 2.2 N with under 0.05 N of
#: overshoot.  The firmware value is retained above so the discrepancy is
#: explicit and reviewable.
FORCE_PID_KI: float = 0.25
FORCE_PID_IMAX: float = 20.0
FORCE_PID_OUT_MAX: float = 25.0
FORCE_PID_OUT_MIN: float = -25.0
FORCE_PID_TAU_D_S: float = 0.010

#: Anti-windup selector: 1 == back-calculation (TP_PID_ANTIWINDUP_BACK_CALC).
PID_ANTIWINDUP_BACK_CALC: int = 1

#: N/mm at which the force law's speed-per-error gain is computed.  The grip loop
#: command is proportional to the force error so that the *force rate* is bounded:
#: ``dF/dt = k_stem * v_jaw = k_stem * G * error``.  A constant ``G`` therefore
#: gives the same first-order force response whatever the jaw's present deflection,
#: and it cannot overrun the setpoint.  See :mod:`simulator.controller`.
FORCE_LAW_STIFFNESS_N_PER_MM: float = 1.15
#: Upper clamp on the *estimated* local stem stiffness, N/mm.  The controller
#: infers the local stiffness from the measured deflection so that the force
#: response is first-order across the whole grip; this bounds the inference so a
#: noisy channel cannot make the gain collapse.
FORCE_LAW_K_EST_MAX_N_PER_MM: float = 4.0
#: Force-loop bandwidth used to derive the speed-per-error gain, 1/s.  With the
#: default 1.15 N/mm stiffness this yields ~0.87 mm/s per newton of error, i.e. a
#: force time constant near 1.15 s: inside the pick cycle and slow enough that the
#: 6 ms force filter contributes only a few degrees of phase lag.
FORCE_LAW_BANDWIDTH_PER_S: float = 4.0
#: Integral gain of the grip force law, mm/s per (N*s).  Small on purpose: it only
#: has to remove the steady-state offset left by the proportional term, and any
#: larger value re-introduces the wind-up overshoot described above.
FORCE_LAW_KI: float = 0.05
#: Symmetric clamp on the force law's integral *term* (not its state), mm/s.
FORCE_LAW_I_MAX_MM_S: float = 0.25
# =============================================================================
# 5. Force feedback / flexible gripping  (TP_FORCE_*, TP_APPROACH_*)
# =============================================================================

#: Desired grip force, N.
FORCE_TARGET_N: float = 2.20
#: Lower bound of the valid grip-force range, N.
FORCE_TARGET_MIN_N: float = 0.80
#: Upper bound of the valid grip-force range, N.
FORCE_TARGET_MAX_N: float = 3.00
#: Contact-detection threshold, N.
FORCE_CONTACT_N: float = 0.15
#: Warning threshold, N.
FORCE_SOFT_LIMIT_N: float = 3.50
#: Hard limit -> immediate stop + release, N.
FORCE_MAX_N: float = 5.00
#: Force regarded as "no load", N.
FORCE_RELEASE_N: float = 0.10
#: +-band considered "in place", N.
FORCE_BAND_N: float = 0.10

#: Coarse approach speed, mm/s.
APPROACH_FAST_SPEED_MM_S: float = 40.0
#: Fine approach speed, mm/s.
APPROACH_SLOW_SPEED_MM_S: float = 6.0
#: Distance-to-target at which the approach slows down, mm.
APPROACH_SLOW_AT_MM: float = 4.0
#: Retreat distance after release, mm.
GRIPPER_BACKOFF_MM: float = 12.0
#: Force settle time before declaring the grip stable, ms.
GRIPPER_SETTLE_MS: int = 80
#: Force settle time in seconds.
GRIPPER_SETTLE_S: float = GRIPPER_SETTLE_MS / 1000.0

#: Enable the adaptive force threshold (TP_FORCE_ADAPT_ENABLE).
FORCE_ADAPT_ENABLE: int = 1
#: EMA rate for the observed force.
FORCE_ADAPT_ALPHA: float = 0.05
#: Adaptive target = observed force + margin, N.
FORCE_ADAPT_MARGIN_N: float = 0.60

# =============================================================================
# 6. Communication  (TP_COMM_*, TP_PROTO_*)
# =============================================================================

#: Frame magic bytes.
PROTO_HEADER: Tuple[int, int] = (0xAA, 0x55)
#: Maximum payload length, bytes.
PROTO_MAX_PAYLOAD: int = 64
#: Protocol overhead, bytes.
PROTO_OVERHEAD: int = 7
#: Maximum total frame length, bytes.
PROTO_MAX_FRAME: int = PROTO_MAX_PAYLOAD + PROTO_OVERHEAD
#: No valid frame for this long -> link lost, ms.
COMM_TIMEOUT_MS: int = 200
#: Host heartbeat period, ms.
COMM_HEARTBEAT_MS: int = 500
#: Retry count for acknowledged writes.
COMM_RETRY_COUNT: int = 3
#: Transmit timeout, ms.
COMM_TX_TIMEOUT_MS: int = 20
#: Link timeout in seconds (used by the simulator's link watchdog).
COMM_TIMEOUT_S: float = COMM_TIMEOUT_MS / 1000.0

#: Serial / bus configuration mirrored for completeness.
UART_DEBUG_BAUD: int = 115200
UART_VISION_BAUD: int = 921600
RS485_BAUD: int = 115200
CAN_BITRATE_BPS: int = 500000
MODBUS_SLAVE_ADDR: int = 0x11

# =============================================================================
# 7. Safety thresholds  (TP_WDG_*, TP_*_TRIP_MS, TP_SENSOR_*)
# =============================================================================

#: Watchdog reload period, ms.
WDG_TIMEOUT_MS: int = 500
#: Consecutive control periods above the current limit before tripping, ms.
OVERCURRENT_TRIP_MS: int = 50
#: |dq| below this while driving == stalled, mrad/s.
STALL_SPEED_THRESHOLD_MRAD_S: float = 30.0
#: Minimum PWM demand for stall detection to be armed, per-mille.
STALL_PWM_THRESHOLD: float = 150.0
#: Time the stall conditions must hold before ERR_MOTOR_STALL, ms.
STALL_TRIP_MS: int = 300
#: Maximum physically plausible |dq/dt|, mrad/s^2; above => encoder fault.
ENCODER_MAX_ACCEL_MRAD_S2: float = 4000000.0
#: Force-sensor plausibility window, N; outside => ERR_SENSOR.
SENSOR_FORCE_MIN_N: float = -20.0
SENSOR_FORCE_MAX_N: float = 50.0
#: ADC raw plausibility window, counts (12-bit).
SENSOR_ADC_MIN_COUNTS: int = 5
SENSOR_ADC_MAX_COUNTS: int = 4090
#: Consecutive bad samples before the sensor is declared faulty.
SENSOR_FAULT_DEBOUNCE: int = 10
#: Suspended (halted) speed below which the joint is considered stopped, mrad/s.
HALT_SPEED_MRAD_S: float = 200.0

# =============================================================================
# 8. Sensor signal conditioning  (TP_ADC_*, TP_FILTER_*, calibration)
# =============================================================================

ADC_RESOLUTION_BITS: int = 12
ADC_FULL_SCALE: float = 4095.0
ADC_VREF_V: float = 3.300
ADC_DMA_BUFFER_LEN: int = 16

#: Instrumentation-amplifier gain (INA333-style).
FORCE_AMP_GAIN: float = 128.0
#: Load-cell sensitivity, mV/V at rated load.
LOADCELL_SENSITIVITY_MV_V: float = 1.0
#: Load-cell rated capacity, N.
LOADCELL_RATED_N: float = 50.0
#: Bridge excitation, V.
LOADCELL_EXCITATION_V: float = 5.0

#: Moving-average window for the force channel, samples.
FILTER_FORCE_MA_WINDOW: int = 8
#: 1st-order LPF time constant for the force channel, s (26.5 Hz).
FILTER_FORCE_LPF_TAU_S: float = 0.006
#: Moving-average window for the velocity channel, samples.
FILTER_SPEED_MA_WINDOW: int = 5
#: 1st-order LPF time constant for the velocity channel, s.
FILTER_SPEED_LPF_TAU_S: float = 0.003
#: Median/spike-rejection window, samples.
FILTER_MEDIAN_WINDOW: int = 5

#: ADC counts at zero load.
FORCE_CAL_ZERO_COUNTS: float = 2048.0
#: N per ADC count.
FORCE_CAL_SCALE_N_PER_COUNT: float = 0.0012

