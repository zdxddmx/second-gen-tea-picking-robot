"""
Core configuration: build identity, task rates and actuator limits.

One of the three value tables that make up :mod:simulator.config, split out
purely to keep every module inside the project's line budget.  The values are
unchanged and are re-exported by the package, so `C.JOINT_MAX_MRAD` and friends
keep working exactly as before.

Contents: build/platform identity, task rates and periods, and the physical and
actuator limits (joint travel, jaw travel, PWM, current and slew rate).
"""

from __future__ import annotations

from typing import Dict, List, Tuple


# =============================================================================
# 1. Version / build identification
# =============================================================================

#: Firmware version mirrored by this simulator (TP_FW_VERSION_*).
FW_VERSION: str = "2.0.0"

#: Config header the constants below were copied from.
FIRMWARE_CONFIG_HEADER: str = "firmware/Application/config.h"

# =============================================================================
# 2. Task rates and periods  (TP_*_FREQ_HZ, TP_CONTROL_DT_S)
# =============================================================================

#: Cascade PID + motor PWM update rate, Hz.
CONTROL_FREQ_HZ: int = 1000
#: ADC / force / encoder sampling rate, Hz.
SENSOR_FREQ_HZ: int = 500
#: UART / CAN / RS485 service rate, Hz.
COMM_FREQ_HZ: int = 100
#: Robot FSM step rate, Hz.
ROBOT_FREQ_HZ: int = 100
#: Fault-detection rate, Hz.
SAFETY_FREQ_HZ: int = 100
#: Periodic telemetry upload rate, Hz.
TELEMETRY_FREQ_HZ: int = 20
#: Log flush / console rate, Hz.
DEBUG_FREQ_HZ: int = 5

#: Control loop sample time, s.  This is the integration step of the plant.
CONTROL_DT_S: float = 1.0 / float(CONTROL_FREQ_HZ)
#: Fault-detection sample time, s.
SAFETY_DT_S: float = 1.0 / float(SAFETY_FREQ_HZ)
#: FSM sample time, s.
ROBOT_DT_S: float = 1.0 / float(ROBOT_FREQ_HZ)
#: Host communication service period, s.
COMM_DT_S: float = 1.0 / float(COMM_FREQ_HZ)

# =============================================================================
# 3. Physical and actuator limits
# =============================================================================

# --- Joint (arm) limits: TP_JOINT_* -----------------------------------------
#: Lower soft limit of the joint, mrad (-90 deg).
JOINT_MIN_MRAD: float = -90000.0
#: Upper soft limit of the joint, mrad (+90 deg).
JOINT_MAX_MRAD: float = 90000.0
#: Power-on / homing position, mrad.
JOINT_HOME_MRAD: float = 0.0
#: Maximum joint speed, mrad/s (60 deg/s).
JOINT_MAX_SPEED_MRAD_S: float = 60000.0
#: Maximum joint acceleration implied by the profile, mrad/s^2.
JOINT_MAX_ACCEL_MRAD_S2: float = 200000.0

# --- Gripper (linear jaw) limits: TP_GRIPPER_* ------------------------------
#: Fully closed jaw gap, mm.
GRIPPER_MIN_MM: float = 0.0
#: Fully open jaw gap, mm.
GRIPPER_MAX_MM: float = 40.0
#: Maximum jaw speed, mm/s.
GRIPPER_MAX_SPEED_MM_S: float = 120.0

# --- Electrical / drive limits: TP_MAX_* ------------------------------------
#: |PWM| clamp, per-mille; 1000 == 100 % duty.
MAX_PWM_PERMILLE: float = 1000.0
#: Continuous motor current limit, mA.
MAX_CURRENT_MA: float = 3000.0
#: Short-time peak current (< 50 ms), mA.
PEAK_CURRENT_MA: float = 5000.0
#: PWM slew limit, per-mille per second.
MAX_DUTY_RATE_PER_S: float = 4000.0

#: Continuous current limit expressed in A (derived, not a second source value).
MAX_CURRENT_A: float = MAX_CURRENT_MA / 1000.0
#: Peak current expressed in A (derived).
PEAK_CURRENT_A: float = PEAK_CURRENT_MA / 1000.0
#: Per-tick PWM slew allowance, per-mille per control tick.
MAX_DUTY_STEP_PERMILLE: float = MAX_DUTY_RATE_PER_S * CONTROL_DT_S

