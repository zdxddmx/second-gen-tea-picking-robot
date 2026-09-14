# -*- coding: utf-8 -*-
"""host/protocol.py -- wire protocol codec for the 2nd-gen tea-picking robot.

This module is the single source of truth for the host<->robot serial framing
shared by every PC-side tool (``host/robot_monitor.py``, ``tools/vision_mock.py``,
``tools/analyze_log.py``).

Frame layout (all multi-byte fields are little-endian)::

    +--------+--------+--------+--------+--------------+---------+---------+
    | 0xAA   | 0x55   | CMD    | LEN    | PAYLOAD[LEN] | CRC_LO  | CRC_HI  |
    +--------+--------+--------+--------+--------------+---------+---------+

* ``CRC16`` is CRC-16/MODBUS: polynomial ``0xA001`` (reflected ``0x8005``),
  init ``0xFFFF``, no final XOR, reflected in/out.  It is computed over
  ``CMD | LEN | PAYLOAD`` (sync bytes and CRC itself are excluded) and is
  transmitted low byte first.
* ``LEN`` is the payload length in bytes and must be in ``0..64``.

Run ``python host/protocol.py`` for the built-in self test.

Python 3.8 compatible: no ``match``, no PEP 604 unions, no PEP 585 builtin
generics in annotations, no ``functools.cache``.
"""

import struct
from collections import namedtuple
from typing import Dict, List, Optional, Tuple, Type

__all__ = [
    "SYNC1", "SYNC2", "SYNC", "MAX_PAYLOAD", "HEADER_LEN", "CRC_LEN",
    "crc16_modbus", "crc16_bytes", "build_frame", "Frame", "FrameParser",
    "CMD", "CMD_NAME", "HOST_TO_ROBOT", "ROBOT_TO_HOST",
    "STATE", "STATE_NAMES", "FAULT", "FAULT_NAMES", "ERRCODE", "ERRCODE_NAMES",
    "LOOP_NAMES", "MODE_IDLE", "MODE_AUTO", "MODE_MANUAL", "MODE_CALIBRATE", "MODE_NAMES",
    "encode_payload", "decode_payload", "describe_frame", "payload_size",
]

# --------------------------------------------------------------------------
# Framing constants
# --------------------------------------------------------------------------

SYNC1 = 0xAA
SYNC2 = 0x55
SYNC = b"\xAA\x55"

HEADER_LEN = 4          # SYNC1 + SYNC2 + CMD + LEN
CRC_LEN = 2
MAX_PAYLOAD = 64


# --------------------------------------------------------------------------
# CRC-16/MODBUS
# --------------------------------------------------------------------------

def _build_crc_table():
    # type: () -> List[int]
    table = []  # type: List[int]
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _build_crc_table()


def crc16_modbus(data, crc=0xFFFF):
    # type: (bytes, int) -> int
    """CRC-16/MODBUS over ``data``.

    poly 0xA001 (reflected 0x8005), init 0xFFFF, no final XOR, reflected
    in/out.  ``crc16_modbus(b"123456789") == 0x4B37``.

    ``crc`` continues a previous computation, mirroring the C
    ``tp_crc16_modbus_update()``.  The firmware needs the incremental form
    because it checksums ``CMD | LEN | PAYLOAD``, which are assembled in two
    different places; a host that wants to verify the same frame the same way
    must be able to do it too, and otherwise has to concatenate a copy.
    """
    for byte in bytearray(data):
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def crc16_modbus_update(crc, data):
    # type: (int, bytes) -> int
    """Continue a CRC-16/MODBUS computation.  See :func:`crc16_modbus`."""
    return crc16_modbus(data, crc)


def crc16_bytes(data):
    # type: (bytes) -> bytes
    """CRC-16/MODBUS of ``data`` as the two on-wire bytes (low byte first)."""
    crc = crc16_modbus(data)
    return struct.pack("<H", crc)


# --------------------------------------------------------------------------
# Command codes
# --------------------------------------------------------------------------

class CMD(object):
    """Command byte values (host -> robot and robot -> host)."""

    # host -> robot
    HEARTBEAT = 0x01
    SET_TARGET = 0x10
    START_PICK = 0x11
    STOP = 0x12
    RESET = 0x13
    GET_STATUS = 0x20
    GET_SENSOR = 0x21
    SET_PID = 0x30
    GET_PID = 0x31
    EMERGENCY_STOP = 0x40
    CLEAR_FAULT = 0x41
    SET_MODE = 0x33
    #: Run one step of a force-sensor calibration.  Present on the robot as
    #: 0x42; a monitor that cannot issue it cannot calibrate the load cell.
    CALIBRATE_FORCE = 0x42
    GET_FAULT = 0x22
    SET_FORCE_TARGET = 0x32

    # robot -> host
    ACK = 0x80
    HEARTBEAT_ACK = 0x81
    STATUS_RPT = 0xA0
    SENSOR_RPT = 0xA1
    PID_RPT = 0xA2
    FAULT_RPT = 0xB0
    TELEMETRY = 0xC0
    NACK = 0xE0


CMD_NAME = {
    CMD.HEARTBEAT: "HEARTBEAT",
    CMD.SET_TARGET: "SET_TARGET",
    CMD.START_PICK: "START_PICK",
    CMD.STOP: "STOP",
    CMD.RESET: "RESET",
    CMD.GET_STATUS: "GET_STATUS",
    CMD.GET_SENSOR: "GET_SENSOR",
    CMD.SET_PID: "SET_PID",
    CMD.GET_PID: "GET_PID",
    CMD.EMERGENCY_STOP: "EMERGENCY_STOP",
    CMD.CLEAR_FAULT: "CLEAR_FAULT",
    CMD.SET_MODE: "SET_MODE",
    CMD.CALIBRATE_FORCE: "CALIBRATE_FORCE",
    CMD.GET_FAULT: "GET_FAULT",
    CMD.SET_FORCE_TARGET: "SET_FORCE_TARGET",
    CMD.ACK: "ACK",
    CMD.HEARTBEAT_ACK: "HEARTBEAT_ACK",
    CMD.STATUS_RPT: "STATUS_RPT",
    CMD.SENSOR_RPT: "SENSOR_RPT",
    CMD.PID_RPT: "PID_RPT",
    CMD.FAULT_RPT: "FAULT_RPT",
    CMD.TELEMETRY: "TELEMETRY",
    CMD.NACK: "NACK",
}  # type: Dict[int, str]

HOST_TO_ROBOT = frozenset([
    CMD.HEARTBEAT, CMD.SET_TARGET, CMD.START_PICK, CMD.STOP, CMD.RESET,
    CMD.GET_STATUS, CMD.GET_SENSOR, CMD.SET_PID, CMD.GET_PID,
    CMD.EMERGENCY_STOP, CMD.CLEAR_FAULT, CMD.SET_MODE, CMD.GET_FAULT,
    CMD.SET_FORCE_TARGET,
])

ROBOT_TO_HOST = frozenset([
    CMD.ACK, CMD.HEARTBEAT_ACK, CMD.STATUS_RPT, CMD.SENSOR_RPT, CMD.PID_RPT,
    CMD.FAULT_RPT, CMD.TELEMETRY, CMD.NACK,
])


def cmd_name(cmd):
    # type: (int) -> str
    """Human readable name for a command byte."""
    return CMD_NAME.get(int(cmd) & 0xFF, "UNKNOWN_0x%02X" % (int(cmd) & 0xFF))


# --------------------------------------------------------------------------
# Robot states / fault codes / error codes
# --------------------------------------------------------------------------

class STATE(object):
    INIT = 0
    SELF_CHECK = 1
    IDLE = 2
    TARGET_RECEIVED = 3
    APPROACH = 4
    CONTACT = 5
    GRIP = 6
    PICK = 7
    RELEASE = 8
    RETURN = 9
    ERROR = 10
    EMERGENCY_STOP = 11


STATE_NAMES = {
    STATE.INIT: "INIT",
    STATE.SELF_CHECK: "SELF_CHECK",
    STATE.IDLE: "IDLE",
    STATE.TARGET_RECEIVED: "TARGET_RECEIVED",
    STATE.APPROACH: "APPROACH",
    STATE.CONTACT: "CONTACT",
    STATE.GRIP: "GRIP",
    STATE.PICK: "PICK",
    STATE.RELEASE: "RELEASE",
    STATE.RETURN: "RETURN",
    STATE.ERROR: "ERROR",
    STATE.EMERGENCY_STOP: "EMERGENCY_STOP",
}  # type: Dict[int, str]


class FAULT(object):
    NONE = 0
    FORCE_OVERLOAD = 1
    MOTOR_STALL = 2
    OVERCURRENT = 3
    ENCODER = 4
    COMM_TIMEOUT = 5
    SENSOR = 6
    LIMIT = 7
    ESTOP = 8
    WATCHDOG = 9
    #: The vision target could not be reached or is no longer usable -- the jaws
    #: closed for their full budget without finding the bud, or the target went
    #: stale before the cycle started.  Deliberately *not* reported as SENSOR:
    #: the two send an operator to completely different places.
    TARGET = 10
    #: An internal invariant was violated (a non-finite control output, an FSM
    #: transition with no handler).  Always a firmware defect.
    INTERNAL = 11


#: The table mirrors ``tp_fault_code_t`` in ``firmware/Safety/fault_codes.h``
#: exactly, including the numeric values, because these codes travel on the wire.
#: A mismatch here is the kind of bug that shows a healthy robot as faulted.
FAULT_NAMES = {
    FAULT.NONE: "ERR_NONE",
    FAULT.FORCE_OVERLOAD: "ERR_FORCE_OVERLOAD",
    FAULT.MOTOR_STALL: "ERR_MOTOR_STALL",
    FAULT.OVERCURRENT: "ERR_OVERCURRENT",
    FAULT.ENCODER: "ERR_ENCODER",
    FAULT.COMM_TIMEOUT: "ERR_COMM_TIMEOUT",
    FAULT.SENSOR: "ERR_SENSOR",
    FAULT.LIMIT: "ERR_LIMIT",
    FAULT.ESTOP: "ERR_ESTOP",
    FAULT.WATCHDOG: "ERR_WATCHDOG",
    FAULT.TARGET: "ERR_TARGET",
    FAULT.INTERNAL: "ERR_INTERNAL",
}  # type: Dict[int, str]

#: Severity per code, mirroring ``tp_fault_severity()``.  The monitor uses this
#: to decide whether a fault is worth a red banner or a yellow note.
FAULT_SEVERITY = {
    FAULT.NONE: "info",
    FAULT.FORCE_OVERLOAD: "error",
    FAULT.MOTOR_STALL: "error",
    FAULT.OVERCURRENT: "error",
    FAULT.ENCODER: "critical",
    FAULT.COMM_TIMEOUT: "warning",
    FAULT.SENSOR: "error",
    FAULT.LIMIT: "warning",
    FAULT.ESTOP: "critical",
    FAULT.WATCHDOG: "critical",
    FAULT.TARGET: "error",
    FAULT.INTERNAL: "error",
}  # type: Dict[int, str]


class ERRCODE(object):
    """NACK error codes."""

    OK = 0
    BAD_CRC = 1
    BAD_LEN = 2
    UNKNOWN_CMD = 3
    BAD_PARAM = 4
    BUSY = 5
    NOT_READY = 6


ERRCODE_NAMES = {
    ERRCODE.OK: "OK",
    ERRCODE.BAD_CRC: "BAD_CRC",
    ERRCODE.BAD_LEN: "BAD_LEN",
    ERRCODE.UNKNOWN_CMD: "UNKNOWN_CMD",
    ERRCODE.BAD_PARAM: "BAD_PARAM",
    ERRCODE.BUSY: "BUSY",
    ERRCODE.NOT_READY: "NOT_READY",
}  # type: Dict[int, str]


MODE_IDLE = 0
MODE_AUTO = 1
#: Operating modes, mirroring ``tp_robot_mode_t`` in the firmware.  MANUAL and
#: CALIBRATE are reachable through SET_MODE on the robot, so a monitor that does
#: not know about them reports an unknown number instead of a mode name.
MODE_MANUAL = 2
MODE_CALIBRATE = 3
MODE_NAMES = {MODE_IDLE: "IDLE", MODE_AUTO: "AUTO", MODE_MANUAL: "MANUAL",
              MODE_CALIBRATE: "CALIBRATE"}  # type: Dict[int, str]

#: The three tunable loops, indexed by the ``loop_id`` byte of SET_PID / PID_RPT.
#:
#: These values **must** match ``tp_pid_loop_t`` in
#: ``firmware/Protocol/protocol_types.h``, which is
#: ``{0: POSITION, 1: VELOCITY, 2: FORCE}``.  They did not, at first: this table
#: said ``{0: POSITION, 1: FORCE, 2: GRIPPER}``, so a host tool that sent
#: ``loop_id = 1`` believing it was tuning the *grip force* would have re-tuned
#: the *velocity loop* on the robot -- silently, with a valid CRC, and with the
#: operator watching the numbers on their own screen.  A conformance test now
#: pins the mapping.
PID_LOOP_NAMES = {0: "POSITION", 1: "VELOCITY", 2: "FORCE"}
LOOP_NAMES = PID_LOOP_NAMES


def state_name(code):
    # type: (int) -> str
    return STATE_NAMES.get(int(code), "STATE_%d" % int(code))


def fault_name(code):
    # type: (int) -> str
    return FAULT_NAMES.get(int(code), "FAULT_%d" % int(code))


# --------------------------------------------------------------------------
# Decoded payload containers (namedtuples: field order == struct field order)
# --------------------------------------------------------------------------

Heartbeat = namedtuple("Heartbeat", "seq")
#: ``SET_TARGET`` payload.  The ``reserved`` byte is **not** padding to be
#: dropped: the firmware's ``tp_payload_set_target_t`` declares it explicitly so
#: that ``target_id`` lands on a 16-bit boundary, and it reads ``target_id`` at
#: offset 8.  Omitting the byte here makes the robot read a target_id that is
#: shifted by one byte -- a silent, cross-implementation wire incompatibility of
#: exactly the kind that a conformance test exists to catch, and one that the
#: Python-side self test could not catch because it only ever checked the Python
#: encoder against the Python decoder.
SetTarget = namedtuple("SetTarget",
                       "x_mm y_mm z_mm confidence reserved target_id")
StartPick = namedtuple("StartPick", "target_id")
SetPid = namedtuple("SetPid", "loop_id kp ki kd")
GetPid = namedtuple("GetPid", "loop_id")
SetMode = namedtuple("SetMode", "mode")
SetForceTarget = namedtuple("SetForceTarget", "force_mN")
Ack = namedtuple("Ack", "acked_cmd")
HeartbeatAck = namedtuple("HeartbeatAck", "seq")
StatusRpt = namedtuple(
    "StatusRpt",
    "state fault_code pos_mrad vel_mrad_s force_mN pwm_permille current_ma cycle_count")
SensorRpt = namedtuple(
    "SensorRpt", "pos_mrad vel_mrad_s force_mN adc_raw current_ma temp_c_x10")
PidRpt = namedtuple("PidRpt", "loop_id kp ki kd")
FaultRpt = namedtuple("FaultRpt", "fault_code timestamp_ms detail")
Telemetry = namedtuple(
    "Telemetry", "t_ms pos_mrad vel_mrad_s force_mN pwm_permille state fault")
Nack = namedtuple("Nack", "ref_cmd err_code detail")


# --------------------------------------------------------------------------
# Payload specification table (fmt, field names)
# --------------------------------------------------------------------------

_PAYLOAD_SPEC = {
    CMD.HEARTBEAT: ("<I", ("seq",)),
    CMD.SET_TARGET: ("<hhhBBH", ("x_mm", "y_mm", "z_mm", "confidence", "reserved", "target_id")),
    CMD.START_PICK: ("<H", ("target_id",)),
    CMD.STOP: ("", ()),
    CMD.RESET: ("", ()),
    CMD.GET_STATUS: ("", ()),
    CMD.GET_SENSOR: ("", ()),
    CMD.SET_PID: ("<Bfff", ("loop_id", "kp", "ki", "kd")),
    CMD.GET_PID: ("<B", ("loop_id",)),
    CMD.EMERGENCY_STOP: ("", ()),
    CMD.CLEAR_FAULT: ("", ()),
    CMD.SET_MODE: ("<B", ("mode",)),
    CMD.GET_FAULT: ("", ()),
    CMD.SET_FORCE_TARGET: ("<H", ("force_mN",)),
    CMD.ACK: ("<B", ("acked_cmd",)),
    CMD.HEARTBEAT_ACK: ("<I", ("seq",)),
    CMD.STATUS_RPT: ("<BBihhhhH",
                     ("state", "fault_code", "pos_mrad", "vel_mrad_s",
                      "force_mN", "pwm_permille", "current_ma", "cycle_count")),
    CMD.SENSOR_RPT: ("<ihhHhh",
                     ("pos_mrad", "vel_mrad_s", "force_mN", "adc_raw",
                      "current_ma", "temp_c_x10")),
    CMD.PID_RPT: ("<Bfff", ("loop_id", "kp", "ki", "kd")),
    CMD.FAULT_RPT: ("<BIB", ("fault_code", "timestamp_ms", "detail")),
    CMD.TELEMETRY: ("<IihhhBB",
                    ("t_ms", "pos_mrad", "vel_mrad_s", "force_mN",
                     "pwm_permille", "state", "fault")),
    CMD.NACK: ("<BBB", ("ref_cmd", "err_code", "detail")),
}  # type: Dict[int, Tuple[str, Tuple[str, ...]]]


def payload_format(cmd):
    # type: (int) -> str
    """Return the ``struct`` format string for ``cmd`` (``""`` if none)."""
    spec = _PAYLOAD_SPEC.get(int(cmd))
    if spec is None:
        raise KeyError("no payload spec for cmd 0x%02X" % (int(cmd) & 0xFF))
    return spec[0]


def payload_size(cmd):
    # type: (int) -> int
    """Expected payload length in bytes for ``cmd``."""
    return struct.calcsize(payload_format(cmd))


def _pack(cmd, values):
    # type: (int, tuple) -> bytes
    fmt = payload_format(cmd)
    try:
        return struct.pack(fmt, *values)
    except struct.error as exc:
        raise ValueError("bad payload for %s (%s): %s" % (cmd_name(cmd), fmt, exc))


def _unpack(cmd, payload, cls):
    # type: (int, bytes, Type[tuple]) -> tuple
    fmt = payload_format(cmd)
    expected = struct.calcsize(fmt)
    if len(payload) != expected:
        raise ValueError(
            "%s payload must be %d bytes for fmt %r, got %d"
            % (cmd_name(cmd), expected, fmt, len(payload)))
    return cls(*struct.unpack(fmt, payload))


# --------------------------------------------------------------------------
# Per-command encode helpers
# --------------------------------------------------------------------------

def encode_heartbeat(seq):
    # type: (int) -> bytes
    return struct.pack("<I", int(seq) & 0xFFFFFFFF)


def encode_set_target(x_mm, y_mm, z_mm, confidence, target_id):
    # type: (int, int, int, int, int) -> bytes
    conf = int(confidence)
    if conf < 0 or conf > 100:
        raise ValueError("confidence must be 0..100, got %d" % conf)
    # "<hhhBBH" is 10 bytes: 3 x int16, confidence, an explicit reserved byte, and
    # a uint16 target_id at offset 8, matching tp_payload_set_target_t exactly.
    return struct.pack("<hhhBBH", int(x_mm), int(y_mm), int(z_mm), conf, 0,
                       int(target_id) & 0xFFFF)


def encode_start_pick(target_id):
    # type: (int) -> bytes
    return struct.pack("<H", int(target_id) & 0xFFFF)


def encode_stop():
    # type: () -> bytes
    return b""


def encode_reset():
    # type: () -> bytes
    return b""


def encode_get_status():
    # type: () -> bytes
    return b""


def encode_get_sensor():
    # type: () -> bytes
    return b""


def encode_set_pid(loop_id, kp, ki, kd):
    # type: (int, float, float, float) -> bytes
    return struct.pack("<Bfff", int(loop_id) & 0xFF, float(kp), float(ki), float(kd))


def encode_get_pid(loop_id):
    # type: (int) -> bytes
    return struct.pack("<B", int(loop_id) & 0xFF)


def encode_emergency_stop():
    # type: () -> bytes
    return b""


def encode_clear_fault():
    # type: () -> bytes
    return b""


def encode_set_mode(mode):
    # type: (int) -> bytes
    return struct.pack("<B", int(mode) & 0xFF)


def encode_get_fault():
    # type: () -> bytes
    return b""


def encode_set_force_target(force_mN):
    # type: (int) -> bytes
    return struct.pack("<H", int(force_mN) & 0xFFFF)


def encode_ack(acked_cmd):
    # type: (int) -> bytes
    return struct.pack("<B", int(acked_cmd) & 0xFF)


def encode_heartbeat_ack(seq):
    # type: (int) -> bytes
    return struct.pack("<I", int(seq) & 0xFFFFFFFF)


def encode_status_rpt(state, fault_code, pos_mrad, vel_mrad_s, force_mN,
                      pwm_permille, current_ma, cycle_count):
    # type: (int, int, int, int, int, int, int, int) -> bytes
    return struct.pack("<BBihhhhH", int(state) & 0xFF, int(fault_code) & 0xFF,
                       int(pos_mrad), int(vel_mrad_s), int(force_mN),
                       int(pwm_permille), int(current_ma),
                       int(cycle_count) & 0xFFFF)


def encode_sensor_rpt(pos_mrad, vel_mrad_s, force_mN, adc_raw, current_ma,
                      temp_c_x10):
    # type: (int, int, int, int, int, int) -> bytes
    return struct.pack("<ihhHhh", int(pos_mrad), int(vel_mrad_s), int(force_mN),
                       int(adc_raw) & 0xFFFF, int(current_ma), int(temp_c_x10))


def encode_pid_rpt(loop_id, kp, ki, kd):
    # type: (int, float, float, float) -> bytes
    return struct.pack("<Bfff", int(loop_id) & 0xFF, float(kp), float(ki), float(kd))


def encode_fault_rpt(fault_code, timestamp_ms, detail):
    # type: (int, int, int) -> bytes
    return struct.pack("<BIB", int(fault_code) & 0xFF,
                       int(timestamp_ms) & 0xFFFFFFFF, int(detail) & 0xFF)


def encode_telemetry(t_ms, pos_mrad, vel_mrad_s, force_mN, pwm_permille, state,
                     fault):
    # type: (int, int, int, int, int, int, int) -> bytes
    return struct.pack("<IihhhBB", int(t_ms) & 0xFFFFFFFF, int(pos_mrad),
                       int(vel_mrad_s), int(force_mN), int(pwm_permille),
                       int(state) & 0xFF, int(fault) & 0xFF)


def encode_nack(ref_cmd, err_code, detail=0):
    # type: (int, int, int) -> bytes
    """``NACK`` payload: three bytes, matching ``tp_payload_nack_t``.

    The ``detail`` byte carries command-specific extra information -- the
    rejected payload length, the out-of-range loop id, and so on.  The first
    version of this encoder emitted only two bytes; the firmware's decoder and
    its own three-byte structure were unaffected, but a host that *parses* a NACK
    from the robot would have read the detail byte as part of the CRC or dropped
    its frame.
    """
    return struct.pack("<BBB", int(ref_cmd) & 0xFF, int(err_code) & 0xFF,
                       int(detail) & 0xFF)


# --------------------------------------------------------------------------
# Per-command decode helpers
# --------------------------------------------------------------------------

def decode_heartbeat(payload):
    # type: (bytes) -> Heartbeat
    return _unpack(CMD.HEARTBEAT, payload, Heartbeat)


def decode_set_target(payload):
    # type: (bytes) -> SetTarget
    return _unpack(CMD.SET_TARGET, payload, SetTarget)


def decode_start_pick(payload):
    # type: (bytes) -> StartPick
    return _unpack(CMD.START_PICK, payload, StartPick)


def decode_stop(payload):
    # type: (bytes) -> None
    _check_empty(CMD.STOP, payload)
    return None


def decode_reset(payload):
    # type: (bytes) -> None
    _check_empty(CMD.RESET, payload)
    return None


def decode_get_status(payload):
    # type: (bytes) -> None
    _check_empty(CMD.GET_STATUS, payload)
    return None


def decode_get_sensor(payload):
    # type: (bytes) -> None
    _check_empty(CMD.GET_SENSOR, payload)
    return None


def decode_set_pid(payload):
    # type: (bytes) -> SetPid
    return _unpack(CMD.SET_PID, payload, SetPid)


def decode_get_pid(payload):
    # type: (bytes) -> GetPid
    return _unpack(CMD.GET_PID, payload, GetPid)


def decode_emergency_stop(payload):
    # type: (bytes) -> None
    _check_empty(CMD.EMERGENCY_STOP, payload)
    return None


def decode_clear_fault(payload):
    # type: (bytes) -> None
    _check_empty(CMD.CLEAR_FAULT, payload)
    return None


def decode_set_mode(payload):
    # type: (bytes) -> SetMode
    return _unpack(CMD.SET_MODE, payload, SetMode)


def decode_get_fault(payload):
    # type: (bytes) -> None
    _check_empty(CMD.GET_FAULT, payload)
    return None


def decode_set_force_target(payload):
    # type: (bytes) -> SetForceTarget
    return _unpack(CMD.SET_FORCE_TARGET, payload, SetForceTarget)


def decode_ack(payload):
    # type: (bytes) -> Ack
    return _unpack(CMD.ACK, payload, Ack)


def decode_heartbeat_ack(payload):
    # type: (bytes) -> HeartbeatAck
    return _unpack(CMD.HEARTBEAT_ACK, payload, HeartbeatAck)


def decode_status_rpt(payload):
    # type: (bytes) -> StatusRpt
    return _unpack(CMD.STATUS_RPT, payload, StatusRpt)


def decode_sensor_rpt(payload):
    # type: (bytes) -> SensorRpt
    return _unpack(CMD.SENSOR_RPT, payload, SensorRpt)


def decode_pid_rpt(payload):
    # type: (bytes) -> PidRpt
    return _unpack(CMD.PID_RPT, payload, PidRpt)


def decode_fault_rpt(payload):
    # type: (bytes) -> FaultRpt
    return _unpack(CMD.FAULT_RPT, payload, FaultRpt)


def decode_telemetry(payload):
    # type: (bytes) -> Telemetry
    return _unpack(CMD.TELEMETRY, payload, Telemetry)


def decode_nack(payload):
    # type: (bytes) -> Nack
    return _unpack(CMD.NACK, payload, Nack)


def _check_empty(cmd, payload):
    # type: (int, bytes) -> None
    if len(payload) != 0:
        raise ValueError("%s takes an empty payload, got %d bytes"
                         % (cmd_name(cmd), len(payload)))


# --------------------------------------------------------------------------
# Generic payload helpers (dispatch by command byte)
# --------------------------------------------------------------------------

_DECODERS = {
    CMD.HEARTBEAT: decode_heartbeat,
    CMD.SET_TARGET: decode_set_target,
    CMD.START_PICK: decode_start_pick,
    CMD.STOP: decode_stop,
    CMD.RESET: decode_reset,
    CMD.GET_STATUS: decode_get_status,
    CMD.GET_SENSOR: decode_get_sensor,
    CMD.SET_PID: decode_set_pid,
    CMD.GET_PID: decode_get_pid,
    CMD.EMERGENCY_STOP: decode_emergency_stop,
    CMD.CLEAR_FAULT: decode_clear_fault,
    CMD.SET_MODE: decode_set_mode,
    CMD.GET_FAULT: decode_get_fault,
    CMD.SET_FORCE_TARGET: decode_set_force_target,
    CMD.ACK: decode_ack,
    CMD.HEARTBEAT_ACK: decode_heartbeat_ack,
    CMD.STATUS_RPT: decode_status_rpt,
    CMD.SENSOR_RPT: decode_sensor_rpt,
    CMD.PID_RPT: decode_pid_rpt,
    CMD.FAULT_RPT: decode_fault_rpt,
    CMD.TELEMETRY: decode_telemetry,
    CMD.NACK: decode_nack,
}  # type: Dict[int, object]


def encode_payload(cmd, **kwargs):
    # type: (int, **object) -> bytes
    """Encode a payload for ``cmd`` from keyword arguments."""
    spec = _PAYLOAD_SPEC.get(int(cmd))
    if spec is None:
        raise KeyError("unknown cmd 0x%02X" % (int(cmd) & 0xFF))
    fields = spec[1]
    values = []
    for name in fields:
        if name not in kwargs:
            raise TypeError("%s missing payload field %r" % (cmd_name(cmd), name))
        values.append(kwargs[name])
    return _pack(cmd, tuple(values))


def decode_payload(cmd, payload):
    # type: (int, bytes) -> Optional[dict]
    """Decode ``payload`` into a plain dict, or ``None`` for empty payloads."""
    decoder = _DECODERS.get(int(cmd))
    if decoder is None:
        return None
    value = decoder(payload)
    if value is None:
        return {}
    return dict(value._asdict())


def describe_frame(frame):
    # type: (Frame) -> str
    """One-line human readable summary of a frame (used for logging / stdout)."""
    head = "%-16s" % cmd_name(frame.cmd)
    try:
        decoded = decode_payload(frame.cmd, frame.payload)
    except (ValueError, struct.error) as exc:
        return "%s len=%2d <decode error: %s>" % (head, len(frame.payload), exc)
    if not decoded:
        return "%s len=%2d" % (head, len(frame.payload))
    parts = []
    for key in sorted(decoded.keys()):
        value = decoded[key]
        if key in ("state",):
            parts.append("state=%s(%d)" % (state_name(value), value))
        elif key in ("fault", "fault_code"):
            parts.append("%s=%s" % (key, fault_name(value)))
        elif isinstance(value, float):
            parts.append("%s=%.4f" % (key, value))
        else:
            parts.append("%s=%s" % (key, value))
    return "%s len=%2d %s" % (head, len(frame.payload), " ".join(parts))


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------

def build_frame(cmd, payload=b""):
    # type: (int, bytes) -> bytes
    """Assemble a complete on-wire frame.

    Raises ``ValueError`` if the payload is longer than ``MAX_PAYLOAD`` (64).
    """
    cmd_b = int(cmd) & 0xFF
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long: %d > %d bytes"
                         % (len(payload), MAX_PAYLOAD))
    body = struct.pack("<BB", cmd_b, len(payload)) + payload
    return SYNC + body + crc16_bytes(body)


class Frame(object):
    """A decoded protocol frame."""

    __slots__ = ("cmd", "payload", "raw")

    def __init__(self, cmd, payload, raw=None):
        # type: (int, bytes, Optional[bytes]) -> None
        self.cmd = int(cmd) & 0xFF
        self.payload = bytes(payload)
        if raw is None:
            raw = build_frame(self.cmd, self.payload)
        self.raw = bytes(raw)

    @property
    def name(self):
        # type: () -> str
        return cmd_name(self.cmd)

    def decoded(self):
        # type: () -> Optional[dict]
        return decode_payload(self.cmd, self.payload)

    def __eq__(self, other):
        if not isinstance(other, Frame):
            return NotImplemented
        return (self.cmd == other.cmd and self.payload == other.payload
                and self.raw == other.raw)

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result

    def __hash__(self):
        return hash((self.cmd, self.payload, self.raw))

    def __repr__(self):
        return "Frame(%s, %r)" % (describe_frame(self), self.raw)

    def __str__(self):
        return describe_frame(self)


class FrameParser(object):
    """Streaming, resynchronising frame decoder.

    Feed arbitrary byte chunks with :meth:`feed`; it returns the list of
    complete, CRC-checked frames found.  Garbage is skipped, bad CRCs are
    counted and the parser resynchronises on the next ``0xAA 0x55``.
    """

    def __init__(self):
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_crc_error = 0
        self.bytes_discarded = 0

    # -- counters ---------------------------------------------------------
    @property
    def frames_bad(self):
        # type: () -> int
        return self.frames_crc_error

    @property
    def buffered(self):
        # type: () -> int
        return len(self._buf)

    def reset(self):
        # type: () -> None
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_crc_error = 0
        self.bytes_discarded = 0

    # -- core -------------------------------------------------------------
    def feed(self, data):
        # type: (bytes) -> List[Frame]
        if data:
            self._buf.extend(data)
        return self._drain()

    def _drop(self, count):
        # type: (int) -> None
        if count <= 0:
            return
        del self._buf[:count]
        self.bytes_discarded += count

    def _drain(self):
        # type: () -> List[Frame]
        out = []  # type: List[Frame]
        while True:
            # 1. find the first sync byte
            idx = self._buf.find(SYNC1)
            if idx < 0:
                self._drop(len(self._buf))
                return out
            if idx > 0:
                self._drop(idx)

            # 2. need the full 4 byte header to know the length
            if len(self._buf) < HEADER_LEN:
                return out

            # 3. second sync byte must match (a lone 0xAA is data, not a header)
            if self._buf[1] != SYNC2:
                self._drop(1)
                continue

            length = self._buf[3]
            if length > MAX_PAYLOAD:
                # impossible length -> this 0xAA was not a header
                self._drop(1)
                continue

            total = HEADER_LEN + length + CRC_LEN
            if len(self._buf) < total:
                return out  # wait for more bytes

            body_start = 2
            body_end = HEADER_LEN + length
            crc_calc = crc16_modbus(bytes(self._buf[body_start:body_end]))
            crc_rx = self._buf[body_end] | (self._buf[body_end + 1] << 8)

            if crc_calc != crc_rx:
                self.frames_crc_error += 1
                self._drop(1)  # resync past this candidate header
                continue

            raw = bytes(self._buf[:total])
            frame = Frame(raw[2], raw[HEADER_LEN:body_end], raw)
            del self._buf[:total]
            self.frames_ok += 1
            out.append(frame)


# --------------------------------------------------------------------------
# Self test
# --------------------------------------------------------------------------

def _roundtrip_cases():
    # type: () -> List[Tuple[int, bytes]]
    return [
        (CMD.HEARTBEAT, encode_heartbeat(0xDEADBEEF)),
        (CMD.SET_TARGET, encode_set_target(-123, 45, 180, 92, 0x1234)),
        (CMD.START_PICK, encode_start_pick(7)),
        (CMD.STOP, encode_stop()),
        (CMD.RESET, encode_reset()),
        (CMD.GET_STATUS, encode_get_status()),
        (CMD.GET_SENSOR, encode_get_sensor()),
        (CMD.SET_PID, encode_set_pid(0, 2.5, 0.125, 0.03125)),
        (CMD.GET_PID, encode_get_pid(2)),
        (CMD.EMERGENCY_STOP, encode_emergency_stop()),
        (CMD.CLEAR_FAULT, encode_clear_fault()),
        (CMD.SET_MODE, encode_set_mode(MODE_AUTO)),
        (CMD.GET_FAULT, encode_get_fault()),
        (CMD.SET_FORCE_TARGET, encode_set_force_target(1500)),
        (CMD.ACK, encode_ack(CMD.SET_TARGET)),
        (CMD.HEARTBEAT_ACK, encode_heartbeat_ack(0xDEADBEEF)),
        (CMD.STATUS_RPT, encode_status_rpt(STATE.GRIP, FAULT.NONE, -12000, -350,
                                           1450, 320, 480, 42)),
        (CMD.SENSOR_RPT, encode_sensor_rpt(1200, -33, 900, 2048, 210, 253)),
        (CMD.PID_RPT, encode_pid_rpt(1, 1.5, 0.25, 0.0625)),
        (CMD.FAULT_RPT, encode_fault_rpt(FAULT.FORCE_OVERLOAD, 123456, 9)),
        (CMD.TELEMETRY, encode_telemetry(987654, -12000, -350, 1450, 320,
                                         STATE.CONTACT, FAULT.NONE)),
        (CMD.NACK, encode_nack(CMD.SET_PID, ERRCODE.BAD_PARAM)),
    ]


def _check_crc_vector():
    # type: () -> Tuple[bool, str]
    got = crc16_modbus(b"123456789")
    ok = (got == 0x4B37)
    return ok, "crc16_modbus(b'123456789') = 0x%04X (expected 0x4B37)" % got


def _check_roundtrip():
    # type: () -> Tuple[bool, str]
    parser = FrameParser()
    problems = []  # type: List[str]
    for cmd, payload in _roundtrip_cases():
        wire = build_frame(cmd, payload)
        frames = parser.feed(wire)
        if len(frames) != 1:
            problems.append("%s: parser produced %d frames" % (cmd_name(cmd), len(frames)))
            continue
        frame = frames[0]
        if frame.cmd != cmd or frame.payload != payload or frame.raw != wire:
            problems.append("%s: round-trip mismatch" % cmd_name(cmd))
            continue
        if frame.decoded() is None and frame.cmd in _DECODERS:
            problems.append("%s: decode returned None" % cmd_name(cmd))
    if problems:
        return False, "; ".join(problems)
    return True, "%d commands round-tripped through build_frame/FrameParser" % len(_roundtrip_cases())


def _check_resync_and_crc():
    # type: () -> Tuple[bool, str]
    problems = []  # type: List[str]
    parser = FrameParser()

    # garbage before a good frame
    good = build_frame(CMD.GET_STATUS)
    frames = parser.feed(b"\x00\x11\xAA\x99junk" + good)
    if len(frames) != 1 or frames[0].cmd != CMD.GET_STATUS:
        problems.append("failed to resync after leading garbage")
    if parser.bytes_discarded == 0:
        problems.append("bytes_discarded not counted for garbage")

    # corrupted CRC must be rejected and counted, then recover
    parser = FrameParser()
    bad = bytearray(build_frame(CMD.SET_FORCE_TARGET, encode_set_force_target(1000)))
    bad[-1] ^= 0xFF
    frames = parser.feed(bytes(bad) + good)
    if len(frames) != 1 or frames[0].cmd != CMD.GET_STATUS:
        problems.append("corrupt CRC not rejected / no recovery")
    if parser.frames_crc_error != 1:
        problems.append("frames_crc_error = %d (expected 1)" % parser.frames_crc_error)

    # byte-at-a-time delivery of several frames
    parser = FrameParser()
    stream = b"".join(build_frame(c, p) for c, p in _roundtrip_cases())
    collected = []  # type: List[Frame]
    for i in range(len(stream)):
        collected.extend(parser.feed(stream[i:i + 1]))
    if len(collected) != len(_roundtrip_cases()):
        problems.append("byte-at-a-time: got %d/%d frames"
                        % (len(collected), len(_roundtrip_cases())))
    if parser.bytes_discarded != 0:
        problems.append("byte-at-a-time discarded %d bytes" % parser.bytes_discarded)

    # oversize payload rejection
    try:
        build_frame(CMD.TELEMETRY, bytes(65))
        problems.append("build_frame accepted a 65 byte payload")
    except ValueError:
        pass

    if problems:
        return False, "; ".join(problems)
    return True, "resync on garbage, CRC rejection, byte-wise streaming, length limit"


def _check_payload_sizes():
    # type: () -> Tuple[bool, str]
    expected = {
        CMD.HEARTBEAT: 4,
        CMD.SET_TARGET: 10,
        CMD.START_PICK: 2,
        CMD.STOP: 0,
        CMD.RESET: 0,
        CMD.GET_STATUS: 0,
        CMD.GET_SENSOR: 0,
        CMD.SET_PID: 13,
        CMD.GET_PID: 1,
        CMD.EMERGENCY_STOP: 0,
        CMD.CLEAR_FAULT: 0,
        CMD.SET_MODE: 1,
        CMD.GET_FAULT: 0,
        CMD.SET_FORCE_TARGET: 2,
        CMD.ACK: 1,
        CMD.HEARTBEAT_ACK: 4,
        CMD.STATUS_RPT: 16,
        CMD.SENSOR_RPT: 14,
        CMD.PID_RPT: 13,
        CMD.FAULT_RPT: 6,
        CMD.TELEMETRY: 16,
        CMD.NACK: 3,
    }  # type: Dict[int, int]
    problems = []  # type: List[str]
    for cmd in sorted(expected):
        got = payload_size(cmd)
        if got != expected[cmd]:
            problems.append("%s: %d != %d" % (cmd_name(cmd), got, expected[cmd]))
    if problems:
        return False, "; ".join(problems)
    return True, "all %d payload sizes match the specification" % len(expected)


def selftest(verbose=True):
    # type: (bool) -> bool
    """Run the protocol self test, printing PASS/FAIL lines.  Returns success."""
    checks = [
        ("CRC-16/MODBUS vector", _check_crc_vector),
        ("frame round-trip", _check_roundtrip),
        ("resync / CRC reject / streaming", _check_resync_and_crc),
        ("payload sizes", _check_payload_sizes),
    ]
    all_ok = True
    for label, fn in checks:
        try:
            ok, detail = fn()
        except Exception as exc:  # noqa: BLE001 - self test must report, not crash
            ok, detail = False, "%s: %s" % (type(exc).__name__, exc)
        all_ok = all_ok and ok
        if verbose:
            print("[%s] %-34s %s" % ("PASS" if ok else "FAIL", label, detail))
    return all_ok


if __name__ == "__main__":
    import sys
    print("host/protocol.py self test")
    print("-" * 72)
    _ok = selftest(verbose=True)
    print("-" * 72)
    print("RESULT: %s" % ("PASS" if _ok else "FAIL"))
    sys.exit(0 if _ok else 1)
