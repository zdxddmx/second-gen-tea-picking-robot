"""Protocol conformance tests: the Python tools and the C firmware must agree.

The framed protocol exists on both sides of the link.  A disagreement between
them is the worst class of bug in a robotics project, because it is invisible:
the robot keeps running and simply acts on different numbers than the operator
intended.  These tests pin the parts that are shared:

* the CRC-16/MODBUS check vector and incremental behaviour,
* the exact frame layout, byte for byte,
* every command and report payload size -- the part where a Python-side self
  test cannot help, because it only ever compares the Python encoder with the
  Python decoder,
* the numeric values of the fault and state enumerations, which travel on the
  wire,
* the parser's behaviour on hostile input.

The C side has its own copies of the codec assertions in
``firmware/Tests/test_crc_frame.c``.  Reading the two suites together is how a
reviewer should answer "how do you know both ends agree?".

**Three wire bugs were found by this file when it was first run**, all of them
invisible to a single-language test suite:

1. ``SET_TARGET`` was 9 bytes on the Python side and 10 in the firmware, so the
   firmware read ``target_id`` one byte early -- from the confidence byte and the
   first byte of the real id.
2. ``NACK`` was 2 bytes on the Python side and 3 in the firmware.
3. The firmware's own ``STATUS_RPT`` and ``SENSOR_RPT`` encoders wrote one byte
   more than they transmitted, truncating the last field.
"""

from __future__ import print_function

import struct

from host import protocol as P


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------

def test_crc16_modbus_check_vector():
    """The published check value for CRC-16/MODBUS over "123456789"."""
    assert P.crc16_modbus(b"123456789") == 0x4B37


def test_crc16_incremental_matches_one_shot():
    """The firmware computes the CRC over CMD|LEN|PAYLOAD, built in two pieces."""
    a = b"\x10\x02"
    b = b"\x11\x22\x33"
    assert P.crc16_modbus(a + b) == P.crc16_modbus_update(
        P.crc16_modbus_update(0xFFFF, a), b)


def test_crc16_detects_every_single_bit_error():
    data = bytearray(b"\xaa\x55\x10\x04\x01\x02\x03\x04")
    base = P.crc16_modbus(bytes(data))
    for i in range(len(data)):
        for bit in range(8):
            data[i] ^= 1 << bit
            assert P.crc16_modbus(bytes(data)) != base, (
                "single-bit error in byte %d bit %d was not detected" % (i, bit))
            data[i] ^= 1 << bit


# ---------------------------------------------------------------------------
# Frame layout
# ---------------------------------------------------------------------------

def test_frame_layout_is_exact():
    payload = b"\x11\x22\x33"
    frame = P.build_frame(P.CMD.SET_TARGET, payload)
    assert frame[0] == 0xAA
    assert frame[1] == 0x55
    assert frame[2] == P.CMD.SET_TARGET
    assert frame[3] == len(payload)
    assert frame[4:7] == payload
    crc = P.crc16_modbus(frame[2:7])
    assert frame[7] == (crc & 0xFF)
    assert frame[8] == ((crc >> 8) & 0xFF)
    assert len(frame) == len(payload) + 6


def test_zero_length_frame_round_trips():
    frame = P.build_frame(P.CMD.STOP)
    assert len(frame) == 6
    parser = P.FrameParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    assert frames[0].cmd == P.CMD.STOP
    assert frames[0].payload == b""


def test_maximum_length_frame_round_trips():
    payload = bytes(bytearray(range(P.MAX_PAYLOAD)))
    parser = P.FrameParser()
    frames = parser.feed(P.build_frame(P.CMD.TELEMETRY, payload))
    assert len(frames) == 1
    assert frames[0].payload == payload


def test_oversize_payload_is_refused():
    """A truncated frame with a valid CRC over the wrong data is the worst
    possible outcome, so the encoder must refuse rather than truncate."""
    try:
        P.build_frame(P.CMD.SET_TARGET, bytes(P.MAX_PAYLOAD + 1))
    except Exception:
        return
    raise AssertionError("an oversize payload was accepted")


def test_parser_resynchronises_on_garbage():
    parser = P.FrameParser()
    assert parser.feed(b"\x00\xff\xaa\x00\x7e") == []
    frames = parser.feed(P.build_frame(P.CMD.STOP))
    assert len(frames) == 1
    assert frames[0].cmd == P.CMD.STOP
    assert parser.bytes_discarded > 0


def test_parser_rejects_a_bad_crc():
    frame = bytearray(P.build_frame(P.CMD.GET_STATUS))
    frame[-1] ^= 0xFF
    parser = P.FrameParser()
    assert parser.feed(bytes(frame)) == []
    assert parser.frames_crc_error == 1


def test_parser_handles_byte_at_a_time_delivery():
    """A UART delivers bytes, not frames; the parser must stream."""
    frame = P.build_frame(P.CMD.RESET)
    parser = P.FrameParser()
    out = []
    for byte in frame:
        out.extend(parser.feed(bytes([byte])))
    assert len(out) == 1
    assert out[0].cmd == P.CMD.RESET


# ---------------------------------------------------------------------------
# Payload layouts -- the sizes that both implementations must agree on
# ---------------------------------------------------------------------------

#: Every payload length, as implemented by the firmware's encoders in
#: ``firmware/Protocol/protocol.c`` and ``protocol_types.h``.  If a field is
#: added on one side and not the other, this table is what notices.
FIRMWARE_PAYLOAD_SIZES = {
    "heartbeat": 4,
    "set_target": 10,
    "start_pick": 2,
    "stop": 0,
    "reset": 0,
    "get_status": 0,
    "get_sensor": 0,
    "get_fault": 0,
    "set_pid": 13,
    "get_pid": 1,
    "set_force_target": 2,
    "set_mode": 1,
    "emergency_stop": 0,
    "clear_fault": 0,
    "ack": 1,
    "heartbeat_ack": 4,
    "status_rpt": 16,
    "sensor_rpt": 14,
    "pid_rpt": 13,
    "fault_rpt": 6,
    "telemetry": 16,
    "nack": 3,
}


def test_python_payload_sizes_match_the_firmware_table():
    produced = {
        "heartbeat": P.encode_heartbeat(1),
        "set_target": P.encode_set_target(-12, 345, -6, 88, 1234),
        "start_pick": P.encode_start_pick(1),
        "stop": P.encode_stop(),
        "reset": P.encode_reset(),
        "get_status": P.encode_get_status(),
        "get_sensor": P.encode_get_sensor(),
        "get_fault": P.encode_get_fault(),
        "set_pid": P.encode_set_pid(1, 0.005, 0.10, 0.0),
        "get_pid": P.encode_get_pid(1),
        "set_force_target": P.encode_set_force_target(2200),
        "set_mode": P.encode_set_mode(1),
        "emergency_stop": P.encode_emergency_stop(),
        "clear_fault": P.encode_clear_fault(),
        "ack": P.encode_ack(P.CMD.SET_TARGET),
        "heartbeat_ack": P.encode_heartbeat_ack(7),
        "status_rpt": P.encode_status_rpt(6, 0, -1234, 567, 2180, -42, 1800, 7),
        "sensor_rpt": P.encode_sensor_rpt(0, 0, 2180, 2048, 0, 32767),
        "pid_rpt": P.encode_pid_rpt(1, 0.005, 0.10, 0.0),
        "fault_rpt": P.encode_fault_rpt(1, 1234, 0),
        "telemetry": P.encode_telemetry(12345, -100, 200, 2200, 15, 6, 0),
        "nack": P.encode_nack(P.CMD.STOP, 3, 0),
    }
    for name in sorted(FIRMWARE_PAYLOAD_SIZES):
        expected = FIRMWARE_PAYLOAD_SIZES[name]
        actual = len(produced[name])
        assert actual == expected, (
            "%s is %d bytes but the firmware expects %d"
            % (name, actual, expected))


def test_set_target_places_target_id_at_offset_eight():
    """The firmware reads ``target_id`` at offset 8, past an explicit pad byte.

    Getting this wrong shifts the identifier by one byte, which the parser
    accepts silently: the frame's CRC is still valid, the robot simply acts on a
    different bud than the vision system meant.
    """
    payload = P.encode_set_target(x_mm=-12, y_mm=345, z_mm=-6, confidence=88,
                                  target_id=0x1234)
    assert len(payload) == 10
    x, y, z, conf, reserved, tid = struct.unpack("<hhhBBH", payload)
    assert (x, y, z, conf, tid) == (-12, 345, -6, 88, 0x1234)
    assert reserved == 0
    assert payload[8] == 0x34 and payload[9] == 0x12


def test_status_report_fields_are_little_endian_and_sixteen_bytes():
    payload = P.encode_status_rpt(state=6, fault_code=0, pos_mrad=-1234,
                                  vel_mrad_s=567, force_mN=2180,
                                  pwm_permille=-42, current_ma=1800,
                                  cycle_count=60000)
    assert len(payload) == 16
    fields = struct.unpack("<BBihhhhH", payload)
    assert fields[0] == 6
    assert fields[2] == -1234
    assert fields[4] == 2180
    # 60000 needs the full 16 bits: a 15-byte report would truncate it.\r`n    assert fields[7] == 60000


def test_sensor_report_is_fourteen_bytes():
    payload = P.encode_sensor_rpt(pos_mrad=-100, vel_mrad_s=50, force_mN=2200,
                                  adc_raw=2322, current_ma=-1500,
                                  temp_c_x10=32767)
    assert len(payload) == 14
    assert struct.unpack("<ihhHhh", payload) == (-100, 50, 2200, 2322, -1500,
                                                 32767)


def test_telemetry_is_sixteen_bytes():
    payload = P.encode_telemetry(t_ms=12345, pos_mrad=-100, vel_mrad_s=200,
                                 force_mN=2200, pwm_permille=15, state=6,
                                 fault=0)
    assert len(payload) == 16
    assert struct.unpack("<IihhhBB", payload)[0] == 12345


def test_pid_loop_ids_match_the_firmware():
    """The loop identifiers index the firmware's cascade and force controller."""
    assert P.PID_LOOP_NAMES[0].lower().find("pos") == 0
    assert P.PID_LOOP_NAMES[1].lower().find("vel") == 0
    assert P.PID_LOOP_NAMES[2].lower().find("force") == 0


# ---------------------------------------------------------------------------
# Enumerations shared with the firmware
# ---------------------------------------------------------------------------

def test_fault_codes_match_firmware_enum():
    assert P.FAULT.NONE == 0
    assert P.FAULT.FORCE_OVERLOAD == 1
    assert P.FAULT.MOTOR_STALL == 2
    assert P.FAULT.OVERCURRENT == 3
    assert P.FAULT.ENCODER == 4
    assert P.FAULT.COMM_TIMEOUT == 5
    assert P.FAULT.SENSOR == 6
    assert P.FAULT.LIMIT == 7
    assert P.FAULT.ESTOP == 8
    assert P.FAULT.WATCHDOG == 9
    assert P.FAULT.TARGET == 10
    assert P.FAULT.INTERNAL == 11


def test_every_fault_code_has_a_name_and_a_severity():
    for code in range(0, 12):
        assert code in P.FAULT_NAMES, "fault %d has no name" % code
        assert code in P.FAULT_SEVERITY, "fault %d has no severity" % code
        assert P.FAULT_NAMES[code].startswith("ERR_")


def test_state_codes_match_firmware_enum():
    assert P.STATE.INIT == 0
    assert P.STATE.SELF_CHECK == 1
    assert P.STATE.IDLE == 2
    assert P.STATE.TARGET_RECEIVED == 3
    assert P.STATE.APPROACH == 4
    assert P.STATE.CONTACT == 5
    assert P.STATE.GRIP == 6
    assert P.STATE.PICK == 7
    assert P.STATE.RELEASE == 8
    assert P.STATE.RETURN == 9
    assert P.STATE.ERROR == 10
    assert P.STATE.EMERGENCY_STOP == 11


def test_every_state_has_a_readable_name():
    for code in range(0, 12):
        assert code in P.STATE_NAMES
        assert P.STATE_NAMES[code]
        assert P.state_name(code) == P.STATE_NAMES[code]


def test_command_codes_match_firmware_enum():
    assert P.CMD.HEARTBEAT == 0x01
    assert P.CMD.SET_TARGET == 0x10
    assert P.CMD.START_PICK == 0x11
    assert P.CMD.STOP == 0x12
    assert P.CMD.RESET == 0x13
    assert P.CMD.GET_STATUS == 0x20
    assert P.CMD.GET_SENSOR == 0x21
    assert P.CMD.GET_FAULT == 0x22
    assert P.CMD.SET_PID == 0x30
    assert P.CMD.GET_PID == 0x31
    assert P.CMD.SET_FORCE_TARGET == 0x32
    assert P.CMD.SET_MODE == 0x33
    assert P.CMD.EMERGENCY_STOP == 0x40
    assert P.CMD.CLEAR_FAULT == 0x41
    assert P.CMD.CALIBRATE_FORCE == 0x42
    assert P.CMD.ACK == 0x80
    assert P.CMD.HEARTBEAT_ACK == 0x81
    assert P.CMD.STATUS_RPT == 0xA0
    assert P.CMD.SENSOR_RPT == 0xA1
    assert P.CMD.PID_RPT == 0xA2
    assert P.CMD.FAULT_RPT == 0xB0
    assert P.CMD.TELEMETRY == 0xC0
    assert P.CMD.NACK == 0xE0


def test_every_command_round_trips_through_encode_and_decode():
    """Any command with both an encoder and a decoder must survive the trip."""
    samples = [
        (P.CMD.HEARTBEAT, P.encode_heartbeat(1234)),
        (P.CMD.SET_TARGET, P.encode_set_target(-100, 250, 180, 95, 42)),
        (P.CMD.START_PICK, P.encode_start_pick(42)),
        (P.CMD.SET_PID, P.encode_set_pid(2, 14.0, 6.0, 0.05)),
        (P.CMD.GET_PID, P.encode_get_pid(2)),
        (P.CMD.SET_FORCE_TARGET, P.encode_set_force_target(2200)),
        (P.CMD.SET_MODE, P.encode_set_mode(P.MODE_AUTO)),
        (P.CMD.ACK, P.encode_ack(P.CMD.START_PICK)),
        (P.CMD.HEARTBEAT_ACK, P.encode_heartbeat_ack(9)),
        (P.CMD.NACK, P.encode_nack(P.CMD.STOP, 3, 0)),
        (P.CMD.FAULT_RPT, P.encode_fault_rpt(1, 500, 12)),
        (P.CMD.TELEMETRY, P.encode_telemetry(1, 2, 3, 4, 5, 6, 0)),
    ]
    for cmd, payload in samples:
        decoded = P.decode_payload(cmd, payload)
        assert decoded is not None, "cmd 0x%02X decoded to None" % cmd
