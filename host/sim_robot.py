# -*- coding: utf-8 -*-
"""host/sim_robot.py -- small *tool-side* robot model for demo / test runs.

IMPORTANT: deliberately NOT the authoritative plant model.  It exists only so the
monitor, the vision mock and the self tests run with no hardware attached; the
authoritative dynamics live in ``simulator/`` and in the C firmware.

Implements the pick state machine, a first-order joint model, a damped gripper
contact-force model with an overload fault, correct wire responses for the host
command subset, and unsolicited TELEMETRY frames for plotting::

    sim  = SimRobot(seed=1)
    link = LoopbackLink(responder=sim.feed)   # answers host commands
    link.push_rx(sim.tick())                  # unsolicited telemetry
"""

import os, random, struct, sys, time  # noqa: E401 - compact stdlib import block
from typing import Dict, Optional, Tuple

if __package__ in (None, ""):  # allow ``python host/sim_robot.py``
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import protocol
from host.protocol import CMD, FAULT, STATE, FrameParser

__all__ = ["SimRobot", "MM_TO_MRAD", "STATE_DWELL", "NEXT_STATE"]

MM_TO_MRAD = 10.0  #: carriage millimetres -> motor shaft milliradians
STATE_DWELL = {STATE.INIT: 0.05, STATE.SELF_CHECK: 0.15, STATE.APPROACH: 0.80,
               STATE.CONTACT: 0.50, STATE.GRIP: 0.50, STATE.PICK: 0.40,
               STATE.RELEASE: 0.30, STATE.RETURN: 0.50}  # type: Dict[int, float]
NEXT_STATE = {STATE.INIT: STATE.SELF_CHECK, STATE.SELF_CHECK: STATE.IDLE,
              STATE.APPROACH: STATE.CONTACT, STATE.CONTACT: STATE.GRIP,
              STATE.GRIP: STATE.PICK, STATE.PICK: STATE.RELEASE,
              STATE.RELEASE: STATE.RETURN, STATE.RETURN: STATE.IDLE}
# state -> (height offset from target in mm, or None for "go home"; force fraction)
_TRAJ = {STATE.APPROACH: (-12.0, 0.0), STATE.CONTACT: (0.0, 0.35),
         STATE.GRIP: (0.0, 1.0), STATE.PICK: (25.0, 0.9), STATE.RELEASE: (-2.0, 0.2),
         STATE.RETURN: (None, 0.0), STATE.INIT: (None, 0.0), STATE.SELF_CHECK: (None, 0.0),
         STATE.IDLE: (None, 0.0), STATE.ERROR: (None, 0.0),
         STATE.EMERGENCY_STOP: (None, 0.0)}  # type: Dict[int, Tuple[Optional[float], float]]


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def _i16(v):  # saturating cast for i16 telemetry fields
    return int(_clamp(int(round(v)), -32768, 32767))


class SimRobot(object):
    """Tool-side robot model speaking the real wire protocol."""

    def __init__(self, seed=0, telemetry_hz=20.0, force_limit_mN=4000, home_mrad=0.0, dt=0.01):
        self.seed, self.dt, self.home_mrad = int(seed), float(dt), float(home_mrad)
        self.telemetry_hz, self.force_limit_mN = float(telemetry_hz), int(force_limit_mN)
        self.rng, self.mode, self.force_target_mN = random.Random(self.seed), protocol.MODE_AUTO, 1200
        self.pid = {0: (2.4, 0.35, 0.05), 1: (1.2, 0.20, 0.02), 2: (1.8, 0.10, 0.01)}
        self.target = None  # type: Optional[protocol.SetTarget]
        self._parser = FrameParser()
        self._t0, self._last_tick, self._last_telemetry = time.time(), 0.0, 0.0
        self.reset()

    def reset(self):
        """Return to power-on conditions (INIT, no fault, carriage at home)."""
        self.state, self.fault, self.fault_pending = STATE.INIT, FAULT.NONE, FAULT.NONE
        self.pos_mrad, self.target_pos_mrad = self.home_mrad, self.home_mrad
        self.vel_mrad_s = self.vel_cmd = 0.0
        self.force_mN = self.desired_force_mN = 0.0
        self.pwm_permille = self.current_ma = 0
        self.temp_c_x10, self.adc_raw, self.cycle_count = 245, 2048, 0
        self.target, self._state_since = None, self._now()

    def _now(self):
        return time.time() - self._t0

    def _enter(self, state):
        if self.state != state:
            self.state, self._state_since = int(state), self._now()

    def _set_fault(self, code):
        self.fault = self.fault_pending = int(code)
        if code != FAULT.NONE and self.state != STATE.EMERGENCY_STOP:
            self.state, self._state_since = STATE.ERROR, self._now()

    def tick(self, now=None):
        # type: (Optional[float]) -> bytes
        """Advance the model; return unsolicited frames to push upstream."""
        t = self._now() if now is None else float(now) - self._t0
        dt = _clamp(t - self._last_tick, 0.0, 0.2)
        self._last_tick = t
        out = bytearray()
        if dt > 0.0:
            self._step_dynamics(dt)
            self._step_fsm()
        if self.fault_pending != FAULT.NONE:
            out.extend(protocol.build_frame(CMD.FAULT_RPT, protocol.encode_fault_rpt(
                self.fault_pending, int(t * 1000.0) & 0xFFFFFFFF, self.cycle_count & 0xFF)))
            self.fault_pending = FAULT.NONE
        if self.telemetry_hz > 0.0 and (t - self._last_telemetry) >= 1.0 / self.telemetry_hz:
            self._last_telemetry = t
            out += self.telemetry_frame()
        return bytes(out)

    def _step_dynamics(self, dt):
        vmax = 900.0
        kp = self.pid.get(0, (2.4, 0.35, 0.05))[0]
        self.vel_cmd = _clamp(kp * 0.04 * (self.target_pos_mrad - self.pos_mrad), -vmax, vmax)
        self.vel_mrad_s += (self.vel_cmd - self.vel_mrad_s) * min(1.0, dt / 0.035)
        self.pos_mrad += self.vel_mrad_s * dt
        self.force_mN = max(0.0, self.force_mN
                            + (self.desired_force_mN - self.force_mN) * min(1.0, dt / 0.08))
        effort = abs(self.vel_cmd - self.vel_mrad_s) / vmax
        payload = self.force_mN / float(max(1, self.force_limit_mN))
        self.pwm_permille = int(_clamp(1000.0 * (0.55 * effort + 0.35 * payload)
                                       + (60.0 if abs(self.vel_mrad_s) > 5.0 else 0.0),
                                       -1000, 1000))
        self.current_ma = int(_clamp(35.0 + abs(self.pwm_permille) * 1.15
                                     + self.force_mN * 0.05, 0, 32767))
        self.temp_c_x10 = int(_clamp(self.temp_c_x10 + dt * 0.35 * self.current_ma / 100.0,
                                     200, 1200))
        self.adc_raw = int(_clamp(2048 + (self.pos_mrad - self.home_mrad) * 0.02
                                  + self.rng.gauss(0.0, 3.0), 0, 65535))
        if self.force_mN > self.force_limit_mN and self.fault == FAULT.NONE:
            self._set_fault(FAULT.FORCE_OVERLOAD)

    def _step_fsm(self):
        st = self.state
        if st == STATE.TARGET_RECEIVED:
            self.desired_force_mN = 0.0
            return
        dwell = STATE_DWELL.get(st)
        if dwell is not None and (self._now() - self._state_since) >= dwell:
            if st == STATE.RETURN:
                self.cycle_count = (self.cycle_count + 1) & 0xFFFF
            self._enter(NEXT_STATE[st])
            st = self.state
        off, frac = _TRAJ.get(st, (None, 0.0))
        base = self.home_mrad
        if off is not None and self.target is not None:
            base = self.target.z_mm * MM_TO_MRAD + self.home_mrad
        self.target_pos_mrad = base if off is None else base + off * MM_TO_MRAD
        self.desired_force_mN = self.force_target_mN * frac

    def status_frame(self):
        return protocol.build_frame(CMD.STATUS_RPT, protocol.encode_status_rpt(
            self.state, self.fault, int(self.pos_mrad), _i16(self.vel_mrad_s),
            _i16(self.force_mN), self.pwm_permille, _i16(self.current_ma), self.cycle_count))

    def sensor_frame(self):
        return protocol.build_frame(CMD.SENSOR_RPT, protocol.encode_sensor_rpt(
            int(self.pos_mrad), _i16(self.vel_mrad_s), _i16(self.force_mN),
            self.adc_raw, _i16(self.current_ma), self.temp_c_x10))

    def telemetry_frame(self):
        return protocol.build_frame(CMD.TELEMETRY, protocol.encode_telemetry(
            int(self._now() * 1000.0) & 0xFFFFFFFF, int(self.pos_mrad),
            _i16(self.vel_mrad_s), _i16(self.force_mN), self.pwm_permille,
            self.state, self.fault))

    def feed(self, data):
        # type: (bytes) -> bytes
        """Link responder: consume host bytes, return robot->host bytes."""
        out = bytearray()
        for frame in self._parser.feed(data):
            out.extend(self.handle(frame))
        return bytes(out)

    def handle(self, frame):
        # type: (protocol.Frame) -> bytes
        handler = _HANDLERS.get(frame.cmd)
        if handler is None:
            return self._nack(frame.cmd, protocol.ERRCODE.UNKNOWN_CMD)
        try:
            return handler(self, frame.payload)
        except (ValueError, struct.error, IndexError, TypeError):
            return self._nack(frame.cmd, protocol.ERRCODE.BAD_PARAM)

    def _ack(self, cmd):
        return protocol.build_frame(CMD.ACK, protocol.encode_ack(cmd))

    def _nack(self, cmd, err):
        return protocol.build_frame(CMD.NACK, protocol.encode_nack(cmd, err))

    def _pid_frame(self, loop):
        kp, ki, kd = self.pid.get(loop, (0.0, 0.0, 0.0))
        return protocol.build_frame(CMD.PID_RPT, protocol.encode_pid_rpt(loop, kp, ki, kd))

    # -- command handlers ----------------------------------------------
    def h_heartbeat(self, payload):
        return protocol.build_frame(CMD.HEARTBEAT_ACK, protocol.encode_heartbeat_ack(
            struct.unpack("<I", payload)[0]))
    def h_set_target(self, payload):
        self.target = protocol.decode_set_target(payload)
        self._enter(STATE.TARGET_RECEIVED)
        return self._ack(CMD.SET_TARGET)
    def h_start_pick(self, payload):
        if self.fault != FAULT.NONE or self.state in (STATE.ERROR, STATE.EMERGENCY_STOP):
            return self._nack(CMD.START_PICK, protocol.ERRCODE.NOT_READY)
        if self.target is None:
            return self._nack(CMD.START_PICK, protocol.ERRCODE.BAD_PARAM)
        self._enter(STATE.APPROACH)
        return self._ack(CMD.START_PICK)
    def h_stop(self, payload):
        self._enter(STATE.IDLE)
        return self._ack(CMD.STOP)
    def h_reset(self, payload):
        self.reset()
        return self._ack(CMD.RESET)
    def h_estop(self, payload):
        self.vel_cmd, self.vel_mrad_s, self.desired_force_mN = 0.0, 0.0, 0.0
        self._enter(STATE.EMERGENCY_STOP)
        self._set_fault(FAULT.ESTOP)
        return self._ack(CMD.EMERGENCY_STOP)
    def h_clear_fault(self, payload):
        self.fault = self.fault_pending = FAULT.NONE
        self._enter(STATE.IDLE)
        return self._ack(CMD.CLEAR_FAULT)
    def h_set_mode(self, payload):
        self.mode = protocol.decode_set_mode(payload).mode
        return self._ack(CMD.SET_MODE)
    def h_set_pid(self, payload):
        loop, kp, ki, kd = protocol.decode_set_pid(payload)
        self.pid[loop] = (kp, ki, kd)
        return self._ack(CMD.SET_PID) + self._pid_frame(loop)
    def h_get_pid(self, payload):
        return self._pid_frame(protocol.decode_get_pid(payload).loop_id)
    def h_set_force_target(self, payload):
        self.force_target_mN = protocol.decode_set_force_target(payload).force_mN
        return self._ack(CMD.SET_FORCE_TARGET)
    def h_get_status(self, payload):
        return self._ack(CMD.GET_STATUS) + self.status_frame()
    def h_get_sensor(self, payload):
        return self._ack(CMD.GET_SENSOR) + self.sensor_frame()
    def h_get_fault(self, payload):
        return protocol.build_frame(CMD.FAULT_RPT, protocol.encode_fault_rpt(
            self.fault, int(self._now() * 1000.0) & 0xFFFFFFFF, self.cycle_count & 0xFF))


_HANDLERS = {
    CMD.HEARTBEAT: SimRobot.h_heartbeat, CMD.SET_TARGET: SimRobot.h_set_target,
    CMD.START_PICK: SimRobot.h_start_pick, CMD.STOP: SimRobot.h_stop,
    CMD.RESET: SimRobot.h_reset, CMD.EMERGENCY_STOP: SimRobot.h_estop,
    CMD.CLEAR_FAULT: SimRobot.h_clear_fault, CMD.SET_MODE: SimRobot.h_set_mode,
    CMD.SET_PID: SimRobot.h_set_pid, CMD.GET_PID: SimRobot.h_get_pid,
    CMD.SET_FORCE_TARGET: SimRobot.h_set_force_target,
    CMD.GET_STATUS: SimRobot.h_get_status, CMD.GET_SENSOR: SimRobot.h_get_sensor,
    CMD.GET_FAULT: SimRobot.h_get_fault,
}  # type: Dict[int, object]
