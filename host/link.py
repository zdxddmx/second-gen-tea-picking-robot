# -*- coding: utf-8 -*-
"""host/link.py -- transport abstraction for the robot protocol.

Three interchangeable transports implement the same tiny interface::

    link.open()                  -> None
    link.close()                 -> None
    link.write(data: bytes)      -> int      (bytes accepted)
    link.read(max_bytes: int)    -> bytes    (may be b"")
    link.is_open                 -> bool

``SerialLink``   real UART via pyserial (imported lazily and defensively)
``TcpLink``      raw TCP socket to the robot's bridge / WiFi module
``LoopbackLink`` fully in-process; used by tests and by ``--demo`` mode

Every link owns a :class:`LinkStats` instance which the monitor displays, so the
GUI, the headless dashboard and the demos all report the same numbers.

Python 3.8 compatible.
"""

import socket
import struct
import time
from typing import Callable, List, Optional

from host import protocol
from host.protocol import Frame, FrameParser

__all__ = [
    "LinkError", "LinkStats", "Link", "SerialLink", "TcpLink", "LoopbackLink",
    "open_link", "parse_hostport",
]


class LinkError(Exception):
    """Raised when a transport cannot be opened or used."""


def parse_hostport(text, default_port=9000):
    # type: (str, int) -> tuple
    """Parse ``"host:port"``, ``"host"`` or ``":port"`` into ``(host, port)``."""
    text = str(text).strip()
    if not text:
        raise ValueError("empty host:port string")
    if ":" in text:
        host, _, port_s = text.rpartition(":")
        host = host or "127.0.0.1"
        try:
            port = int(port_s)
        except ValueError:
            raise ValueError("bad port in %r" % text)
    else:
        host, port = text, int(default_port)
    if not (0 < port < 65536):
        raise ValueError("port out of range: %d" % port)
    return host, port


# --------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------

class LinkStats(object):
    """Counters shared by every transport and displayed by the monitor."""

    def __init__(self):
        self.tx_frames = 0
        self.rx_frames = 0
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.crc_errors = 0
        self.bytes_discarded = 0
        self.dropped_tx = 0          # writes intentionally dropped (loss sim)
        self.packets_lost = 0        # requests that never got any response
        self.timeouts = 0
        self.opened_at = None        # type: Optional[float]
        self.closed_at = None        # type: Optional[float]
        # outstanding request bookkeeping (estimated packet loss)
        self._outstanding = 0
        self._requests = 0
        self._responses = 0

    # -- counters ---------------------------------------------------------
    def note_tx_frame(self, size):
        # type: (int) -> None
        self.tx_frames += 1
        self.tx_bytes += int(size)

    def note_rx_frame(self, size):
        # type: (int) -> None
        self.rx_frames += 1
        self.rx_bytes += int(size)

    def note_tx_dropped(self, size):
        # type: (int) -> None
        self.dropped_tx += 1
        self.tx_bytes += int(size)

    def note_parser(self, parser):
        # type: (FrameParser) -> None
        self.crc_errors = parser.frames_crc_error
        self.bytes_discarded = parser.bytes_discarded

    # -- estimated packet loss -------------------------------------------
    def note_request(self):
        # type: () -> None
        """A command that expects a response was sent."""
        self._requests += 1
        self._outstanding += 1

    def note_response(self):
        # type: () -> None
        """Any unsolicited or solicited frame arrived from the robot."""
        self._responses += 1
        if self._outstanding > 0:
            self._outstanding -= 1

    def note_timeout(self):
        # type: () -> None
        """No response arrived in time; charge one lost packet."""
        self.timeouts += 1
        if self._outstanding > 0:
            self._outstanding -= 1
            self.packets_lost += 1

    @property
    def outstanding(self):
        # type: () -> int
        return self._outstanding

    @property
    def requests(self):
        # type: () -> int
        return self._requests

    @property
    def responses(self):
        # type: () -> int
        return self._responses

    @property
    def loss_ratio(self):
        # type: () -> float
        if self._requests <= 0:
            return 0.0
        return float(self.packets_lost) / float(self._requests)

    def reset(self):
        # type: () -> None
        self.__init__()

    # -- presentation -----------------------------------------------------
    def as_dict(self):
        # type: () -> dict
        return {
            "tx_frames": self.tx_frames,
            "rx_frames": self.rx_frames,
            "tx_bytes": self.tx_bytes,
            "rx_bytes": self.rx_bytes,
            "crc_errors": self.crc_errors,
            "bytes_discarded": self.bytes_discarded,
            "dropped_tx": self.dropped_tx,
            "packets_lost": self.packets_lost,
            "loss_ratio": self.loss_ratio,
            "timeouts": self.timeouts,
        }

    def summary(self):
        # type: () -> str
        return ("tx=%d rx=%d bytes=%d/%d crc_err=%d discarded=%d lost=%d (%.2f%%)"
                % (self.tx_frames, self.rx_frames, self.tx_bytes, self.rx_bytes,
                   self.crc_errors, self.bytes_discarded, self.packets_lost,
                   100.0 * self.loss_ratio))

    def __repr__(self):
        return "LinkStats(%s)" % self.summary()


# --------------------------------------------------------------------------
# Base class
# --------------------------------------------------------------------------

class Link(object):
    """Abstract transport.  Subclasses implement the ``_``-prefixed hooks."""

    name = "link"

    def __init__(self):
        self.stats = LinkStats()
        self.parser = FrameParser()
        self._open = False

    # -- lifecycle --------------------------------------------------------
    @property
    def is_open(self):
        # type: () -> bool
        return self._open

    def open(self):
        # type: () -> None
        self._do_open()
        self._open = True
        self.stats.opened_at = time.time()

    def close(self):
        # type: () -> None
        if self._open:
            try:
                self._do_close()
            finally:
                self._open = False
                self.stats.closed_at = time.time()

    # -- I/O --------------------------------------------------------------
    def write(self, data):
        # type: (bytes) -> int
        raise NotImplementedError

    def read(self, max_bytes=4096):
        # type: (int) -> bytes
        raise NotImplementedError

    def _do_open(self):
        pass

    def _do_close(self):
        pass

    # -- helpers ----------------------------------------------------------
    def send_frame(self, cmd, payload=b"", expect_response=True):
        # type: (int, bytes, bool) -> bytes
        """Build and write a frame, updating statistics."""
        wire = protocol.build_frame(cmd, payload)
        written = self.write(wire)
        self.stats.note_tx_frame(len(wire))
        if expect_response and cmd in protocol.HOST_TO_ROBOT:
            self.stats.note_request()
        return wire

    def read_frames(self, max_bytes=4096):
        # type: (int) -> List[Frame]
        """Read whatever is available and decode it into frames."""
        data = self.read(max_bytes)
        if not data:
            return []
        self.stats.rx_bytes += len(data)
        frames = self.parser.feed(data)
        self.stats.note_parser(self.parser)
        for frame in frames:
            self.stats.note_rx_frame(len(frame.raw))
            self.stats.note_response()
        return frames

    def describe(self):
        # type: () -> str
        return self.name

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


# --------------------------------------------------------------------------
# Serial
# --------------------------------------------------------------------------

_SERIAL_MODULE = None
_SERIAL_IMPORT_ERROR = None


def _import_serial():
    """Import pyserial lazily; cache the failure so callers get a clear error."""
    global _SERIAL_MODULE, _SERIAL_IMPORT_ERROR
    if _SERIAL_MODULE is not None:
        return _SERIAL_MODULE
    if _SERIAL_IMPORT_ERROR is not None:
        raise LinkError("pyserial is not available: %s" % _SERIAL_IMPORT_ERROR)
    try:
        import serial  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on the machine
        _SERIAL_IMPORT_ERROR = "%s: %s" % (type(exc).__name__, exc)
        raise LinkError("pyserial is not available: %s" % _SERIAL_IMPORT_ERROR)
    _SERIAL_MODULE = serial
    return serial


class SerialLink(Link):
    """UART transport (default 921600 8N1, matching the firmware)."""

    name = "serial"

    def __init__(self, port, baud=921600, timeout=0.02):
        # type: (str, int, float) -> None
        Link.__init__(self)
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self._serial = None
        self._serial_mod = None

    def _do_open(self):
        serial_mod = _import_serial()
        self._serial_mod = serial_mod
        try:
            self._serial = serial_mod.Serial(
                port=self.port, baudrate=self.baud, bytesize=8,
                parity=serial_mod.PARITY_NONE, stopbits=1,
                timeout=self.timeout, write_timeout=1.0)
        except Exception as exc:
            raise LinkError("cannot open serial port %s: %s: %s"
                            % (self.port, type(exc).__name__, exc))

    def _do_close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def write(self, data):
        # type: (bytes) -> int
        if self._serial is None:
            raise LinkError("serial port %s is not open" % self.port)
        try:
            return int(self._serial.write(data))
        except Exception as exc:
            raise LinkError("serial write failed: %s: %s" % (type(exc).__name__, exc))

    def read(self, max_bytes=4096):
        # type: (int) -> bytes
        if self._serial is None:
            raise LinkError("serial port %s is not open" % self.port)
        try:
            waiting = int(getattr(self._serial, "in_waiting", 0) or 0)
        except Exception:
            waiting = 0
        count = max(1, min(int(max_bytes), waiting if waiting > 0 else 1))
        try:
            return bytes(self._serial.read(count))
        except Exception as exc:
            raise LinkError("serial read failed: %s: %s" % (type(exc).__name__, exc))

    def describe(self):
        # type: () -> str
        return "serial %s @ %d baud" % (self.port, self.baud)


# --------------------------------------------------------------------------
# TCP
# --------------------------------------------------------------------------

class TcpLink(Link):
    """Raw TCP transport, e.g. an ESP32/PC bridge or a serial-over-WiFi module."""

    name = "tcp"

    def __init__(self, host, port, timeout=0.02, connect_timeout=3.0):
        # type: (str, int, float, float) -> None
        Link.__init__(self)
        self.host = str(host)
        self.port = int(port)
        self.timeout = float(timeout)
        self.connect_timeout = float(connect_timeout)
        self._sock = None

    def _do_open(self):
        try:
            sock = socket.create_connection((self.host, self.port),
                                            timeout=self.connect_timeout)
        except OSError as exc:
            raise LinkError("cannot connect to %s:%d: %s" % (self.host, self.port, exc))
        sock.settimeout(self.timeout)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._sock = sock

    def _do_close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def write(self, data):
        # type: (bytes) -> int
        if self._sock is None:
            raise LinkError("tcp link %s:%d is not open" % (self.host, self.port))
        try:
            return int(self._sock.send(data))
        except OSError as exc:
            raise LinkError("tcp write failed: %s" % exc)

    def read(self, max_bytes=4096):
        # type: (int) -> bytes
        if self._sock is None:
            raise LinkError("tcp link %s:%d is not open" % (self.host, self.port))
        try:
            return bytes(self._sock.recv(int(max_bytes)))
        except socket.timeout:
            return b""
        except OSError as exc:
            raise LinkError("tcp read failed: %s" % exc)

    def describe(self):
        # type: () -> str
        return "tcp %s:%d" % (self.host, self.port)


# --------------------------------------------------------------------------
# Loopback
# --------------------------------------------------------------------------

class LoopbackLink(Link):
    """Fully in-process transport.

    * every ``write`` is recorded in :attr:`tx_log`
    * if a ``responder`` callable is supplied it is invoked with the written
      bytes and whatever it returns is queued for reading
    * :meth:`push_rx` injects bytes directly (used to stream unsolicited
      telemetry out of :class:`host.sim_robot.SimRobot`)
    * ``drop_rate`` > 0 drops writes on purpose so packet-loss behaviour can be
      exercised without any hardware
    """

    name = "loopback"

    def __init__(self, responder=None, drop_rate=0.0, rng=None):
        # type: (Optional[Callable[[bytes], bytes]], float, Optional[object]) -> None
        Link.__init__(self)
        self.responder = responder
        self.drop_rate = float(drop_rate)
        self.tx_log = []       # type: List[bytes]
        self.tx_frames_log = []  # type: List[Frame]
        self._rx = bytearray()
        self._rng = rng
        self.dropped = 0

    # -- internals --------------------------------------------------------
    def _random(self):
        # type: () -> float
        if self._rng is not None:
            return float(self._rng.random())
        import random
        return random.random()

    # -- Link API ---------------------------------------------------------
    def _do_open(self):
        self._rx = bytearray()

    def _do_close(self):
        self._rx = bytearray()

    def write(self, data):
        # type: (bytes) -> int
        data = bytes(data)
        self.tx_log.append(data)
        if self.drop_rate > 0.0 and self._random() < self.drop_rate:
            self.dropped += 1
            self.stats.note_tx_dropped(len(data))
            return 0
        if self.responder is not None:
            reply = self.responder(data)
            if reply:
                self._rx.extend(reply)
        return len(data)

    def read(self, max_bytes=4096):
        # type: (int) -> bytes
        if not self._rx:
            return b""
        count = min(int(max_bytes), len(self._rx))
        out = bytes(self._rx[:count])
        del self._rx[:count]
        return out

    # -- extra API --------------------------------------------------------
    def push_rx(self, data):
        # type: (bytes) -> None
        """Queue bytes as if they had arrived from the robot."""
        if data:
            self._rx.extend(bytes(data))

    def clear(self):
        # type: () -> None
        self.tx_log = []
        self.tx_frames_log = []
        self._rx = bytearray()

    def tx_frame_list(self):
        # type: () -> List[Frame]
        """Decode everything that was written, in order (test helper)."""
        parser = FrameParser()
        out = []  # type: List[Frame]
        for chunk in self.tx_log:
            out.extend(parser.feed(chunk))
        return out

    def describe(self):
        # type: () -> str
        target = "sim" if self.responder is not None else "loop"
        return "loopback(%s, drop=%.2f)" % (target, self.drop_rate)


# --------------------------------------------------------------------------
# Factory
# --------------------------------------------------------------------------

def open_link(kind, target=None, baud=921600, timeout=0.02, responder=None):
    # type: (str, Optional[str], int, float, Optional[Callable[[bytes], bytes]]) -> Link
    """Create and open a link.

    ``kind`` is ``"serial"``, ``"tcp"`` or ``"loopback"``.  For serial,
    ``target`` is the port name (``COM5``); for TCP it is ``host:port``.
    """
    kind = str(kind).lower()
    if kind == "serial":
        if not target:
            raise LinkError("serial link needs a port name, e.g. COM5")
        link = SerialLink(target, baud=baud, timeout=timeout)  # type: Link
    elif kind == "tcp":
        host, port = parse_hostport(target or "127.0.0.1:9000")
        link = TcpLink(host, port, timeout=timeout)
    elif kind in ("loopback", "demo", "loop"):
        link = LoopbackLink(responder=responder)
    else:
        raise LinkError("unknown link kind %r (use serial|tcp|loopback)" % kind)
    link.open()
    return link


if __name__ == "__main__":
    # Tiny smoke test: loopback round-trip with an echo responder.
    frames_seen = []  # type: List[Frame]

    def echo(data):
        frames_seen.append(data)
        return data  # echo everything back

    lk = LoopbackLink(responder=echo)
    lk.open()
    lk.send_frame(protocol.CMD.GET_STATUS)
    got = lk.read_frames()
    print("loopback echo frames:", [str(f) for f in got])
    print("stats:", lk.stats.summary())
    lk.close()
    assert len(got) == 1 and got[0].cmd == protocol.CMD.GET_STATUS
    print("link.py smoke test PASS")
