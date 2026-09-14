# -*- coding: utf-8 -*-
"""tools/vision_mock.py -- vision system mock for the tea-picking robot.

Generates tea-bud detections on a jittered 3-D lattice that matches a real tea
bush and emits them as ``SET_TARGET`` frames at a configurable rate.

Behaviours modelled
-------------------
* bud placement: row spacing along ``y``, bud spacing along ``x``, bud height
  ``~N(180, 25) mm``, lateral jitter ``~N(0, 8) mm``
* each cycle the "camera" reports the highest-confidence *un-picked* bud that is
  inside the field of view (default 120 x 90 mm at the working distance)
* configurable miss rate (camera sees nothing this frame)
* confidence drawn from a clipped Beta distribution, mostly 0.75 .. 0.98
* configurable false-positive and duplicate-detection rates
* configurable measurement noise ``--noise-mm``
* **real** packet loss: ``--loss 0.05`` genuinely discards frames before they
  reach the transport, and the counts are reported at exit

Transports: ``--port COM5`` (serial), ``--tcp host:port``, or ``--stdout``
(hex dump, fully testable with no hardware).

Examples::

    python tools/vision_mock.py --stdout --count 20 --seed 1
    python tools/vision_mock.py --stdout --count 20 --seed 1 --loss 0.2 --quiet
    python tools/vision_mock.py --tcp 127.0.0.1:9000 --rate 10 --auto-pick

Python 3.8 compatible.
"""

import argparse
import os
import random
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple

if __package__ in (None, ""):  # allow ``python tools/vision_mock.py``
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import protocol
from host.link import Link, LinkError, TcpLink, parse_hostport, open_link
from host.protocol import CMD, Frame

__all__ = ["Bud", "Detection", "TeaBushScene", "VisionEmitter", "main"]


# --------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------

class Bud(object):
    """One tea bud on the bush (ground truth)."""

    __slots__ = ("target_id", "x_mm", "y_mm", "z_mm", "confidence", "picked")

    def __init__(self, target_id, x_mm, y_mm, z_mm, confidence):
        self.target_id = int(target_id)
        self.x_mm = float(x_mm)
        self.y_mm = float(y_mm)
        self.z_mm = float(z_mm)
        self.confidence = float(confidence)
        self.picked = False

    def __repr__(self):
        return "Bud(id=%d, x=%.1f, y=%.1f, z=%.1f, conf=%.3f, picked=%s)" % (
            self.target_id, self.x_mm, self.y_mm, self.z_mm, self.confidence,
            self.picked)


class Detection(object):
    """One measurement handed to the host (may be a false positive/duplicate)."""

    __slots__ = ("target_id", "x_mm", "y_mm", "z_mm", "confidence",
                 "false_positive", "duplicate")

    def __init__(self, target_id, x_mm, y_mm, z_mm, confidence,
                 false_positive=False, duplicate=False):
        self.target_id = int(target_id)
        self.x_mm = float(x_mm)
        self.y_mm = float(y_mm)
        self.z_mm = float(z_mm)
        self.confidence = float(confidence)
        self.false_positive = bool(false_positive)
        self.duplicate = bool(duplicate)

    def payload(self):
        # type: () -> bytes
        """Encode as a SET_TARGET payload (clamped into range)."""
        conf = int(round(_clamp(self.confidence, 0.0, 1.0) * 100.0))
        return protocol.encode_set_target(
            int(round(_clamp(self.x_mm, -32768, 32767))),
            int(round(_clamp(self.y_mm, -32768, 32767))),
            int(round(_clamp(self.z_mm, 0, 32767))),
            conf, self.target_id & 0xFFFF)

    def __repr__(self):
        tag = ""
        if self.false_positive:
            tag = " FP"
        elif self.duplicate:
            tag = " DUP"
        return "Detection(id=%d, x=%.1f, y=%.1f, z=%.1f, conf=%.2f%s)" % (
            self.target_id, self.x_mm, self.y_mm, self.z_mm, self.confidence, tag)


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# --------------------------------------------------------------------------
# Scene
# --------------------------------------------------------------------------

class TeaBushScene(object):
    """Jittered 3-D tea-bush lattice with a simple pinhole-style camera model."""

    def __init__(self, seed=1, fov=(120.0, 90.0), working_distance_mm=350.0,
                 bud_spacing_mm=42.0, row_spacing_mm=30.0, n_rows=9, n_cols=9,
                 height_mu_mm=180.0, height_sigma_mm=25.0, lateral_jitter_mm=8.0,
                 miss_rate=0.05, fp_rate=0.02, dup_rate=0.03,
                 conf_beta=(9.0, 1.4), conf_min=0.50, conf_max=0.995):
        # type: (int, Tuple[float, float], float, float, float, int, int, float, float, float, float, float, float, Tuple[float, float], float, float) -> None
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self.fov_x_mm, self.fov_y_mm = float(fov[0]), float(fov[1])
        self.working_distance_mm = float(working_distance_mm)
        self.bud_spacing_mm = float(bud_spacing_mm)
        self.row_spacing_mm = float(row_spacing_mm)
        self.height_mu_mm = float(height_mu_mm)
        self.height_sigma_mm = float(height_sigma_mm)
        self.lateral_jitter_mm = float(lateral_jitter_mm)
        self.miss_rate = float(miss_rate)
        self.fp_rate = float(fp_rate)
        self.dup_rate = float(dup_rate)
        self.conf_beta = (float(conf_beta[0]), float(conf_beta[1]))
        self.conf_min = float(conf_min)
        self.conf_max = float(conf_max)

        self.buds = []  # type: List[Bud]
        self._next_fp_id = 9000
        self.miss_count = 0
        self.fp_count = 0
        self.dup_count = 0
        self.cycles = 0
        self._build(n_rows, n_cols)

    # -- construction ---------------------------------------------------
    def _build(self, n_rows, n_cols):
        # type: (int, int) -> None
        tid = 1
        for row in range(int(n_rows)):
            y = (row - (int(n_rows) - 1) / 2.0) * self.row_spacing_mm
            for col in range(int(n_cols)):
                x = (col - (int(n_cols) - 1) / 2.0) * self.bud_spacing_mm
                self.buds.append(Bud(
                    tid,
                    x + self.rng.gauss(0.0, self.lateral_jitter_mm),
                    y + self.rng.gauss(0.0, self.lateral_jitter_mm),
                    _clamp(self.rng.gauss(self.height_mu_mm, self.height_sigma_mm),
                           60.0, 320.0),
                    self._draw_confidence()))
                tid += 1

    def _draw_confidence(self):
        # type: () -> float
        """Clipped Beta: mean ~0.865, essentially all mass in 0.75 .. 0.98."""
        a, b = self.conf_beta
        return _clamp(self.rng.betavariate(a, b), self.conf_min, self.conf_max)

    # -- camera ---------------------------------------------------------
    def in_view(self, bud):
        # type: (Bud) -> bool
        return (abs(bud.x_mm) <= self.fov_x_mm / 2.0
                and abs(bud.y_mm) <= self.fov_y_mm / 2.0)

    def visible(self):
        # type: () -> List[Bud]
        return [b for b in self.buds if (not b.picked) and self.in_view(b)]

    def target(self):
        # type: () -> Optional[Bud]
        """Highest-confidence un-picked bud currently in the field of view."""
        cands = self.visible()
        if not cands:
            return None
        return max(cands, key=lambda b: (b.confidence, -b.target_id))

    def _measure(self, bud, noise_mm, duplicate=False):
        # type: (Bud, float, bool) -> Detection
        return Detection(bud.target_id,
                         bud.x_mm + self.rng.gauss(0.0, noise_mm),
                         bud.y_mm + self.rng.gauss(0.0, noise_mm),
                         bud.z_mm + self.rng.gauss(0.0, noise_mm),
                         _clamp(bud.confidence + self.rng.gauss(0.0, 0.01),
                                self.conf_min * 0.6, self.conf_max),
                         duplicate=duplicate)

    def _false_positive(self, noise_mm):
        # type: (float) -> Detection
        self._next_fp_id += 1
        self.fp_count += 1
        return Detection(self._next_fp_id,
                         self.rng.uniform(-self.fov_x_mm / 2.0, self.fov_x_mm / 2.0)
                         + self.rng.gauss(0.0, noise_mm),
                         self.rng.uniform(-self.fov_y_mm / 2.0, self.fov_y_mm / 2.0)
                         + self.rng.gauss(0.0, noise_mm),
                         _clamp(self.rng.gauss(self.height_mu_mm, self.height_sigma_mm),
                                60.0, 320.0),
                         self.rng.uniform(0.35, 0.72),
                         false_positive=True)

    def detect(self, noise_mm=1.5):
        # type: (float) -> List[Detection]
        """Run one camera cycle; returns 0..n detections."""
        self.cycles += 1
        out = []  # type: List[Detection]
        if self.rng.random() < self.miss_rate:
            self.miss_count += 1   # camera saw nothing this frame
            return out
        primary = self.target()
        if primary is not None:
            out.append(self._measure(primary, noise_mm))
        if self.rng.random() < self.fp_rate:
            out.append(self._false_positive(noise_mm))
        if primary is not None and self.rng.random() < self.dup_rate:
            self.dup_count += 1
            out.append(self._measure(primary, noise_mm, duplicate=True))
        return out

    # -- bookkeeping ----------------------------------------------------
    def mark_picked(self, target_id):
        # type: (int) -> bool
        for bud in self.buds:
            if bud.target_id == int(target_id) and not bud.picked:
                bud.picked = True
                return True
        return False

    def picked_count(self):
        # type: () -> int
        return sum(1 for b in self.buds if b.picked)

    def stats(self):
        # type: () -> Dict[str, object]
        confs = [b.confidence for b in self.buds]
        return {"buds": len(self.buds), "picked": self.picked_count(),
                "in_view": len(self.visible()), "cycles": self.cycles,
                "misses": self.miss_count, "false_positives": self.fp_count,
                "duplicates": self.dup_count,
                "conf_min": min(confs) if confs else 0.0,
                "conf_max": max(confs) if confs else 0.0,
                "conf_mean": statistics.mean(confs) if confs else 0.0}


def parse_fov(text):
    # type: (str) -> Tuple[float, float]
    """Parse ``"120x90"`` / ``"120,90"`` / ``"120 90"`` into ``(x, y)`` mm."""
    raw = str(text).lower().replace("x", " ").replace(",", " ").split()
    if len(raw) != 2:
        raise argparse.ArgumentTypeError("--fov expects WxH in mm, e.g. 120x90")
    try:
        fx, fy = float(raw[0]), float(raw[1])
    except ValueError:
        raise argparse.ArgumentTypeError("--fov expects numbers, e.g. 120x90")
    if fx <= 0 or fy <= 0:
        raise argparse.ArgumentTypeError("--fov dimensions must be positive")
    return fx, fy


# --------------------------------------------------------------------------
# Transport / statistics
# --------------------------------------------------------------------------

def hexstr(data):
    # type: (bytes) -> str
    return " ".join("%02X" % b for b in bytearray(data))


class VisionEmitter(object):
    """Writes SET_TARGET frames to a link or to stdout, simulating packet loss."""

    def __init__(self, link=None, to_stdout=False, quiet=False, loss=0.0, seed=1,
                 tag=""):
        # type: (Optional[Link], bool, bool, float, int, str) -> None
        self.link = link
        self.to_stdout = bool(to_stdout)
        self.quiet = bool(quiet)
        self.loss = _clamp(float(loss), 0.0, 1.0)
        self.rng = random.Random(int(seed) ^ 0x5EED)
        self.tag = tag
        self.generated = 0
        self.sent = 0
        self.dropped = 0
        self.bytes_sent = 0
        self.targets_emitted = {}  # type: Dict[int, int]
        self.rate_limited = False

    def emit(self, frame, decoded=None):
        # type: (bytes, Optional[str]) -> bool
        """Try to put one frame on the wire.  Returns True when it was sent."""
        self.generated += 1
        if self.loss > 0.0 and self.rng.random() < self.loss:
            self.dropped += 1      # genuine drop: nothing reaches the transport
            return False
        if self.to_stdout:
            line = "%s%s" % (hexstr(frame), "" if self.quiet else "  | " + (decoded or ""))
            sys.stdout.write(line.rstrip() + "\n")
            sys.stdout.flush()
        else:
            if self.link is None:
                raise RuntimeError("no transport configured")
            try:
                self.link.write(frame)
            except LinkError as exc:
                raise LinkError("vision_mock write failed: %s" % exc)
            if not self.quiet:
                sys.stdout.write("tx %s\n" % (decoded or hexstr(frame)))
                sys.stdout.flush()
        self.sent += 1
        self.bytes_sent += len(frame)
        return True

    def poll(self, max_bytes=4096):
        # type: (int) -> List[Frame]
        if self.link is None:
            return []
        return self.link.read_frames(max_bytes)

    def summary(self, scene, transport_name, elapsed):
        # type: (TeaBushScene, str, float) -> str
        st = scene.stats()
        ratio = (100.0 * self.dropped / self.generated) if self.generated else 0.0
        lines = [
            "--- vision_mock summary ---",
            "transport        : %s" % transport_name,
            "elapsed          : %.2f s" % elapsed,
            "frames generated : %d" % self.generated,
            "frames sent      : %d" % self.sent,
            "frames dropped   : %d (%.1f%% simulated loss)" % (self.dropped, ratio),
            "byte count sent  : %d" % self.bytes_sent,
            "camera cycles    : %d (misses %d)" % (st["cycles"], st["misses"]),
            "false positives  : %d" % st["false_positives"],
            "duplicates       : %d" % st["duplicates"],
            "distinct targets : %d" % len(self.targets_emitted),
            "buds picked      : %d / %d" % (st["picked"], st["buds"]),
            "buds in view     : %d" % st["in_view"],
            "bud confidence   : min %.3f  mean %.3f  max %.3f"
            % (st["conf_min"], st["conf_mean"], st["conf_max"]),
        ]
        return "\n".join(lines)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_arg_parser():
    # type: () -> argparse.ArgumentParser
    p = argparse.ArgumentParser(
        prog="vision_mock.py",
        description="Tea-bud vision system mock: emits SET_TARGET frames.")
    t = p.add_argument_group("transport (pick one; default is --stdout)")
    t.add_argument("--port", metavar="COM5", help="serial port, e.g. COM5")
    t.add_argument("--tcp", metavar="HOST:PORT", help="TCP bridge, e.g. 127.0.0.1:9000")
    t.add_argument("--stdout", action="store_true",
                   help="print hex frames to stdout (no hardware needed)")
    t.add_argument("--baud", type=int, default=921600, help="serial baud (default 921600)")

    r = p.add_argument_group("generation")
    r.add_argument("--rate", type=float, default=10.0, help="frame rate in Hz (default 10)")
    r.add_argument("--loss", type=float, default=0.0,
                   help="simulated packet loss probability, e.g. 0.05")
    r.add_argument("--count", type=int, default=0,
                   help="stop after N frames generated (before loss); 0 = unlimited")
    r.add_argument("--duration", type=float, default=0.0,
                   help="stop after S seconds; 0 = unlimited")
    r.add_argument("--seed", type=int, default=1, help="RNG seed (default 1)")
    r.add_argument("--noise-mm", type=float, default=1.5, dest="noise_mm",
                   help="measurement noise sigma in mm (default 1.5)")
    r.add_argument("--fov", type=parse_fov, default=(120.0, 90.0),
                   help="field of view WxH in mm at working distance (default 120x90)")
    r.add_argument("--miss", type=float, default=0.05, dest="miss_rate",
                   help="camera miss rate per cycle (default 0.05)")
    r.add_argument("--fp-rate", type=float, default=0.02, dest="fp_rate",
                   help="false-positive rate per cycle (default 0.02)")
    r.add_argument("--dup-rate", type=float, default=0.03, dest="dup_rate",
                   help="duplicate-detection rate per cycle (default 0.03)")
    r.add_argument("--auto-pick", action="store_true", dest="auto_pick",
                   help="retire a bud once the robot acknowledges START_PICK")
    r.add_argument("--empty-cycles", type=int, default=5, dest="empty_cycles",
                   help="stop after N consecutive cycles with no un-picked bud in "
                        "the field of view (0 = never stop; default 5)")
    r.add_argument("--quiet", action="store_true", help="suppress decoded summaries")
    return p


def _make_transport(args):
    # type: (argparse.Namespace) -> Tuple[Optional[Link], bool, str]
    if args.stdout:
        return None, True, "stdout"
    if args.port:
        return open_link("serial", args.port, baud=args.baud), False, "serial %s" % args.port
    if args.tcp:
        host, port = parse_hostport(args.tcp)
        return open_link("tcp", args.tcp), False, "tcp %s:%d" % (host, port)
    return None, True, "stdout (default; no --port/--tcp given)"


def main(argv=None):
    # type: (Optional[List[str]]) -> int
    args = build_arg_parser().parse_args(argv)
    rate = float(args.rate)
    if rate <= 0.0:
        sys.stderr.write("--rate must be > 0\n")
        return 2
    if not (0.0 <= args.loss <= 1.0):
        sys.stderr.write("--loss must be within 0..1\n")
        return 2

    try:
        link, to_stdout, transport = _make_transport(args)
    except (LinkError, ValueError) as exc:
        sys.stderr.write("transport error: %s\n" % exc)
        return 2

    scene = TeaBushScene(seed=args.seed, fov=args.fov, miss_rate=args.miss_rate,
                         fp_rate=args.fp_rate, dup_rate=args.dup_rate)
    emitter = VisionEmitter(link=link, to_stdout=to_stdout, quiet=args.quiet,
                            loss=args.loss, seed=args.seed)
    if not args.quiet:
        sys.stdout.write(
            "vision_mock: transport=%s rate=%.1f Hz loss=%.0f%% seed=%d fov=%.0fx%.0f mm\n"
            % (transport, rate, args.loss * 100.0, args.seed, args.fov[0], args.fov[1]))
        sys.stdout.write(
            "scene: %d buds, %d in view, confidence mean %.3f\n"
            % (scene.stats()["buds"], scene.stats()["in_view"], scene.stats()["conf_mean"]))
        sys.stdout.flush()

    period = 1.0 / rate
    t_start = time.monotonic()
    deadline = t_start + args.duration if args.duration > 0 else None
    next_t = t_start
    pending = []  # type: List[List[float]]
    # --auto-pick over a real link retires a bud when a genuine
    # ACK(acked_cmd=START_PICK) arrives from the robot.  --stdout has no return
    # path at all, so for that transport only we emulate the acknowledgement one
    # cycle after the target is emitted (each entry is [target_id, cycles_left]).
    inflight = None  # type: Optional[int]
    empty_streak = 0
    exit_code = 0

    try:
        while True:
            if args.count > 0 and emitter.generated >= args.count:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break

            for det in scene.detect(args.noise_mm):
                payload = det.payload()
                # keep the payload and the reported values consistent
                decoded = "%s | %s" % (protocol.cmd_name(CMD.SET_TARGET),
                                       _detection_text(det))
                sent = emitter.emit(protocol.build_frame(CMD.SET_TARGET, payload), decoded)
                if sent:
                    emitter.targets_emitted[det.target_id] = \
                        emitter.targets_emitted.get(det.target_id, 0) + 1
                    if args.auto_pick and not det.false_positive and not det.duplicate:
                        if to_stdout:
                            pending.append([float(det.target_id), 1.0])
                        else:
                            inflight = det.target_id

            # --auto-pick bookkeeping
            if args.auto_pick:
                if to_stdout:
                    for item in list(pending):
                        item[1] -= 1.0
                        if item[1] <= 0.0:
                            scene.mark_picked(int(item[0]))
                            pending.remove(item)
                elif link is not None:
                    for frame in emitter.poll():
                        if frame.cmd == CMD.ACK:
                            ack = protocol.decode_ack(frame.payload)
                            if ack.acked_cmd == CMD.START_PICK and inflight is not None:
                                scene.mark_picked(inflight)
                                inflight = None

            # stop when the camera has nothing left to look at (e.g. --auto-pick
            # has retired every bud inside the field of view)
            empty_streak = empty_streak + 1 if scene.target() is None else 0
            if args.empty_cycles > 0 and empty_streak >= args.empty_cycles:
                sys.stdout.write(
                    "no un-picked buds left in the field of view; stopping after "
                    "%d empty cycles\n" % empty_streak)
                sys.stdout.flush()
                break

            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # we fell behind: resynchronise
    except KeyboardInterrupt:
        if not args.quiet:
            sys.stdout.write("\ninterrupted\n")
    except LinkError as exc:
        sys.stderr.write("link error: %s\n" % exc)
        exit_code = 3
    finally:
        if link is not None:
            link.close()

    sys.stdout.write(emitter.summary(scene, transport, time.monotonic() - t_start) + "\n")
    sys.stdout.flush()
    return exit_code


def _detection_text(det):
    # type: (Detection) -> str
    tag = ""
    if det.false_positive:
        tag = " [FALSE-POSITIVE]"
    elif det.duplicate:
        tag = " [DUPLICATE]"
    return ("x=%+5.1fmm y=%+5.1fmm z=%5.1fmm conf=%.2f target_id=%d%s"
            % (det.x_mm, det.y_mm, det.z_mm, det.confidence, det.target_id, tag))


if __name__ == "__main__":
    sys.exit(main())
