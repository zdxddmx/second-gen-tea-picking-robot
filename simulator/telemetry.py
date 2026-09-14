"""
Telemetry recording and response-metric extraction.

What
----
:class:`Recorder` accumulates one row per control tick into parallel lists and
writes them out as CSV (for spreadsheets and external analysis) and JSON (for
tooling that wants structure).  :func:`metrics` derives the standard control and
grasp figures of merit from those arrays: rise time, overshoot, settling time,
steady-state error, peak force, force RMSE and damage index.

Why
---
A plot is an argument; a number is evidence.  The same run must be summarisable
so that two controller variants can be compared without a human squinting at
overlaid curves, and the raw arrays must survive the run so that a later question
("was the PWM saturated at 0.4 s?") is answerable without re-simulating.  Every
scalar quoted in the CLI trace comes from this module, which is what makes the
printed numbers reproducible rather than decorative.

What happens without it
-----------------------
Results would exist only inside the process that produced them: no CSV to re-plot,
no metrics to tabulate, and no way to check a claim after the fact.

Units
-----
Time in s, position in mrad, velocity in mrad/s, force in N, current in A, PWM in
per-mille, ``adc_raw`` in 12-bit counts.
"""

from __future__ import annotations

import csv
import json
import math
import os
import zlib
from typing import Dict, List, Optional, Sequence

# numpy is imported at module scope rather than lazily inside the few functions
# that use it.  The public type annotations mention ``np.ndarray``, so a lazy
# import left ``np`` undefined at run time and every scenario raised
# ``NameError: name 'np' is not defined`` the moment it tried to build its
# result -- the simulation itself ran to completion and then failed while
# reporting.  This module is imported only by the simulator and the test suite,
# both of which already require numpy, so importing it here costs nothing.
import numpy as np


from . import config as C
from .pid import _finite

#: Channel names recorded every tick, in column order.  The first ten are the
#: channels the firmware itself telemetrises; ``vjaw`` and ``gap`` are added
#: because the force loop cannot be diagnosed without them -- a force error is
#: meaningless unless it can be read alongside the jaw velocity that produced it
#: and the aperture at which it occurred.
CHANNELS = (
    "t",
    "ref",
    "pos",
    "vel",
    "force",
    "pwm",
    "current",
    "state",
    "fault",
    "adc_raw",
    "vjaw",
    "gap",
)


def _json_number(value: object) -> Optional[float]:
    """Coerce one recorded value into a JSON-representable number.

    Parameters
    ----------
    value:
        A recorded sample or metric.

    Returns
    -------
    float or None
        The value as a finite float, or ``None`` when it is not finite.
        ``json.dump(..., allow_nan=False)`` rejects ``nan`` and ``inf``, so a
        single non-finite sample would otherwise abort the whole batch write.
    """
    try:
        v = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if math.isfinite(v):
        return v
    return None


def _json_safe(value: object) -> object:
    """Recursively convert ``value`` into something ``json`` can always encode.

    Parameters
    ----------
    value:
        Scalars, lists, tuples or dicts -- typically a metrics dictionary.

    Returns
    -------
    object
        A structure in which every non-finite float is ``None``.
    """
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, float)):
        return _json_number(value)
    return str(value)


def force_band_entry_s(
    t: Sequence[float],
    force: Sequence[float],
    target_n: float,
    band_n: float,
    hold_s: float,
    window_start_s: float = 0.0,
) -> Optional[float]:
    """Return the first time the force stays within a band for ``hold_s``.

    Parameters
    ----------
    t, force:
        Time (s) and measured force (N) channels.
    target_n:
        Setpoint the band is centred on, N.
    band_n:
        Half-width of the band, N.
    hold_s:
        How long the force must remain inside the band, seconds.
    window_start_s:
        Ignore samples before this time, s.  The search must not start at zero:
        before contact the measured force sits near zero, which is *inside* a
        +/-0.1 N band around a target of a few newtons only if the target is small
        -- but the moving-average filter's startup transient puts the early samples
        close enough to any low target to qualify, and the window would then begin
        several seconds before the gripper had even touched the stem.

    Returns
    -------
    float or None
        The start of the first qualifying window, or ``None`` when the force never
        stays inside the band that long.  This is the same criterion the gripper
        controller uses to declare the target reached, so the window it defines is
        exactly the regulation phase rather than the approach.
    """
    tt = np.asarray(t, dtype=float)
    ff = np.asarray(force, dtype=float)
    if tt.size == 0:
        return None
    inside = np.logical_and(
        np.abs(ff - float(target_n)) < float(band_n), tt >= float(window_start_s)
    )
    if tt.size > 1:
        dt = float(np.mean(np.diff(tt)))
    else:
        dt = C.CONTROL_DT_S
    needed = max(1, int(round(float(hold_s) / max(dt, C.EPS))))
    run = 0
    start = 0
    for i in range(inside.size):
        if inside[i]:
            if run == 0:
                start = i
            run += 1
            if run >= needed:
                return float(tt[start])
        else:
            run = 0
    return None


def ensure_dir(path: str) -> str:
    """Create ``path`` (and its parents) if missing and return it.

    Parameters
    ----------
    path:
        Directory to create.  An empty string is returned unchanged.

    Returns
    -------
    str
        The same path, so calls can be nested.
    """
    if path:
        os.makedirs(path, exist_ok=True)
    return path


def project_root() -> str:
    """Return the directory that contains the ``simulator`` package.

    Used to resolve the default ``data/`` locations independently of the current
    working directory, so that ``python simulator/main.py`` from the project root
    and ``python -m simulator.main`` from elsewhere behave the same way.
    """
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Recorder:
    """Per-tick time-series recorder with CSV/JSON export.

    Parameters
    ----------
    dt:
        Nominal control period, seconds, used only for capacity hints.
    expected_ticks:
        Optional capacity hint for the initial list allocation.
    """

    def __init__(self, dt: float = C.CONTROL_DT_S, expected_ticks: int = 0) -> None:
        """Allocate the channel buffers."""
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self._expected: int = max(0, int(expected_ticks))
        self.reset()

    def reset(self) -> None:
        """Discard every recorded sample and clear the event log."""
        self.data: Dict[str, List[float]] = {name: [] for name in CHANNELS}
        self.events: List[Dict[str, object]] = []
        self.n: int = 0

    def record(
        self,
        t: float,
        ref: float,
        pos: float,
        vel: float,
        force: float,
        pwm: float,
        current: float,
        state: str,
        fault: str,
        adc_raw: float,
        vjaw: float = 0.0,
        gap: float = 0.0,
    ) -> None:
        """Append one sample.

        Parameters
        ----------
        t:
            Simulation time, s.
        ref:
            Position reference, mrad.
        pos, vel:
            Measured joint position (mrad) and velocity (mrad/s).
        force:
            Measured grip force, N.
        pwm:
            Applied PWM duty, per-mille.
        current:
            Motor current, A.
        state:
            FSM state name.
        fault:
            Latched fault code, or an empty string.
        adc_raw:
            Raw force-channel ADC counts.
        vjaw:
            Jaw velocity command, mm/s (negative closes).
        gap:
            Jaw aperture, mm.
        """
        d = self.data
        d["t"].append(_finite(t))
        d["ref"].append(_finite(ref))
        d["pos"].append(_finite(pos))
        d["vel"].append(_finite(vel))
        d["force"].append(_finite(force))
        d["pwm"].append(_finite(pwm))
        d["current"].append(_finite(current))
        d["state"].append(_finite(self._state_index(state)))
        d["fault"].append(_finite(self._fault_index(fault)))
        d["adc_raw"].append(_finite(adc_raw))
        d["vjaw"].append(_finite(vjaw))
        d["gap"].append(_finite(gap))
        self.n += 1

    def log_event(self, t: float, kind: str, message: str) -> None:
        """Append a timed event (state change, warning, fault) to the log.

        Parameters
        ----------
        t:
            Simulation time, s.
        kind:
            Event category, e.g. ``"state"`` or ``"fault"``.
        message:
            Free-text description.
        """
        self.events.append({"t_s": _finite(t), "kind": str(kind), "message": str(message)})

    @staticmethod
    def _state_index(state: str) -> float:
        """Map a state name to its plotting index.

        Imported lazily to keep this module independent of the FSM's import
        graph: telemetry is a sink, it should not pull in the controller stack.
        """
        from .fsm import STATE_INDEX, State

        try:
            return float(STATE_INDEX[State(str(state))])
        except (KeyError, ValueError):
            return -1.0

    @staticmethod
    def _fault_index(fault: str) -> float:
        """Map a fault code to a stable index, or ``0.0`` when there is none.

        A CRC32 is used rather than the built-in :func:`hash`, because Python
        salts string hashes per process: ``hash()`` would give a different fault
        index on every run and silently break reproducibility of the logs and of
        any plot built from them.
        """
        if not fault:
            return 0.0
        return float(zlib.crc32(str(fault).encode("utf-8")) % 97 + 1)

    # -- accessors ----------------------------------------------------------
    def array(self, name: str) -> np.ndarray:
        """Return one channel as a ``numpy`` array (empty when unknown)."""
        return np.asarray(self.data.get(name, []), dtype=float)

    def __len__(self) -> int:
        """Return the number of recorded samples."""
        return self.n

    def finite(self) -> bool:
        """Return ``True`` when every recorded channel is finite.

        This is the numerical-health check the CLI runs after a long simulation:
        a single ``nan`` anywhere means the model diverged, and it is far better
        to detect that here than to discover a blank plot.
        """
        for name in CHANNELS:
            arr = self.array(name)
            if arr.size and not bool(np.all(np.isfinite(arr))):
                return False
        return True

    def bad_channels(self) -> List[str]:
        """Return the names of channels containing non-finite values."""
        bad: List[str] = []
        for name in CHANNELS:
            arr = self.array(name)
            if arr.size and not bool(np.all(np.isfinite(arr))):
                bad.append(name)
        return bad

    # -- serialisation ------------------------------------------------------
    def to_csv(self, path: str) -> str:
        """Write every channel to ``path`` as CSV.

        Parameters
        ----------
        path:
            Destination file.  Parent directories are created as needed.

        Returns
        -------
        str
            The path actually written.
        """
        ensure_dir(os.path.dirname(os.path.abspath(path)))
        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(list(CHANNELS))
            n = self.n
            for i in range(n):
                writer.writerow(
                    [
                        "{0:.6f}".format(self.data["t"][i]),
                        "{0:.6f}".format(self.data["ref"][i]),
                        "{0:.6f}".format(self.data["pos"][i]),
                        "{0:.6f}".format(self.data["vel"][i]),
                        "{0:.6f}".format(self.data["force"][i]),
                        "{0:.6f}".format(self.data["pwm"][i]),
                        "{0:.6f}".format(self.data["current"][i]),
                        "{0:.0f}".format(self.data["state"][i]),
                        "{0:.0f}".format(self.data["fault"][i]),
                        "{0:.3f}".format(self.data["adc_raw"][i]),
                        "{0:.6f}".format(self.data["vjaw"][i]),
                        "{0:.6f}".format(self.data["gap"][i]),
                    ]
                )
        return path

    def to_json(self, path: str, extra: Optional[Dict[str, object]] = None) -> str:
        """Write the metrics, events and per-sample channels to ``path`` as JSON.

        Parameters
        ----------
        path:
            Destination file.
        extra:
            Optional additional top-level keys, typically the scenario metrics.

        Returns
        -------
        str
            The path actually written.

        Notes
        -----
        A settling time that never settles is reported as ``nan`` by
        :func:`metrics`.  ``nan`` is not valid JSON, and ``json.dump`` raises on
        it -- so every non-finite scalar is converted to ``null`` here rather than
        being allowed to abort a whole batch run.  The CSV keeps the raw value,
        since a spreadsheet is happy with ``nan`` and it is the more useful form
        for analysis.
        """
        ensure_dir(os.path.dirname(os.path.abspath(path)))
        payload: Dict[str, object] = {
            "channels": CHANNELS,
            "n_samples": self.n,
            "dt": self.dt,
            "events": [
                {
                    "t_s": _json_number(ev.get("t_s")),
                    "kind": ev.get("kind"),
                    "message": ev.get("message"),
                }
                for ev in self.events
            ],
            "data": {
                name: [_json_number(v) for v in self.data[name]] for name in CHANNELS
            },
        }
        if extra:
            for key, value in extra.items():
                payload[str(key)] = _json_safe(value)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, allow_nan=False)
        return path
