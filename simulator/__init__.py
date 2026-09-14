"""
Self-contained simulation package for the second-generation tea-picking robot.

What
----
A deterministic, dependency-light simulator of the robot described by
``firmware/Application/config.h``.  It contains a plant model (motor joint and
flexible stem gripper), a PID library that mirrors the C implementation, a
table-driven robot state machine, cascade and force controllers, seven named
scenarios, telemetry, and plotting.

Why
---
The firmware's control behaviour cannot be regression-tested on hardware at
1 kHz, one scenario at a time, with a reproducible outcome.  This package can.
It deliberately mirrors the firmware's constants *and* its arithmetic (see
:mod:`simulator.pid`) so that a discrepancy between the two is a genuine finding
rather than a units or gain-scaling artefact.

What happens without it
-----------------------
Gains, force limits and safety thresholds could only be validated on a physical
machine, and a change to the grip force or the anti-windup strategy would be
judged by eye instead of against recorded metrics.

Entry point
-----------
``python simulator/main.py [--scenario A|...|all]``
"""

from __future__ import annotations

from typing import List

__version__ = "2.0.0"

#: Firmware version whose configuration this simulator mirrors.
__firmware_version__ = "2.0.0"

__all__: List[str] = [
    "config",
    "pid",
    "plant",
    "profiles",
    "fsm",
    "controller",
    "telemetry",
    "plotting",
    "scenarios",
    "main",
    "__version__",
]
