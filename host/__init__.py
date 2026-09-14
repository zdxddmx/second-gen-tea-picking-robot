# -*- coding: utf-8 -*-
"""PC-side host tools for the 2nd-gen tea-picking robot.

Public modules
--------------
``host.protocol``       -- shared serial frame codec (CRC-16/MODBUS)
``host.link``           -- transport abstraction: serial / TCP / loopback
``host.sim_robot``      -- tool-side robot model for ``--demo`` runs
``host.robot_monitor``  -- Tkinter + headless monitoring dashboard
``host.selftest``       -- one-shot programmatic test suite

Nothing in this package imports ``tkinter`` at import time and nothing imports
``pytest``; both are hard requirements of the target machine (Python 3.8.0,
no network, no display guaranteed).
"""

__version__ = "1.0.0"
