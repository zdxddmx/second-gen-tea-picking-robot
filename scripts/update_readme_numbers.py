#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Update the reference numbers in README.md and README_EN.md.

Why this is a script and not a handful of editor operations: the two READMEs
quote roughly twenty numbers that all come from one run of the firmware, and
those numbers change whenever the plant model or the gains are corrected.  A
script makes the update reproducible and, more importantly, makes it *checkable*
- every replacement asserts that the text it is looking for was actually found,
so a silent no-op (which is what a stale search string produces) fails loudly
instead of leaving a document quoting numbers the code no longer produces.

The first attempt at this edit went through PowerShell's `Get-Content -Raw` /
`Set-Content` pair, which round-trips a UTF-8 file through the console code page
and destroys every non-ASCII character.  Python reads and writes bytes, so the
Chinese text in README.md survives untouched.

This revision follows the force-back-drive sign fix in `plant_model.c`, which
changed every force number in the sweep: the jaw is no longer sucked closed by
the stem, so the grip peaks lower (2.363 N against 2.493 N) and scenario D
releases the bud instead of grinding it to the sensor rail.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def edit(path, replacements):
    """Apply literal replacements, asserting each one matched exactly once."""
    full = os.path.join(ROOT, path)
    with io.open(full, "r", encoding="utf-8") as handle:
        text = handle.read()
    original = text
    problems = []
    for old, new in replacements:
        count = text.count(old)
        if count != 1:
            problems.append("  %d match(es) for: %s" % (count, old[:90]))
            continue
        text = text.replace(old, new)
    if problems:
        print("FAILED %s:" % path)
        for problem in problems:
            print(problem)
        return False
    if text == original:
        print("no change needed: %s" % path)
        return True
    with io.open(full, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    print("updated %s (%d bytes -> %d bytes)" % (path, len(original), len(text)))
    return True


# ---------------------------------------------------------------------------
# README.md (Chinese)
# ---------------------------------------------------------------------------

ZH = [
    # 30-second summary
    ("仿真：完整采摘周期成功，夹持力 **2.26 N** vs 目标 2.20 N，**过冲 0.00 N**，"
     "夹持期间平均力 **2.37 N**，峰值 **2.493 N**（硬限 5.0 N），**0 次控制周期超时**",
     "仿真：完整采摘周期成功，夹持力 **2.18 N** vs 目标 2.20 N，**过冲 0.00 N**，"
     "夹持期间平均力 **2.27 N**，峰值 **2.363 N**（硬限 5.0 N），**0 次控制周期超时**"),

    # scenario A table
    ("| **接触检测** | **2.62 s** | **间隙 5.688 mm，F = 0.156 N** |",
     "| **接触检测** | **2.62 s** | **间隙 5.682 mm，F = 0.156 N** |"),
    ("| **达到目标力** | **2.97 s** | **2.26 N**（目标 2.20 N ±0.10 N），"
     "**过冲 0.00 N**，267 个控制周期（267 ms）整定 |",
     "| **达到目标力** | **3.02 s** | **2.18 N**（目标 2.20 N ±0.10 N），"
     "**过冲 0.00 N**，319 个控制周期（319 ms）整定 |"),
    ("| 脱离 | 3.22 s | 夹持期间平均力 **2.37 N** |",
     "| 脱离 | 3.27 s | 夹持期间平均力 **2.27 N** |"),
    ("| 松开 | 4.51 s | 完全张开 |", "| 松开 | 3.27 s | 完全张开 |"),
    ("| 回零 | 4.55 s | `cycle_count = 1` |",
     "| 回零 | 4.59 s | `cycle_count = 1` |"),
    ("| 峰值夹持力 | **2.493 N**（硬限 5.00 N） |",
     "| 峰值夹持力 | **2.363 N**（硬限 5.00 N） |"),
    ("| 控制周期执行数 | 6000（1 kHz）/ 3000 传感器 / 600 通信 / 600 机器人 / 600 安全 |",
     "| 控制周期执行数 | 4890（1 kHz）/ 2445 传感器 / 489 通信 / 489 机器人 / 489 安全 |"),
    ("| 主机链路解码 | 2112 B，96 帧，**0 CRC 错误** |",
     "| 主机链路解码 | 2134 B，97 帧，**0 CRC 错误** |"),

    # seven-scenario table
    ("| A 正常采摘 | 周期完成 | 2.493 N | idle |",
     "| A 正常采摘 | 周期完成 | 2.363 N | idle |"),
    ("| C 力传感器噪声 | 周期完成 | 2.521 N | idle |",
     "| C 力传感器噪声 | 周期完成 | 2.421 N | idle |"),
    ("| D 夹持力过大 | 保护触发 | 5.013 N | error（`ERR_FORCE_OVERLOAD`） |",
     "| D 夹持力过大 | 保护触发 | 5.027 N | error（`ERR_FORCE_OVERLOAD`） |"),
]

# ---------------------------------------------------------------------------
# README_EN.md
# ---------------------------------------------------------------------------

EN = [
    ("Simulation: full pick cycle completes with a grip force of **2.26 N** against "
     "a 2.20 N setpoint, **0.00 N overshoot**, **2.37 N** mean hold force and a "
     "**2.493 N** peak (5.0 N hard limit)",
     "Simulation: full pick cycle completes with a grip force of **2.18 N** against "
     "a 2.20 N setpoint, **0.00 N overshoot**, **2.27 N** mean hold force and a "
     "**2.363 N** peak (5.0 N hard limit)"),

    ("| **Contact detected** | **2.62 s** | **gap 5.688 mm, F = 0.156 N** |",
     "| **Contact detected** | **2.62 s** | **gap 5.682 mm, F = 0.156 N** |"),
    ("| **Target force reached** | **2.97 s** | **2.26 N** against 2.20 N ± 0.10 N, "
     "**0.00 N overshoot**, 267 control periods (267 ms) to settle |",
     "| **Target force reached** | **3.02 s** | **2.18 N** against 2.20 N ± 0.10 N, "
     "**0.00 N overshoot**, 319 control periods (319 ms) to settle |"),
    ("| Detach | 3.22 s | mean force while gripping **2.37 N** |",
     "| Detach | 3.27 s | mean force while gripping **2.27 N** |"),
    ("| Release | 4.51 s | fully open |", "| Release | 3.27 s | fully open |"),
    ("| Home | 4.55 s | `cycle_count = 1` |",
     "| Home | 4.59 s | `cycle_count = 1` |"),
    ("| Peak grip force | **2.493 N** (hard limit 5.00 N) |",
     "| Peak grip force | **2.363 N** (hard limit 5.00 N) |"),
    ("| Host-link decode | 2112 B, 96 frames, **0 CRC errors** |",
     "| Host-link decode | 2134 B, 97 frames, **0 CRC errors** |"),

    ("| A nominal pick | cycle completed | 2.493 N | idle |",
     "| A nominal pick | cycle completed | 2.363 N | idle |"),
    ("| C force sensor noise | cycle completed | 2.521 N | idle |",
     "| C force sensor noise | cycle completed | 2.421 N | idle |"),
    ("| D excessive grip force | protection tripped | 5.013 N | "
     "error (`ERR_FORCE_OVERLOAD`) |",
     "| D excessive grip force | protection tripped | 5.027 N | "
     "error (`ERR_FORCE_OVERLOAD`) |"),
]


def main():
    ok = edit("README.md", ZH)
    ok = edit("README_EN.md", EN) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
