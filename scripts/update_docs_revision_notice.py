#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Extend the revision notice in every document to cover the third correction.

Each document in `docs/` and `hardware/` opens with a `REVISION NOTICE` block
whose table lists the constants and result values that have been superseded,
because the prose was written before the corrections and is kept rather than
rewritten (rewriting it would erase the evidence that the correction happened).
That convention only works if the table is complete: a reader who trusts the
table and finds a value in the prose that is in neither column has no way to
tell which one is current.

So this script does not hunt down every stale number in twenty documents - it
extends the authoritative table in all of them at once, which is what the
convention actually requires.  The change is identical in every file, and the
script asserts that the exact text it replaces occurs exactly once per file, so
a document whose notice has drifted fails loudly instead of being skipped.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

OLD_ROWS = (
    "> | 场景 A 峰值夹持力 | 2.386 N | **2.493 N** | 随整定更新 |\n"
    "> | 场景 A 达到目标力 | 2.17 N | **2.26 N** | 随整定更新 |"
)

NEW_ROWS = (
    "> | 场景 A 峰值夹持力 | 2.493 N | **2.363 N** | 第三次校正：力背驱符号修正 |\n"
    "> | 场景 A 达到目标力 | 2.26 N | **2.18 N** | 同上 |\n"
    "> | 场景 A 接触间隙 | 5.688 mm | **5.682 mm** | 同上 |\n"
    "> | 场景 A 主机链路解码 | 2200 B / 100 帧 | **2134 B / 97 帧** | 同上 |\n"
    "> | `TP_PLANT_JAW_BACKDRIVE` 的作用方向 | 减去（`-=`） | **加上（`+=`）** | "
    "符号错误使茎秆把空夹爪\"吸\"到机械硬限：力从 5.0 N 涨到 16.4 N，并误锁存传感器故障 |\n"
    "> | 力环服务速率 | 100 Hz（与注释不符） | **1 kHz** | "
    "力环搬入 1 kHz 控制任务，\"N 个控制周期\"从此等于 N ms |"
)

OLD_HEADING = "> ### ⚠️ 参数修订说明（第二次校正，覆盖全文数值）"
NEW_HEADING = "> ### ⚠️ 参数修订说明（第三次校正，覆盖全文数值）"

OLD_INTRO = ("> 本文档写作期间，项目经历了两轮**更正**，因此正文中的部分**具体数字已经过期**。\n"
             "> 保留原文是为了不掩盖修正过程，但读数时请以代码为准：")
NEW_INTRO = ("> 本文档写作期间，项目经历了三轮**更正**，因此正文中的部分**具体数字已经过期**。\n"
             "> 保留原文是为了不掩盖修正过程，但读数时请以代码为准。\n"
             "> **第三次更正的完整缺陷清单见 [`docs/12_results_analysis.md`](12_results_analysis.md) 的 §3.5。**")


def process(path):
    """Return (state, note) for one document.

    ``state`` is ``True`` when the file was updated, ``False`` when it was
    already current, and ``None`` when it has a revision notice this script does
    not recognise (which is reported rather than skipped silently).

    Idempotence matters for a script that is checked in: a second run must be a
    no-op, not a failure.  The marker is the third-revision heading, because a
    document either has it or has the second-revision one.
    """
    with io.open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    if "REVISION NOTICE" not in text:
        return False, "no revision notice"
    if NEW_HEADING in text and NEW_ROWS in text:
        return False, "already at the third revision"

    problems = []
    for old, new in ((OLD_ROWS, NEW_ROWS),
                     (OLD_HEADING, NEW_HEADING),
                     (OLD_INTRO, NEW_INTRO)):
        count = text.count(old)
        if count != 1:
            problems.append("%d occurrence(s) of %s"
                            % (count, _safe(old[:50])))
            continue
        text = text.replace(old, new)
    if problems:
        return None, "; ".join(problems)

    with io.open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    return True, "notice extended"


def _safe(text):
    """Render text for a console that may not be UTF-8.

    The console here is CP936; printing a warning sign from a document heading
    raises ``UnicodeEncodeError`` and would turn a reported problem into a crash
    - which is the failure mode this script exists to avoid.
    """
    try:
        text.encode(sys.stdout.encoding or "utf-8")
        return text
    except (UnicodeEncodeError, LookupError):
        return text.encode("ascii", "backslashreplace").decode("ascii")


def main():
    failures = []
    changed = 0
    for folder in ("docs", "hardware"):
        base = os.path.join(ROOT, folder)
        for name in sorted(os.listdir(base)):
            if not name.endswith(".md"):
                continue
            path = os.path.join(base, name)
            result, note = process(path)
            rel = "%s/%s" % (folder, name)
            if result is True:
                changed += 1
                print("extended %s" % rel)
            elif result is None:
                failures.append("%s: %s" % (rel, note))
    for failure in failures:
        print("FAILED %s" % _safe(failure))
    print("-" * 70)
    print("%d document(s) updated, %d problem(s)" % (changed, len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
