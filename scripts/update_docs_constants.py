#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bring the documentation in line with the corrected constants.

Why an anchored substitution script rather than a hand edit of nineteen
documents: the documents quote the same twenty numbers, and a manual pass would
miss some and transcribe others wrongly.  A script reports the match count for
every pattern, so a pattern that no longer matches anything is *visible* rather
than silently doing nothing.

Why the patterns are narrow: a blind global replace of, say, `0.015` would also
rewrite unrelated numbers - the stem stiffness is 0.30 N/mm, the viscous damping
is 3e-5, and a bare `0.30` appears in both the gains and the plant.  Every
pattern below therefore includes enough context to be unambiguous, and the script
reports how many times each one matched so the result can be reviewed rather
than trusted.

What is NOT done here: the prose in the affected documents is not rewritten, and
where a derivation is stated in words rather than in numbers it may still
describe the old plant.  Each affected document therefore also gets a dated
revision notice at the top naming `config.h` and `plant_model.h` as the single
authority.  That pointer is the part that makes the stale prose survivable; the
substitutions are what stop a reader being actively misled by a number.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: (pattern, replacement).  Patterns are literal, not regular expressions, so
#: nothing can escape and hit the wrong quantity.  Only substitutions whose
#: context is unique are listed: `60 rad/s`, for instance, is deliberately *not*
#: here, because 60 rad/s is the drive's no-load speed and is still correct.
#: `0.05 s` is likewise absent: a 50 ms debounce is also written that way.
SUBSTITUTIONS = [
    # --- velocity loop, retuned when Kt was corrected ---
    ("TP_VEL_PID_KP = 0.005", "TP_VEL_PID_KP = 0.026"),
    ("TP_VEL_PID_KP = 0.005f", "TP_VEL_PID_KP = 0.026f"),
    ("TP_VEL_PID_KI = 0.10", "TP_VEL_PID_KI = 0.83"),
    ("TP_VEL_PID_KI = 0.30", "TP_VEL_PID_KI = 0.83"),
    ("Kp = 0.0052", "Kp = 0.026"),
    ("Kp=0.0052", "Kp=0.026"),
    ("Kp = 0.005 ", "Kp = 0.026 "),
    ("Ki = Kp/τ = 0.104", "Ki = Kp/τ = 0.83"),
    ("TP_VEL_PID_IMAX = 400", "TP_VEL_PID_IMAX = 250"),

    # --- plant DC gain ---
    ("480 mrad/s per per-mille", "60 mrad/s per per-mille"),
    ("480 mrad/s/permille", "60 mrad/s/permille"),
    ("480 mrad/s per permille", "60 mrad/s per permille"),

    # --- position loop, retuned in the first correction round ---
    ("TP_POS_PID_KP = 12.0", "TP_POS_PID_KP = 5.0"),
    ("TP_POS_PID_IMAX = 8000", "TP_POS_PID_IMAX = 2000"),
    ("TP_POS_PID_OUT_MAX = 60000", "TP_POS_PID_OUT_MAX = 40000"),

    # --- motor constants, corrected for the Kt == Ke identity ---
    ("Kt = 2.0 N·m/A", "Kt = 0.4 N·m/A"),
    ("Ke = 0.05 V·s/rad", "Ke = 0.4 V·s/rad"),
    ("2.0 N·m/A", "0.4 N·m/A"),
    ("0.05 V·s/rad", "0.4 V·s/rad"),

    # --- speed limit, lowered for torque margin ---
    ("TP_JOINT_MAX_SPEED_MRAD_S = 60000", "TP_JOINT_MAX_SPEED_MRAD_S = 40000"),
    ("60000 mrad/s", "40000 mrad/s"),
    ("±60000", "±40000"),
    ("60000.0f", "40000.0f"),

    # --- slew limiter (first correction round) ---
    ("TP_MAX_DUTY_RATE_PER_S = 4000", "TP_MAX_DUTY_RATE_PER_S = 200000"),
    ("4000 permille/s", "200000 permille/s"),
]

#: Frozen simulated results that changed with the retune.  These are quoted as
#: results, so leaving them would misreport what the code produces.
RESULT_SUBSTITUTIONS = [
    ("2.386 N", "2.493 N"),
    ("2.17 N", "2.26 N"),
    ("5.824 mm", "5.688 mm"),
    ("0.148 N", "0.156 N"),
]

REVISION_NOTICE = """<!-- REVISION NOTICE - do not remove -->
> ### ⚠️ 参数修订说明（第二次校正，覆盖全文数值）
>
> 本文档写作期间，项目经历了两轮**更正**，因此正文中的部分**具体数字已经过期**。
> 保留原文是为了不掩盖修正过程，但读数时请以代码为准：
>
> | 常量 | 已废弃的值 | 当前值（权威） | 更正原因 |
> |---|---|---|---|
> | `TP_PLANT_MOTOR_KT_NM_PER_A` | 2.0 | **0.4** | SI 单位下 `Kt` 与 `Ke` 是同一物理量，原值相差 40 倍，违反能量守恒 |
> | `TP_PLANT_MOTOR_KE_VS_RAD` | 0.05 | **0.4** | 同上 |
> | `TP_VEL_PID_KP` | 0.005 | **0.026** | 被控对象 DC 增益由 480 变为 60 mrad/s/permille，必须重新整定 |
> | `TP_VEL_PID_KI` | 0.10 | **0.83** | 同上 |
> | `TP_POS_PID_KP` | 12.0 | **5.0** | 第一次校正 |
> | `TP_POS_PID_IMAX` | 8000 | **2000** | 第一次校正 |
> | `TP_JOINT_MAX_SPEED_MRAD_S` | 60000 | **40000** | 原值恰等于空载转速，只能在零转矩下达到 |
> | `TP_MAX_PWM_RATE_PER_S` | 4000 | **200000** | 第一次校正：原值是速度环内的主导滞后 |
> | 场景 A 峰值夹持力 | 2.386 N | **2.493 N** | 随整定更新 |
> | 场景 A 达到目标力 | 2.17 N | **2.26 N** | 随整定更新 |
>
> **唯一权威**：`firmware/Application/config.h` 与 `firmware/BSP/host/plant_model.h`。
> 二者的一致性由 `tests/test_config_agreement.py` 自动校验——它会解析这两个头文件并与
> Python 模型逐条比对，因此任何再次漂移都会变成失败的测试而不是过期的文档。
<!-- /REVISION NOTICE -->
"""

#: Documents that quote the corrected constants and therefore get the notice.
TARGETS = [
    "docs/01_project_background.md",
    "docs/02_system_design.md",
    "docs/03_hardware_design.md",
    "docs/04_software_architecture.md",
    "docs/05_control_algorithm.md",
    "docs/06_force_control.md",
    "docs/07_communication_protocol.md",
    "docs/08_freertos_design.md",
    "docs/09_safety_design.md",
    "docs/10_simulation.md",
    "docs/11_experiment_design.md",
    "docs/12_results_analysis.md",
    "docs/13_future_work.md",
    "docs/interview_guide.md",
    "docs/resume_project.md",
    "hardware/motor_driver.md",
    "hardware/interface_design.md",
    "hardware/power_design.md",
    "hardware/signal_conditioning.md",
]


def apply_substitutions(text, table, report):
    """Apply one table of literal substitutions, recording the match counts.

    A pattern that matches nothing is not an error - most documents do not quote
    every constant - but it *is* worth knowing, because a pattern that has gone
    stale silently stops correcting anything.
    """
    for pattern, replacement in table:
        count = text.count(pattern)
        if count:
            text = text.replace(pattern, replacement)
            report.append((pattern, replacement, count))
    return text


def main():
    total_changed = 0
    for rel in TARGETS:
        full = os.path.join(ROOT, rel)
        if not os.path.exists(full):
            print("missing: %s" % rel)
            continue
        with io.open(full, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()

        # The revision notice *quotes* the superseded values in its "deprecated"
        # column, so it must be excluded from the substitution pass.  Without
        # this the notice would rewrite itself and start claiming that the old
        # value was the new one - a document that corrects a number by printing
        # the same number twice.
        if "<!-- REVISION NOTICE" in text and "<!-- /REVISION NOTICE -->" in text:
            head, rest = text.split("<!-- REVISION NOTICE", 1)
            notice, tail = rest.split("<!-- /REVISION NOTICE -->", 1)
            notice = "<!-- REVISION NOTICE" + notice + "<!-- /REVISION NOTICE -->"
        else:
            head, notice, tail = text, None, ""

        report = []
        head = apply_substitutions(head, SUBSTITUTIONS, report)
        head = apply_substitutions(head, RESULT_SUBSTITUTIONS, report)
        tail = apply_substitutions(tail, SUBSTITUTIONS, report)
        tail = apply_substitutions(tail, RESULT_SUBSTITUTIONS, report)

        if notice is None:
            # Inserted after the document's own title so it still opens normally.
            lines = head.split("\n")
            insert_at = 0
            for i, line in enumerate(lines[:40]):
                if line.startswith("# "):
                    insert_at = i + 1
                    break
            lines.insert(insert_at, "\n" + REVISION_NOTICE)
            head = "\n".join(lines)
            text = head
        else:
            text = head + notice + tail

        with io.open(full, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
        total_changed += 1
        applied = sum(n for _, _, n in report)
        print("%-42s %3d substitution(s) in %2d pattern(s)"
              % (rel, applied, len(report)))

    print("\n%d document(s) updated" % total_changed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
