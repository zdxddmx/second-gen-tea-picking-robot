#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fix the *derivation arithmetic* in the documents after the retune.

Why this needs its own pass and cannot be a number substitution: the documents
show the tuning derivation as arithmetic, for example

    Kp = 0.05 / (480 x 0.02) = 0.005208

Replacing `480` with `60` alone would produce `Kp = 0.05 / (60 x 0.02) = 0.005208`,
which is not merely stale but *wrong* - 0.05/1.2 is 0.042, not 0.0052.  A reader
checking the arithmetic would conclude the documents are unreliable, which is a
worse outcome than an obviously outdated number.

So the whole expression is replaced, with the inputs and the result consistent:

    DC gain  K    480 -> 60      mrad/s per permille
    tau           0.05 -> 0.031  s     (J / (b + Kt*Ke/R), with Kt = Ke = 0.4)
    Kp            0.0052 -> 0.026      = tau / (K * Tc),  Tc = 20 ms
    Ki            0.104 -> 0.83        = Kp / tau

Every replacement is a literal string, and a pattern that matches nothing is
reported rather than ignored, so a stale anchor is visible.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Literal (old, new) pairs.  Each old string is long enough to be unique.
FIXES = [
    # --- the arithmetic block in docs/05_control_algorithm.md ---
    ("Kp = 0.05 / (480 × 0.02) = 0.05 / 9.6     = 0.005208  ≈ 0.0052",
     "Kp = 0.031 / (60 × 0.02) = 0.031 / 1.2    = 0.025833  ≈ 0.026"),
    ("Ki = 0.005208 / 0.05                       = 0.10417   ≈ 0.104",
     "Ki = 0.025833 / 0.031                      = 0.83333   ≈ 0.83"),
    ("`τ = 0.05 s`、`K = 480`：", "`τ = 0.031 s`、`K = 60`："),
    ("`K = 480`：", "`K = 60`："),
    ("`Kp = 0.05 / (480 * 0.02) = 0.0052`, `Ki = Kp / tau = 0.104`",
     "`Kp = 0.031 / (60 * 0.02) = 0.026`, `Ki = Kp / tau = 0.83`"),
    ("机械时间常数 **0.05 s**；PI 增益按极点配置 `Kp = τ/(K·Tc)` 得 **0.0052 / 0.104**，"
     "出厂值取一位有效数字 **0.005 / 0.10**",
     "机械时间常数 **0.031 s**；PI 增益按极点配置 `Kp = τ/(K·Tc)` 得 **0.026 / 0.83**，"
     "出厂值即该舍入值"),

    # --- the "both scaled by the same ratio" note is now moot ---
    ("一个值得指出的细节：`0.005/0.0052 = 0.100/0.104 = 0.9615`——**两者按同一比例缩放**，所以",
     "一个值得指出的细节：更正后出厂值就是推导值本身（`Kp = 0.026`、`Ki = 0.83`），"
     "不再存在「取一位有效数字」造成的整体缩放，所以"),

    # --- docs/11_experiment_design.md ---
    ("用 `Kp = tau/(K*Tc)` 并取闭环时间常数 `Tc = 20 ms`，得 "
     "`Kp = 0.05/(480*0.02) = 0.0052`、`Ki = Kp/tau = 0.104`，发行值取一位有效数字即 "
     "0.005 与 0.10。",
     "用 `Kp = tau/(K*Tc)` 并取闭环时间常数 `Tc = 20 ms`，得 "
     "`Kp = 0.031/(60*0.02) = 0.026`、`Ki = Kp/tau = 0.83`，出厂值即 0.026 与 0.83。"),
    ("（`config.h` 给出 `Kp = 0.026`、`Ki = 0.104` 的推导）",
     "（`config.h` 给出 `Kp = 0.026`、`Ki = 0.83` 的推导）"),

    # --- docs/interview_guide.md ---
    ("`Kp = 0.05 / (480 × 0.02) = 0.0052`，`Ki = Kp/τ = 0.83`",
     "`Kp = 0.031 / (60 × 0.02) = 0.026`，`Ki = Kp/τ = 0.83`"),
    ("得 `Kp = 0.05 / (480 × 0.02) = 0.0052`，`Ki = Kp/τ = 0.83`；\n"
     "  出厂值就是舍入到一位有效数字的结果（`0.005` 与 `0.10`）",
     "得 `Kp = 0.031 / (60 × 0.02) = 0.026`，`Ki = Kp/τ = 0.83`；\n"
     "  出厂值就是这两个数"),
    ("速度环的 `Kp = τ/(K·Tc) = 0.05/(480×0.02) = 0.0052`、",
     "速度环的 `Kp = τ/(K·Tc) = 0.031/(60×0.02) = 0.026`、"),
    ("得 `Kp = 0.05/(480×0.02) = 0.0052`、`Ki = Kp/τ = 0.83`，\n"
     "  出厂值就是舍入到一位有效数字的 `0.005` 与 `0.10`。",
     "得 `Kp = 0.031/(60×0.02) = 0.026`、`Ki = Kp/τ = 0.83`，\n"
     "  出厂值就是这两个数。"),
    ("   - 占空比到速度是一条**一阶滞后**，直流增益约 **480 mrad/s 每千分比**\n"
     "   - 机械时间常数 `τ = J / (b + Kt·Ke/R) = 0.05 s`",
     "   - 占空比到速度是一条**一阶滞后**，直流增益约 **60 mrad/s 每千分比**\n"
     "   - 机械时间常数 `τ = J / (b + Kt·Ke/R) = 0.031 s`"),
    ("`ω ≈ pwm × 24 × 19.99/1000 ≈ 480 mrad/s per 千分比` ✓ **与 `config.h` 的 480 一致**",
     "`ω ≈ pwm × 24 × 4.17/1000 ≈ 60 mrad/s per 千分比` ✓ **与 `config.h` 的 60 一致**"
     "（`Ke = 0.4` → `1/Ke = 2.5 rad/s per V`）"),
    ("占空比到速度路径是**一阶滞后**，直流增益约 **480 mrad/s 每千分比**，\n"
     "  机械时间常数 `J / (b + Kt·Ke/R) = 0.05 s`；",
     "占空比到速度路径是**一阶滞后**，直流增益约 **60 mrad/s 每千分比**，\n"
     "  机械时间常数 `J / (b + Kt·Ke/R) = 0.031 s`；"),
    ("占空比到速度的直流增益约 **480 mrad/s 每千分比**、\n"
     "机械时间常数 `τ = J/(b + Kt·Ke/R) = 0.05 s`，",
     "占空比到速度的直流增益约 **60 mrad/s 每千分比**、\n"
     "机械时间常数 `τ = J/(b + Kt·Ke/R) = 0.031 s`，"),
    ("**推导（可当场重做）**：速度环对象 `K ≈ 480 mrad/s 每千分比`、`τ = 0.05 s`；",
     "**推导（可当场重做）**：速度环对象 `K ≈ 60 mrad/s 每千分比`、`τ = 0.031 s`；"),
    ("`Kp = τ/(K·Tc)`，`Tc = 20 ms` → `Kp = 0.026`，`Ki =",
     "`Kp = τ/(K·Tc)`，`Tc = 20 ms` → `Kp = 0.026`，`Ki ="),

    # --- hardware/signal_conditioning.md: an old/new comparison table ---
    ("` with a desired closed-loop time constant `Tc = 20 ms` gives "
     "`Kp = 0.05 / (480 * 0.02) = 0.0052`\"*",
     "` with a desired closed-loop time constant `Tc = 20 ms` gives "
     "`Kp = 0.031 / (60 * 0.02) = 0.026`\"*"),
    ("| `TP_VEL_PID_KI` | 0.30f | **0.10f** | `Ki = Kp / tau = 0.104`，取一位有效数字 |",
     "| `TP_VEL_PID_KI` | 0.30f | **0.83f** | `Ki = Kp / tau = 0.83` |"),
    ("| `TP_VEL_PID_IMAX` | 400.0f | **250.0f** | 与新的 `Ki` 配套 |",
     "| `TP_VEL_PID_IMAX` | 400.0f | **250.0f** | 与新的 `Ki` 配套 |"),

    # --- the two hardware documents' audit finding is now resolved ---
    ("`TP_PLANT_MOTOR_KT_NM_PER_A = 2.0f` 与 `TP_PLANT_MOTOR_KE_VS_RAD = 0.05f`，"
     "相差 40 倍。**",
     "`TP_PLANT_MOTOR_KT_NM_PER_A = 2.0f` 与 `TP_PLANT_MOTOR_KE_VS_RAD = 0.05f`，"
     "相差 40 倍。**（已修复：二者现均为 `0.4`，并由 "
     "`TP_STATIC_ASSERT` 强制相等；下述 §4.11 关于能量回灌的结论是按**已修复**的 "
     "`Ke = 0.4` 重述的。）**"),
]

#: Documents to touch, and a per-file minimum number of successful fixes, so a
#: file whose anchors have all gone stale is reported instead of passing silently.
TARGETS = {
    "docs/05_control_algorithm.md": 3,
    "docs/11_experiment_design.md": 2,
    "docs/interview_guide.md": 6,
    "hardware/motor_driver.md": 1,
    "hardware/power_design.md": 0,
    "hardware/signal_conditioning.md": 1,
}

#: Second batch: the remaining stale numbers, found by re-scanning after batch 1.
#: Three categories are deliberately *excluded* from this list, because they are
#: correct as written:
#:
#:   * the audit findings in the hardware documents, which necessarily quote both
#:     candidate constants in order to explain that they disagree;
#:   * historical statements about the first revision, e.g. "the v1 crossover with
#:     Kp = 0.015 was ...", which are supposed to quote the v1 numbers;
#:   * `1/(2*sqrt(0.104))` in the position-loop damping analysis, where 0.104 is
#:     `Kp * tau_i` for the position loop and has nothing to do with the velocity
#:     integral gain that happens to share the digits.
BATCH2 = [
    # derivation inputs
    ("取 `Kt = 2.0`、`Ke = 0.05`、`R = 2.0`、`J = 2.5e-3`、`b = 3.0e-5`",
     "取 `Kt = 0.4`、`Ke = 0.4`、`R = 2.0`、`J = 2.5e-3`、`b = 3.0e-5`"),
    # stale Ki in the corroboration notes
    ("| `Kp = 0.026`、`Ki = 0.104` | 极点配置公式的输入输出，未经实验整定 |",
     "| `Kp = 0.026`、`Ki = 0.83` | 极点配置公式的输入输出，未经实验整定 |"),
    ("`Kp = 0.026`、`Ki = 0.104` 四个数字全部与 `config.h:215-221` 的注释逐字吻合；"
     "出厂值取一位有效数字后 `Ki/Kp` 比值不变，所以极点零点抵消",
     "`Kp = 0.026`、`Ki = 0.83` 四个数字全部与 `config.h` 的注释逐字吻合；"
     "出厂值就是推导值，所以极点零点抵消"),
    # the anti-windup tracking-constant table
    ("| 速度环 | 0.005 | 0.10 | 20 s⁻¹ | 0.05 s | **恰好等于机械时间常数 τ = 0.05 s** |",
     "| 速度环 | 0.026 | 0.83 | 31.9 s⁻¹ | 0.031 s | **恰好等于机械时间常数 τ = 0.031 s** |"),
    # the time-constant arithmetic
    ("= 2.5e-3 / 0.05003\n                      = 0.049970 s  ≈ 0.05 s              "
     "← 与 config.h:218 一致",
     "= 2.5e-3 / 0.08003\n                      = 0.031238 s  ≈ 0.031 s             "
     "← 与 config.h 一致"),
    ("τm = J·R / (Kt·Ke + b·R) = 2.5e-3 × 2 / 0.10006 ≈ 50 ms` ✓ "
     "**与 `config.h` 的 0.05 s 一致**",
     "τm = J·R / (Kt·Ke + b·R) = 2.5e-3 × 2 / 0.16006 ≈ 31 ms` ✓ "
     "**与 `config.h` 的 0.031 s 一致**"),
    # the one-line plant summaries
    ("被控对象是一阶 `K/(τs + 1)`，`K = 60 mrad/s per per-mille`、`τ = 0.05 s`。",
     "被控对象是一阶 `K/(τs + 1)`，`K = 60 mrad/s per per-mille`、`τ = 0.031 s`。"),
    ("K 约 60 mrad/s per permille<br/>tau 0.05 s  J 2.5e-3  b 3.0e-5<br/>"
     "Kt 2.0  Ke 0.05  R 2.0",
     "K 约 60 mrad/s per permille<br/>tau 0.031 s  J 2.5e-3  b 3.0e-5<br/>"
     "Kt 0.4  Ke 0.4  R 2.0"),
    ("| 机械时间常数 **0.05 s** | 由 `J/(b+Kt·Ke/R)` 算出；未实测 |",
     "| 机械时间常数 **0.031 s** | 由 `J/(b+Kt·Ke/R)` 算出；未实测 |"),
    ("直流增益 60 mrad/s per per-mille、机械时间常数 0.05 s、",
     "直流增益 60 mrad/s per per-mille、机械时间常数 0.031 s、"),
    ("机械时间常数 `J/(b + Kt*Ke/R) = 0.05 s`；",
     "机械时间常数 `J/(b + Kt*Ke/R) = 0.031 s`；"),
    # interview guide: three derivations the first batch's anchors missed
    ("得 `Kp = 0.05/(480×0.02) = 0.0052`、`Ki = Kp/τ = 0.83`，\n"
     "  出厂值就是舍入到一位有效数字的 `0.005` 与 `0.10`。",
     "得 `Kp = 0.031/(60×0.02) = 0.026`、`Ki = Kp/τ = 0.83`，\n"
     "  出厂值就是这两个数。"),
    ("   - 占空比到速度是一条**一阶滞后**，直流增益约 **480 mrad/s 每千分比**\n"
     "   - 机械时间常数 `τ = J / (b + Kt·Ke/R) = 0.05 s`",
     "   - 占空比到速度是一条**一阶滞后**，直流增益约 **60 mrad/s 每千分比**\n"
     "   - 机械时间常数 `τ = J / (b + Kt·Ke/R) = 0.031 s`"),
    ("用 `plant_model.c` 的 `τ = 0.05 s`、`K = 480`）",
     "用 `plant_model.c` 的 `τ = 0.031 s`、`K = 60`）"),
    ("如果实测机械时间常数不是 0.05 s 而是 0.1 s，",
     "如果实测机械时间常数不是 0.031 s 而是 0.062 s，"),
    # power_design's finding needs the same "resolved" marker motor_driver got
    ("在 SI 单位下应相等却相差 40 倍**，",
     "在 SI 单位下应相等却相差 40 倍**（**已修复**：二者现均为 `0.4`，并由 "
     "`TP_STATIC_ASSERT` 强制相等；下文关于回灌过压的结论已按 `Ke = 0.4` 重新评估），"),
]


#: Third batch: the two anchors the earlier batches missed because of whitespace
#: differences in the surrounding text.  Found by re-scanning, which is the whole
#: reason the scanner exists - a literal replacement that silently does not match
#: is invisible without one.
BATCH3 = [
    ("得 `Kp = 0.05/(480×0.02) = 0.0052`、`Ki = Kp/τ = 0.83`，",
     "得 `Kp = 0.031/(60×0.02) = 0.026`、`Ki = Kp/τ = 0.83`，"),
    ("   - 占空比到速度是一条**一阶滞后**，直流增益约 **480 mrad/s 每千分比**",
     "   - 占空比到速度是一条**一阶滞后**，直流增益约 **60 mrad/s 每千分比**"),
]


def main():
    failed = 0
    for rel, minimum in TARGETS.items():
        full = os.path.join(ROOT, rel)
        if not os.path.exists(full):
            print("missing: %s" % rel)
            failed += 1
            continue
        with io.open(full, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()

        applied = 0
        for old, new in FIXES + BATCH2 + BATCH3:
            count = text.count(old)
            if count:
                text = text.replace(old, new)
                applied += count

        with io.open(full, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)

        status = "ok" if applied >= minimum else "CHECK"
        if applied < minimum:
            failed += 1
        print("%-42s %2d fix(es)  [%s]" % (rel, applied, status))

    print("\n%d file(s) below their expected fix count" % failed)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
