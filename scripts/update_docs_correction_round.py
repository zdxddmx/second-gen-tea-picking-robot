#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-align docs/12_results_analysis.md with the corrected scenario A run.

Why a script: this document quotes about forty numbers, every one of them taken
from a firmware trace, and the correction round that fixed the plant model and
the gains moved all of them.  Hand-editing forty numbers in a file that also
contains Chinese text is how the numbers and the prose drift apart.  Every
replacement below asserts that it matched exactly once, so a stale search string
fails the whole run instead of silently doing nothing.

The edits also *replace* two subsections that were analyses of defects since
fixed, rather than deleting them: the stale text is kept as a record of what was
wrong, with the current behaviour stated next to it.  A correction that erases
its own history is not verifiable by the reader.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "docs", "12_results_analysis.md")


def apply(text, replacements):
    problems = []
    for old, new in replacements:
        count = text.count(old)
        if count != 1:
            problems.append("  %d match(es) for: %s" % (count, old[:100]))
            continue
        text = text.replace(old, new)
    return text, problems


# --- §2.1 opening paragraph: the whole timeline moved -----------------------

OLD_TIMELINE = (
    "**观测到什么。** 目标在 t = 1.20 s 被检测到；接触判定发生在夹爪间隙 **5.688 mm**、"
    "**F = 0.156 N**；力达到 **2.26 N**（设定 2.20 N，band ±0.10 N），**超调 0.00 N**，"
    "**49 个控制周期**收敛；受控阶段**平均保持力 2.26 N**；**峰值夹持力 2.493 N**"
    "（硬限 5.00 N）。节拍：APPROACH→CONTACT **1.24 s**、GRIP **2.57 s**、PICK **3.14 s**、"
    "RELEASE **3.39 s**、RETURN/IDLE **4.78 s**，`cycle_count` = 1。"
    "主机链路解码校验：**2200 B、100 帧、0 CRC 错误**。"
)

NEW_TIMELINE = (
    "**观测到什么（下面每个数字都来自 `data/logs/demo_firmware_A.log` 与对应的 CSV 轨迹，"
    "可用 `python tools/analyze_log.py data/logs/demo_firmware_A.csv` 复现）。** "
    "目标在 t = 1.20 s 被检测到；接触判定发生在夹爪间隙 **5.688 mm**、**F = 0.156 N**；"
    "力达到 **2.26 N**（设定 2.20 N，band ±0.10 N），**超调 0.00 N**，"
    "**267 个控制周期（267 ms）**收敛；受控阶段**平均保持力 2.37 N**；"
    "**峰值夹持力 2.493 N**（硬限 5.00 N）。节拍：APPROACH 结束 **1.24 s**、"
    "CONTACT_DETECT 判定 **2.62 s**、FORCE_CONTROL→PICK **2.97 s**、RELEASE **3.22 s**、"
    "RETURN/IDLE **4.55 s**，`cycle_count` = 1。"
    "主机链路解码校验：**2112 B、96 帧、0 CRC 错误**。"
)

# --- §2.1.1: the contact self-consistency arithmetic moved with the gap -----

OLD_SELFCONSIST = (
    "**自洽性核对（本文推算，常数全部来自真实代码）**：茎秆名义表面在 6.0 mm，"
    "接触判定发生在 5.688 mm，故穿透量 d = 6.0 − 5.824 = **0.176 mm**。"
    "模型 `plant_model.c:341-348` 为 `F = k1·d + k2·d² + c·max(0,−v_jaw)`，"
    "慢接近时 v_jaw = −6 mm/s，阻尼项 = 0.015 × 6 = 0.09 N：\n\n"
    "`F ≈ 0.30×0.176 + 0.06×0.176² + 0.09 = 0.0528 + 0.0019 + 0.0900 = 0.1447 N`\n\n"
    "观测值 **0.156 N**，差 3 mN，**小于一个力 ADC 量化步长 8.0 mN**"
    "（`config.h:435-446` 给出 `TP_FORCE_CAL_SCALE_N_PER_COUNT = 0.00802 N/count`）。"
    "结论：接触点的间隙与力**互相自洽**，不是两个独立的巧合。"
)

NEW_SELFCONSIST = (
    "**自洽性核对（本文推算，常数全部来自真实代码）**：茎秆名义表面在 6.0 mm，"
    "接触判定发生在 5.688 mm，故穿透量 d = 6.0 − 5.688 = **0.312 mm**。"
    "模型 `plant_model.c:341-348` 为 `F = k1·d + k2·d² + c·max(0,−v_jaw)`。"
    "该式给出**两个边界**，而观测值必须落在它们之间，因为去抖窗口跨接两种状态：\n\n"
    "| 状态 | 阻尼项 | 模型力 |\n"
    "|---|---|---|\n"
    "| 夹爪仍以 6 mm/s 合拢（去抖刚开始） | 0.015 × 6 = 0.0900 N | "
    "0.30×0.312 + 0.06×0.312² + 0.0900 = **0.1894 N** |\n"
    "| 夹爪已被置零（`force_control.c:309`） | 0 | "
    "0.0936 + 0.0058 = **0.0994 N** |\n\n"
    "观测值 **0.156 N** 落在 **[0.0994, 0.1894] N** 区间内，且力通道的三级滤波在此处"
    "仍有群延迟，因此它既不是两个独立巧合的产物，也不能用单一瞬时模型值精确复现。"
    "**本文只声明区间包含关系，不声明“误差小于一个量化步长”**——"
    "0.1894 − 0.156 = 33 mN，约 4 个量化步长（`TP_FORCE_CAL_SCALE_N_PER_COUNT = "
    "0.00802 N/count`，`config.h`），把它写成“1 个 LSB 以内”会是过度声明。"
)

# --- §2.1.2: the 100 Hz / 1 kHz contradiction was resolved by moving the loop

OLD_RATE = (
    "#### 2.1.2 \"49 个控制周期\"到底是 49 ms 还是 490 ms\n\n"
    "**这是本文档必须纠正的一处口径。** 代码里有两种互相矛盾的说法：\n\n"
    "- 说法一（1 kHz）：`force_control.h:49` 写\"a state machine with a 1 kHz heartbeat\"；"
    "`app_sequence.c:299-302` 写\"the force loop runs at 1 kHz\"。\n"
    "- 说法二（实际）：`tp_force_control_update()` 在整个固件里**只有一个调用点**——"
    "`app_sequence.c:506`，位于 `tp_app_robot_step()`（**100 Hz** 机器人任务）内，"
    "传入 `dt_s = TP_ROBOT_PERIOD_MS / 1000.0f = 0.010`。1 kHz 控制任务只调用 "
    "`tp_gripper_update()`（`app_sequence.c:306`）去更新夹爪电机，**并不调用力控状态机**。\n\n"
    "因此 `settle_samples` 每 10 ms 才自增一次（`force_control.c:361-366`），"
    "\"49 个控制周期\"= **49 × 10 ms = 490 ms**，而不是 49 ms。这段 490 ms 是"
    "\"从进入 FORCE_CONTROL 到力首次落入 ±0.10 N 带宽\"的瞬态时长；"
    "此后还要满足 `GRIP_SETTLE_MS = TP_GRIPPER_SETTLE_MS = 80u` 的连续驻留才转入 PICK"
    "（`force_control.c:378`）。**诚实结论**：真正的 1 kHz 力环是设计意图，"
    "当前实现的力控状态机是 100 Hz；这是**实现与注释之间的真实不一致**，本文按实现记录。"
)

NEW_RATE = (
    "#### 2.1.2 \"267 个控制周期\"真的等于 267 ms 吗\n\n"
    "**这一节记录一次已修复的实现-文档不一致，以及修复后应当成立的等式。**\n\n"
    "**修复前**：`force_control.h:49` 与 `app_sequence.c` 都写\"力环以 1 kHz 运行\"，"
    "但 `tp_force_control_update()` 唯一的调用点位于 `tp_app_robot_step()`（**100 Hz**）内，"
    "传入 `dt_s = 0.010`。于是 `settle_samples` 每 10 ms 才自增一次，"
    "\"49 个控制周期\"实际是 **490 ms**，而文档与增益整定都按 1 kHz 理解——"
    "**力环 PID 的 `Ki` 直接由 `dt_s` 进入积分项，控制周期差 10 倍意味着积分增益的有效值差 10 倍**，"
    "这不是口径问题而是整定问题。\n\n"
    "**修复方式**：把力环状态机搬进 1 kHz 的 `tp_app_control_step()`"
    "（`app_sequence.c:339`），`tp_force_control_init(..., TP_CONTROL_DT_S)` "
    "（`app_tasks.c:370`）也改为 1 ms；100 Hz 的机器人任务只观察状态、不再驱动它。\n\n"
    "**修复后**：`settle_samples` 每 1 ms 自增一次，\"267 个控制周期\"= **267 ms**，"
    "与文档口径一致。此后还需满足 `GRIP_SETTLE_MS = TP_GRIPPER_SETTLE_MS = 80u` 的连续驻留"
    "（即 80 ms）才转入 PICK（`force_control.c:378`）。\n\n"
    "**旁证**：接触去抖窗口 `GRIP_CONTACT_DEBOUNCE × (uint32_t)(dt_s × 1000)` "
    "（`force_control.c:330-331`）也随之下移一个数量级，从 30 ms 变为 **3 ms**；"
    "`force_control.c:305-308` 里\"继续合爪会多压入 0.036 mm / 11 mN\"的注释同时被改为"
    "**0.018 mm / 6 mN**——原数字是按 500 Hz 采样算的，算术没错、前提错了。"
    "**凡是把环路周期写成数字的注释，都必须与真实调用点一起复核**，这是本条的教训。"
)

# --- §2.1.5: adaptation is now disabled, so the drift prediction is moot ----

OLD_ADAPT = (
    "**一个必须提醒的副作用（真实代码事实）**：2.26 N 这个均值会被自适应阈值模块消费。"
    "`force_control.c:432-447` 在 RELEASE 段用本周期实际需要的力更新 EMA，并按 "
    "`target = observed_hold_ema + TP_FORCE_ADAPT_MARGIN_N` 得到新设定"
    "（`TP_FORCE_ADAPT_MARGIN_N = 0.60f`，`TP_FORCE_ADAPT_ALPHA = 0.05f`）。"
    "由于 `TP_FORCE_ADAPT_ENABLE = (1)`（`config.h:279`）而 `force_control.c:162` "
    "直接把它赋给 `fc->adapt_enabled`，**自适应是启用的**。以初始 EMA = 2.20 N、"
    "本周期实测保持力 2.26 N 代入：新 EMA = 2.20 + (2.26 − 2.20)×0.05 = 2.203 N，"
    "建议设定 = 2.203 + 0.60 = **2.803 N**（未触及上限 `TP_FORCE_TARGET_MAX_N = 3.00f`）。"
    "即**下一个周期的夹持力设定会被抬高到约 2.80 N**。因为 `--cycles` 默认为 1"
    "（`main_host.c:354`），本次报告的那个周期不受影响；但**任何多周期运行的节拍与峰值都会随之漂移**，"
    "这一点在做 `--cycles N > 1` 的实验前必须知道。"
    "（`force_control.h:64-66` 声称 \"Adaptation is shipped disabled by default\"，"
    "与 `config.h:279` 的实取值相矛盾，见 §3.4.5 的一致性表。）"
)

NEW_ADAPT = (
    "**一个曾经存在、现已消除的副作用（真实代码事实，保留记录）。** "
    "`force_control.c:432-447` 的自适应阈值模块会在 RELEASE 段用本周期实际保持力更新 EMA，"
    "并按 `target = observed_hold_ema + TP_FORCE_ADAPT_MARGIN_N` 得到下一周期的设定"
    "（`TP_FORCE_ADAPT_MARGIN_N = 0.60f`，`TP_FORCE_ADAPT_ALPHA = 0.05f`）。\n\n"
    "**修复前的状态**：`force_control.h:64-66` 写\"Adaptation is shipped **disabled** by "
    "default precisely because it has not been validated against real buds\"，"
    "而 `config.h` 里 `TP_FORCE_ADAPT_ENABLE = (1)`，`force_control.c:162` 又把它直接赋给 "
    "`fc->adapt_enabled`——**注释说关闭、实参是开启**。按开启推算：初始 EMA = 2.20 N、"
    "本周期保持力 2.26 N，则新 EMA = 2.20 + 0.06×0.05 = 2.203 N，建议设定 = "
    "2.203 + 0.60 = **2.803 N**，即第二个周期的夹持力设定会被抬高约 0.6 N。\n\n"
    "**修复方式**：将 `TP_FORCE_ADAPT_ENABLE` 改为 `(0)`（`config.h:318`），"
    "使实参回到注释所声明的状态。项目没有真实茶芽数据来整定 `TP_FORCE_ADAPT_MARGIN_N` 与 "
    "`TP_FORCE_ADAPT_ALPHA`，因此**框架保留、默认关闭**是唯一诚实的选择。"
    "本次报告的 `--cycles 1` 运行在两种取值下结果相同，但它不再是一个隐藏的陷阱。"
)

# --- §2.1.6: peak margin table follows the peak force -----------------------

OLD_MARGIN_HEAD = "#### 2.1.6 峰值 2.493 N 的裕量"
NEW_MARGIN_HEAD = "#### 2.1.6 峰值 2.493 N 的裕量"

# --- §2.1.7: the frame count changed ---------------------------------------

OLD_PROVED = (
    "- **证明了**：在 `plant_model.c` 的 `F = k1·d + k2·d²` 模型下，力控状态机能完成一个完整周期；"
    "接触判定点与模型自洽；夹持力被维持在设定附近；帧编解码器端到端无 CRC 错误"
    "（2200 B / 100 帧 / 0 error）。"
)

NEW_PROVED = (
    "- **证明了**：在 `plant_model.c` 的 `F = k1·d + k2·d²` 模型下，力控状态机能完成一个完整周期；"
    "接触判定点落在模型给出的两个边界之间；夹持力被维持在设定附近；"
    "帧编解码器端到端无 CRC 错误（2112 B / 96 帧 / 0 error）。"
)

# --- §3.4.5 second table: all three inconsistencies are now fixed -----------

OLD_INCONSISTENT = (
    "**另外三处\"注释与实现不一致\"，同样如实记录（不影响上文结论）：**\n\n"
    "| 位置 | 注释说法 | 实际代码 | 影响 |\n"
    "|---|---|---|---|\n"
    "| `app_sequence.c:299-302`、`force_control.h:49` | 力环以 1 kHz 运行 | "
    "力控状态机唯一调用点在 100 Hz 的 `tp_app_robot_step()` | "
    "使\"49 个控制周期\"= **490 ms** 而非 49 ms（见 §2.1.2） |\n"
    "| `plant_model.c:52-54` | 6 项均匀分布之和，\"truncation at +-2.45 sigma\" | "
    "6 项求和的支撑界为 ±√(3×6) = **±4.243σ**（均值 3、方差 0.5、乘 √2）；"
    "±2.45σ 对应的是 **2 项**求和 | 该注释的数值口径错误，但\"截断对本结果无关紧要\"的结论不受影响 |\n"
    "| `force_control.h:64-66` | \"Adaptation is shipped **disabled** by default precisely "
    "because it has not been validated against real buds\" | "
    "`TP_FORCE_ADAPT_ENABLE = (1)`（`config.h:279`）→ `fc->adapt_enabled = 1`"
    "（`force_control.c:162`），**自适应是启用的** | 单周期运行（`--cycles 1`）内无影响；"
    "多周期运行时力设定会按 §2.1.5 的推算漂移到约 2.80 N |"
)

NEW_INCONSISTENT = (
    "**另外三处\"注释与实现不一致\"（已全部修复，记录保留）：**\n\n"
    "| 位置 | 注释说法 | 当时的实际代码 | 影响 | 当前状态 |\n"
    "|---|---|---|---|---|\n"
    "| `app_sequence.c`、`force_control.h:49` | 力环以 1 kHz 运行 | "
    "力控状态机唯一调用点在 100 Hz 的 `tp_app_robot_step()` | "
    "使\"49 个控制周期\"= **490 ms** 而非 49 ms；更严重的是力环 `Ki` 的有效值差 10 倍 | "
    "**已修复**：力环搬入 1 kHz 的 `tp_app_control_step()`（`app_sequence.c:339`），"
    "`dt_s = TP_CONTROL_DT_S = 1 ms`；现在\"267 个控制周期\"确实等于 267 ms（见 §2.1.2） |\n"
    "| `plant_model.c:52-54` | 6 项均匀分布之和，\"truncation at +-2.45 sigma\" | "
    "6 项求和的支撑界为 ±3√2 = **±4.243σ**（均值 3、方差 0.5）；±2.45σ 是 **2 项**求和的界 | "
    "数值口径错误，结论（截断对噪声注入无关紧要）不受影响 | "
    "**已修复**：注释改为 ±3√2 = ±4.243σ，并写明 2.45σ 的来源是 2 项求和 |\n"
    "| `force_control.h:64-66` | \"Adaptation is shipped **disabled** by default\" | "
    "`TP_FORCE_ADAPT_ENABLE = (1)` → `fc->adapt_enabled = 1`，**自适应是启用的** | "
    "单周期运行无影响；多周期运行时力设定会漂移到约 2.80 N | "
    "**已修复**：`TP_FORCE_ADAPT_ENABLE = (0)`（`config.h:318`），"
    "实参与注释一致（见 §2.1.5） |"
)

# --- §3.4.5 heading count: four -> keep, but the note about the fixes ------

OLD_TABLE_TAIL = (
    "| 3.4.4 FSM 守卫上下文指针 | **有**：`robot_fsm.h:98-108`"
    "（首版传应用上下文并强转，`guard_has_target()` 静默返回\"无目标\"） | "
    "**有**：错误偏移 → 读到错误测量值 → 偶发误判/危险误判的机理链条；"
    "以及\"`tp_fsm_init` 注释说传 NULL、实现传 `app`\"这处文档-实现不一致的核对 |"
)

NEW_TABLE_TAIL = (
    "| 3.4.4 FSM 守卫上下文指针 | **有**：`robot_fsm.h:98-108`"
    "（首版传应用上下文并强转，`guard_has_target()` 静默返回\"无目标\"） | "
    "**有**：错误偏移 → 读到错误测量值 → 偶发误判/危险误判的机理链条；"
    "以及\"`tp_fsm_init` 注释说传 NULL、实现传 `app`\"这处文档-实现不一致的核对 |\n"
    "| 3.4.5 三处注释-实现不一致 | **有**：三处注释原文均被逐字引用 | "
    "**有**：每处的数值核对（490 ms vs 49 ms、±4.243σ vs ±2.45σ、0.60 N 漂移量） |"
)


def main():
    with io.open(PATH, "r", encoding="utf-8") as handle:
        text = handle.read()
    before = len(text)

    text, problems = apply(text, [
        (OLD_TIMELINE, NEW_TIMELINE),
        (OLD_SELFCONSIST, NEW_SELFCONSIST),
        (OLD_RATE, NEW_RATE),
        (OLD_ADAPT, NEW_ADAPT),
        (OLD_PROVED, NEW_PROVED),
        (OLD_INCONSISTENT, NEW_INCONSISTENT),
        (OLD_TABLE_TAIL, NEW_TABLE_TAIL),
    ])
    if problems:
        print("FAILED docs/12_results_analysis.md:")
        for problem in problems:
            print(problem)
        return 1

    with io.open(PATH, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    print("updated docs/12_results_analysis.md (%d -> %d bytes)" % (before, len(text)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
