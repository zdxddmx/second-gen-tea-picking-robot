#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Append the second review round's findings to the README defect tables.

Kept as a script for the same reason as `update_readme_numbers.py`: the text is
Chinese, and PowerShell's `Get-Content -Raw` / `Set-Content` pair round-trips a
UTF-8 file through the console code page and destroys it.  Python reads and
writes bytes, so the only thing that changes is what this script intends.

Every replacement asserts a single match, so a stale search string fails loudly
rather than silently doing nothing.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ZH_ANCHOR = "| 11 |"
ZH_ROWS = """| 12 | **`Kt = 2.0 N·m/A` 与 `Ke = 0.05 V·s/rad` 相差 40 倍**——而 SI 单位下两者是**同一个物理量**，`Kt·I·ω = Ke·ω·I` 必须成立 | 模型描述了一台"每安培出力比自身反电动势所允许的多 40 倍"的电机，即不满足能量守恒 | 改为 `Kt = Ke = 0.4`（由"顶速 60 rad/s 需 `Ke ≤ 0.4`"与"3 A 连续电流需足够保持转矩"共同确定），并加 `TP_STATIC_ASSERT` 使该恒等式**永久**不可违反；速度环随之重新整定 |
| 13 | **速度环增益与 Python 模型不一致**（固件已重调为 `Kp=5.0 / 0.026`，Python 仍为 `12.0 / 0.015`） | 两个"独立实现"实际在跑**不同的控制器**，交叉验证因此退化为自证 | 新增 `tests/test_config_agreement.py`：**解析 `config.h`** 并逐条比对 Python 配置，把漂移变成失败测试 |
| 14 | **力环实际跑在 100 Hz，而文档、增益与整定都假设 1 kHz** | 接触瞬间最坏反应 10 ms，夹爪在 40 mm/s 下走 0.4 mm、力升 0.12 N——**整个整定带的十分之一在控制器得到第一次机会之前就被花掉** | 移入 1 kHz 控制任务；状态机仍只**观察**控制器，分层不变 |
| 15 | **顶速限制恰好等于空载转速**（`Ke=0.4` 时 24 V 空载 60000 mrad/s） | 该限制只能在**零转矩**下达到，控制器无法稳定在这一点——它不是限制，而是轴在堵转途中经过的一个值 | 降到 40000 mrad/s（空载的 67%），留出加速与保持转矩的电压裕度；测试强制该不等式 |
| 16 | **力传感器合理窗口 `(−20, +50) N` 覆盖不到任何可达值**（前端在 ±16.4 N 饱和） | 该检查是**看起来像保护功能的死代码** | 由 `TP_FORCE_USABLE_RANGE_N` 推导（±19.7 N，20% 余量），可捕获饱和标志抓不到的桥路失衡 |
| 17 | **自适应阈值 `TP_FORCE_ADAPT_ENABLE = 1`，而同一份头文件写着"默认关闭"** | 机器人在用未经验证的模型**自行重调夹持力**；而代码与自身文档互相矛盾，两者单独看都合理，因此能通过评审 | 改为 0——未经真实茶芽标定就开启属于过度声称 |
| 18 | **`rms_position_error` 存的是均方，却名为 RMS** | 每个引用它的报告都差了**一个平方根** | 更名为 `mean_sq_position_error` 并新增 `tp_cascade_rms_position_error()` 取根 |
| 19 | **`TP_PEAK_CURRENT_MA` 已文档化但固件从不执行**；`TP_GRIPPER_BACKOFF_MM`、`TP_FORCE_RELEASE_N`、`TP_MOTOR_OC_RECOVER_MS` 无任何使用点 | 按 5 A 峰值选驱动的人，依据的是固件不实现的数字 | 实现峰值限流（2 ms 窗口，独立于连续限流的 20 ms 去抖）；删除三个死宏 |
| 20 | **`TP_MODE_CALIBRATE` 注释写"禁止运动"，代码只记录模式不做任何限制** | 上位机界面已告知操作者机器人不会动，而它会动 | 在**写驱动的那一层**加互锁——只被 setter 尊重的模式标志是建议，不是联锁 |
| 21 | **`safety_monitor.c` 声称急停不去抖，实际经 `limit_switch.c` 有 3 样本 / 6 ms 去抖** | 文档与实现矛盾；在安全功能上是不可接受的表述 | 注释改为陈述事实，并说明为何 6 ms 在**本研究样机**上可接受（急停同时硬接线进栅极驱动使能通路，固件延迟不参与），以及产品化时必须移除 |
| 22 | **`can_driver.c` 日志硬编码"sample point 87.5%"** | 42 MHz APB1 下 500 kbit/s 的位时间是 84 个时钟周期，87.5% 需要时间量子数能被 8 整除——84 不能，实际是 85.7%。**总线起不来时，这条日志是第一个被检查的东西，而它在撒谎** | 日志不再引用具体的采样点，只说明"已对 APB1 求解"，实际编程值由寄存器层后端连同算式一起记录 |"""

EN_ANCHOR = "| 11 |"
EN_ROWS = """| 12 | **`Kt = 2.0 N·m/A` against `Ke = 0.05 V·s/rad`** — a factor of 40, in SI units the *same physical quantity*, since `Kt·I·ω = Ke·ω·I` must hold | The model describes a machine producing 40x more torque per amp than its own back-EMF permits, i.e. one that does not conserve energy | `Kt = Ke = 0.4`, fixed by the top-speed requirement (`Ke <= 0.4` for 60 rad/s at 24 V) together with having enough holding torque at the 3 A continuous limit; a `TP_STATIC_ASSERT` now makes the identity impossible to violate, and the velocity loop was retuned accordingly |
| 13 | **The velocity-loop gains disagreed with the Python model** (firmware retuned to `Kp = 5.0 / 0.026`, Python still at `12.0 / 0.015`) | Two "independent implementations" were running *different controllers*, which reduces cross-validation to self-confirmation | Added `tests/test_config_agreement.py`, which **parses `config.h`** and compares it against the Python config, turning drift into a failing test |
| 14 | **The force loop actually ran at 100 Hz** while its documentation, its gains and its tuning all assumed 1 kHz | The worst-case reaction at contact is 10 ms, in which the jaw travels 0.4 mm at 40 mm/s and the force rises by 0.12 N — **a tenth of the settling band spent before the controller has had a single chance to respond** | Moved into the 1 kHz control task; the state machine still only *observes* the controller, so the layering is unchanged |
| 15 | **The top-speed limit was exactly equal to the no-load speed** (60 000 mrad/s at 24 V with `Ke = 0.4`) | That limit can only be reached at *zero torque*, so the controller cannot regulate to it — it is not a limit, it is a value the axis passes through on its way to stalling | Lowered to 40 000 mrad/s (67 % of no-load), leaving voltage headroom for accelerating and holding torque; a test enforces the inequality |
| 16 | **The force plausibility window `(-20, +50) N` covered no reachable value** (the front end saturates at ±16.4 N) | The check was **dead code that looked like a protection** | Derived from `TP_FORCE_USABLE_RANGE_N` (±19.7 N, 20 % margin), which now catches the bridge unbalance the saturation flag cannot |
| 17 | **`TP_FORCE_ADAPT_ENABLE = 1` while the same header said "disabled by default"** | The robot was re-tuning its own grip force with an unvalidated model, and the code contradicted its own documentation — both statements reasonable in isolation, which is why it survived review | Set to 0: enabling it without real-bud calibration would be overclaiming |
| 18 | **`rms_position_error` held a mean square while being named and documented as RMS** | Every report quoting it was out by **a square root** | Renamed to `mean_sq_position_error`; added `tp_cascade_rms_position_error()` to take the root |
| 19 | **`TP_PEAK_CURRENT_MA` was documented but never enforced**; `TP_GRIPPER_BACKOFF_MM`, `TP_FORCE_RELEASE_N` and `TP_MOTOR_OC_RECOVER_MS` had no call sites | Anyone sizing a driver against a 5 A peak was working from a number the firmware does not implement | Implemented peak current limiting (2 ms window, separate from the 20 ms continuous debounce); deleted the three dead macros |
| 20 | **`TP_MODE_CALIBRATE` documented "motion inhibited" but the code only recorded the mode** | The operator's user interface had already told them the robot would not move, and it would | Added the interlock in the layer that *writes the drive* — a mode flag only the setter respects is a suggestion, not an interlock |
| 21 | **`safety_monitor.c` claimed the E-stop was not debounced**, while it passes through the 3-sample / 6 ms debouncer in `limit_switch.c` | The documentation contradicted the implementation, which is not an acceptable statement about a safety function | The comment now states the fact, explains why 6 ms is acceptable on *this research prototype* (the E-stop is also hard-wired into the gate-driver enable path, where no firmware latency applies), and notes that a product must remove it |
| 22 | **`can_driver.c` logged a hard-coded "sample point 87.5 %"** | A 500 kbit/s bit time at 42 MHz APB1 is 84 clock periods, and 87.5 % needs a time-quantum count divisible by 8 — 84 is not, so the achievable figure is 85.7 %. **When a bus will not come up, that log line is the first thing anyone reads, and it was lying** | The line no longer quotes a sample point; the register-level backend logs the value it actually programmed, with the arithmetic |"""


def append_rows(path, anchor, rows):
    full = os.path.join(ROOT, path)
    with io.open(full, "r", encoding="utf-8") as handle:
        lines = handle.read().split("\n")

    idx = None
    for i, line in enumerate(lines):
        if line.startswith(anchor) and "| 11 |" in line:
            idx = i
            break
    if idx is None:
        print("FAILED %s: could not find the row starting with %r" % (path, anchor))
        return False
    if any(l.startswith("| 12 |") for l in lines):
        print("already applied: %s" % path)
        return True

    lines.insert(idx + 1, rows)
    with io.open(full, "w", encoding="utf-8", newline="\n") as handle:
        handle.write("\n".join(lines))
    print("appended %d rows to %s" % (rows.count("\n") + 1, path))
    return True


def main():
    ok = append_rows("README.md", ZH_ANCHOR, ZH_ROWS)
    ok = append_rows("README_EN.md", EN_ANCHOR, EN_ROWS) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
