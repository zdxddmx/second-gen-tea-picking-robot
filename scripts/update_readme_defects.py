#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bring README.md and README_EN.md in line with the second correction round.

Two kinds of edit, both driven by the same principle as the other update
scripts: assert that the text being replaced was found exactly once, so a stale
search string fails loudly instead of quietly leaving a document that describes
a version of the code that no longer exists.

1. Structural counts (file counts, line counts, module counts, test counts)
   come from `scripts/project_stats.py` and the test runners.
2. The defect table gains the entries found in this round.  They are appended
   rather than merged into the earlier entries, because a reader who wants to
   know what the *first* implementation got wrong should not have to untangle
   it from what the second round found.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NEW_ZH_ROWS = (
    "| 23 | **夹爪轴的力反作用项符号写反了**（`v_jaw -= force/20`，而该模型正值 = 张开）"
    " | 力越大夹爪闭合越快，闭合又使力更大——**正反馈发散**。场景 D 中保护在 5.013 N 正确触发后，"
    "夹爪被命令零占空比，力却继续涨到 **16.4 N**、间隙降到 1.0 mm 硬限，并把传感器 ADC 顶到轨、"
    "**误锁存 `ERR_SENSOR`**。全套测试、七个场景、六张曲线都不报错 | "
    "符号改为 `+=`；新增单元测试 `a_free_jaw_is_pushed_open_by_the_stem`——它断言的是**方向**"
    "而非数值，因此对参数变化免疫 |\n"
    "| 24 | **过力跳闸后夹爪只是\"停住\"而不是\"张开\"** | `grip_trip()` 命令的 30 mm/s 张开被 1 kHz "
    "控制任务里的 `tp_gripper_stop()` 立刻覆盖。场景 D 最终确实松开了，但那是第 23 条的反作用力把夹爪顶开的——"
    "**不是控制器让它松开的**。若茎秆更软，夹爪会持续夹着茶芽，而这正是保护想结束的状态 | "
    "故障阻挡运动时关节仍保持位置，但**夹爪改为主动张开**——\"停止\"不是通用的安全动作，"
    "每个执行器都有自己的安全方向 |\n"
    "| 25 | **测试用不等宽窗口比较力增量**（1010 / 1050 / 1082 步，窗口 40 与 32 步） | "
    "该断言本意是验证力-位移曲线的二阶项，实际在比较**两个不同长度窗口的增量**，"
    "只在曲率超额恰好大于窗口长度差时通过。修正第 23 条后它立刻失败（0.868 < 0.884），"
    "**失败的是测试，不是物理** | 改为等宽窗口 1010 / 1042 / 1074；教训是\"数值断言的单位必须与被断言的量一致\" |\n"
    "| 26 | **轨迹只记录反馈，不记录指令** | CSV 有位置、速度、力、间隙、关节占空比，"
    "却**没有夹爪速度指令与实际值**。于是\"夹爪在闭合\"无法区分两种相反的故障：控制器命令它闭合，"
    "还是执行器无视了控制器——第 23 条的定位因此多花了一整轮排查 | "
    "CSV 新增 `grip_cmd_mm_s`、`grip_applied_mm_s`、`grip_duty`、`grip_mode` 四列；"
    "**指令与反馈必须一起记录** |\n"
    "| 27 | **`CALIBRATE_FORCE`（0x42）在 Python 侧没有载荷规格** | "
    "固件定义了 23 条命令，`host/protocol.py` 的枚举也列了 23 条，但 `_PAYLOAD_SPEC` 里缺这一条，"
    "`payload_format()` 对未知命令直接抛 `KeyError`——**上位机根本无法构造这条命令**。"
    "而协议自检长期输出 \"22 commands round-tripped\" 并显示 PASS，因为它只遍历自己手上的列表 | "
    "补齐规格与编解码器；新增 `_check_roundtrip_coverage()`，断言命令名集合与往返列表、载荷规格、"
    "长度表**三者相等**。现在输出 23，漏掉任何一条都会失败。"
    "**覆盖率检查必须是集合比较，不能是列表遍历** |\n"
    "| 28 | **无采样时把 24 V 母线打印成 `0.0 V`** | 状态行在第一个 500 Hz 采样之前打印，"
    "`I=0mA Vbus=0.0V` 与\"母线掉电\"无法区分，而这是操作员最需要一眼分辨的一行 | "
    "`sensor_cycles == 0` 时打印 `-`；**\"还没有数据\"与\"数据是零\"必须在输出里可区分** |\n"
    "| 29 | **场景 F 的日志引用 `Kt = 2.0` 时代的驱动能力**（写 6.0 N·m，真实是 1.2 N·m） | "
    "结论（8 N·m 是真堵转）反而被低估了——真实倍数是 6.7 倍。但日志是唯一会被贴进报告的东西 | "
    "倍数改为在运行时由 `TP_PLANT_MOTOR_KT_NM_PER_A × TP_MAX_CURRENT_MA` 计算；"
    "**凡是由常量算出的结论，不要把结论写死在字符串里** |\n"
    "| 30 | **注释中的破折号被 GBK 破坏**（14 个文件 35 处） | 早期的文档编辑用了 PowerShell 的 "
    "`Get-Content -Raw` / `Set-Content`，它在控制台代码页上往返 UTF-8：`E2 80 94` 变成 "
    "`E2 80`（= U+9225）加一个非法字节。不影响编译，但说明**当时的编辑工具链不可信** | "
    "新增 `scripts/fix_mojibake.py`（只匹配这一对码位，不误伤中文）；此后所有中文文件一律用 "
    "Python 按字节读写或 `edit` 工具修改 |\n"
)

ZH = [
    # defect table grows
    ("| 22 | **`can_driver.c` 日志硬编码\"sample point 87.5%\"** | 42 MHz APB1 下 500 kbit/s 的位时间是 84 个时钟周期，"
     "87.5% 需要时间量子数能被 8 整除——84 不能，实际是 85.7%。**总线起不来时，这条日志是第一个被检查的东西，"
     "而它在撒谎** | 日志不再引用具体的采样点，只说明\"已对 APB1 求解\"，实际编程值由寄存器层后端连同算式一起记录 |\n",
     "| 22 | **`can_driver.c` 日志硬编码\"sample point 87.5%\"** | 42 MHz APB1 下 500 kbit/s 的位时间是 84 个时钟周期，"
     "87.5% 需要时间量子数能被 8 整除——84 不能，实际是 85.7%。**总线起不来时，这条日志是第一个被检查的东西，"
     "而它在撒谎** | 日志不再引用具体的采样点，只说明\"已对 APB1 求解\"，实际编程值由寄存器层后端连同算式一起记录 |\n"
     + NEW_ZH_ROWS),

    # directory structure counts
    ("├── firmware/                        固件（89 个 C 文件 / 25,939 行）",
     "├── firmware/                        固件（89 个 C 文件 / 29,496 行）"),
    ("│   └── Tests/                       125 个 C 单元测试 + 零依赖测试框架",
     "│   └── Tests/                       126 个 C 单元测试 + 零依赖测试框架"),
    ("├── simulator/                       Python 独立仿真（18 模块）",
     "├── simulator/                       Python 独立仿真（20 模块）"),
    ("├── docs/                            13 篇论文级文档 + 面试手册 + 简历 + 标定",
     "├── docs/                            13 篇论文级文档 + 面试手册 + 简历 + 标定"),
    ("├── scripts/                         run_demo.py · 构建 · 场景扫描 · 烧录",
     "├── scripts/                         run_demo.py · 构建 · 场景扫描 · 统计 · 文档同步"),
]

EN = [
    ("125 C unit tests", "126 C unit tests"),
]


def edit(path, replacements):
    full = os.path.join(ROOT, path)
    with io.open(full, "r", encoding="utf-8") as handle:
        text = handle.read()
    problems = []
    for old, new in replacements:
        count = text.count(old)
        if count != 1:
            problems.append("  %d match(es) for: %s" % (count, old[:80]))
            continue
        text = text.replace(old, new)
    if problems:
        print("FAILED %s:" % path)
        for problem in problems:
            print(problem)
        return False
    with io.open(full, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    print("updated %s" % path)
    return True


def main():
    ok = edit("README.md", ZH)
    ok = edit("README_EN.md", EN) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
