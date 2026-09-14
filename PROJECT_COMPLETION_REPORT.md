# 项目完成报告 · PROJECT COMPLETION REPORT

**项目名称**：第二代智能无人机采茶机器人系统
**Second-Generation Intelligent UAV Tea-Picking Robotic System**
**项目周期**：2024 – 至今
**报告生成**：由本次工程实施的最终验证结果生成，所有数字均可在本仓库复现

---

## 1. 项目完成内容

### 1.1 交付概览

| 交付物 | 内容 | 状态 |
|---|---|---|
| **嵌入式固件** | 分层 C 固件：驱动 → 控制算法 → 协议 → 安全 → 应用状态机，支持 STM32F407 与 PC 双平台 | ✅ 完成并通过测试 |
| **实时操作系统抽象** | OSAL 双后端（FreeRTOS 抢占式 / 裸机确定性协作式），七个任务 | ✅ 完成 |
| **控制算法** | 位置/速度双闭环级联 PID、力反馈柔性夹持、梯形与 S 曲线轨迹、三级数字滤波 | ✅ 完成并通过测试 |
| **通信协议** | CRC16 帧协议（23 条命令 + 6 份回报）+ Modbus RTU 帧层 + CAN 传输层 | ✅ 完成并通过跨实现一致性测试 |
| **安全机制** | 九类保护、四级严重度、失效安全极性、看门狗单点喂狗 | ✅ 完成并通过场景验证 |
| **PC 仿真（C 固件）** | 同一份固件源码在确定性物理模型上运行，七个场景 | ✅ 7/7 通过 |
| **Python 仿真** | 独立编写的物理模型与控制器，七个场景 + 五类曲线 | ✅ 7/7 通过 |
| **上位机工具** | 监控 GUI（含无界面回退）、视觉模拟器、日志分析器 | ✅ 7/7 自检通过 |
| **测试** | 125 个 C 单元测试 + 22 个 Python 测试（含跨实现一致性测试） | ✅ 全部通过 |
| **文档** | 13 篇论文级设计文档 + 5 篇硬件文档 + 标定流程 + 135 道面试题 + 简历文案 | ✅ 完成 |
| **构建系统** | 根 Makefile（host / test / stm32）+ CMake 等价配置 + PowerShell 脚本 | ✅ Makefile 已验证 |
| **Git / GitHub** | 分阶段专业提交历史 + 公开仓库 | ✅ 完成 |

### 1.2 分阶段执行情况

| 阶段 | 内容 | 结果 |
|---|---|---|
| PHASE 1 | 项目架构设计、`config.h` 参数中心、接口契约、第三方源码引入 | ✅ |
| PHASE 2 | 核心 C 模块：9 个设备驱动 + 中间件（滤波/环形缓冲/日志/格式化） | ✅ |
| PHASE 3 | PID（含两种抗积分饱和）、双闭环级联、轨迹规划、表驱动 FSM | ✅ |
| PHASE 4 | 力反馈柔性夹持六段状态机 + 三级保护 + 标定接口 | ✅ |
| PHASE 5 | CRC16、流式帧解析器、命令分发、Modbus RTU 帧层 | ✅ |
| PHASE 6 | 故障注册表、安全监控、九类保护、看门狗 | ✅ |
| PHASE 7 | OSAL 双后端 + 七个任务 + 主机入口 | ✅ |
| PHASE 8 | 125 个 C 测试 + 22 个 Python 测试 | ✅ |
| PHASE 9 | 13 篇设计文档 + 5 篇硬件文档 + 面试手册 + 简历 | ✅ |
| PHASE 10 | README 中文版 + 英文版 + Badges | ✅ |
| PHASE 11 | 本地验证（构建 / 测试 / 仿真 / ARM 交叉编译） | ✅ |
| PHASE 12 | Git 分阶段提交历史 | ✅ |
| PHASE 13 | GitHub 公开仓库 + 描述 + Topics | ✅ |

---

## 2. 文件结构

```
second-gen-tea-picking-robot/
├── firmware/                       嵌入式固件
│   ├── Core/                       入口、任务体、采摘编排、STM32 中断
│   ├── Application/                config.h、robot_fsm.c
│   ├── Control/                    pid · cascade · force_control · trajectory
│   ├── Protocol/                   crc16 · frame · protocol · protocol_types · codec
│   ├── Safety/                     fault_codes · fault_manager · safety_monitor
│   ├── Middleware/
│   │   ├── osal/                   osal.h + freertos / baremetal 后端
│   │   └── utils/                  filters · ring_buffer · logger · tp_format · robot_state
│   ├── BSP/                        9 个设备驱动 + 双平台后端
│   ├── Drivers/                    startup · 链接脚本 · third_party（HAL/CMSIS/FreeRTOS）
│   └── Tests/                      125 个 C 单元测试 + 零依赖框架
├── simulator/                      Python 独立仿真（18 模块）
├── host/                           协议 · 监控 GUI · 链路抽象 · 自检
├── tools/                          vision_mock.py · analyze_log.py
├── tests/                          Python 测试 + 跨实现一致性测试 + 零依赖运行器
├── docs/                           21 篇文档（13 设计 + 面试 + 简历 + 标定 + 2 索引）
├── hardware/                       5 篇硬件设计文档
├── data/                           logs / results
├── images/                         README 插图
├── scripts/                        run_demo.py · 构建 · 场景扫描 · 烧录
├── toolchain/                      CMake 交叉编译工具链
├── Makefile · CMakeLists.txt · LICENSE · .gitignore · requirements.txt
├── README.md · README_EN.md
└── PROJECT_COMPLETION_REPORT.md
```

---

## 3. 核心模块

| 模块 | 文件 | 职责 | 关键设计点 |
|---|---|---|---|
| **硬件缝隙** | `BSP/platform_api.h` | 唯一的硬件访问入口 | 单一编译期缝隙，1 kHz 环路无间接调用；`--gc-sections` 可裁剪 |
| **PID** | `Control/pid.c` | 三个环共用 | 梯形积分、微分先行、微分低通、反算 + 条件积分两种抗饱和、运行时改增益 |
| **双闭环** | `Control/cascade.c` | 位置→速度级联 | 内环输出即外环输入范围；带宽分离 10 倍由设计约束保证 |
| **柔性夹持** | `Control/force_control.c` | 六段状态机 | 力环输出为夹爪速度；三级保护；接触去抖；整定相与调节相统计分离 |
| **状态机** | `Application/robot_fsm.c` | 12 状态 / 14 事件 | 转移表作为**数据**：可枚举、可校验、可记录、可绘图 |
| **协议** | `Protocol/frame.c` | 流式解析 | 不假设一次读取等于一帧；CRC 失败不吞掉下一帧；超长长度立即拒绝 |
| **安全** | `Safety/fault_manager.c` | 故障注册表 | active / latched / count 三态分离；严重度驱动反应策略 |
| **OSAL** | `Middleware/osal/` | 任务抽象 | 任务体是"单次迭代"，使测试可直接驱动、仿真逐位可复现 |
| **物理模型** | `BSP/host/plant_model.c` | 主机物理模型 | 电流一阶滞后（数值稳定）、粘滑堵转约束、非线性茶芽刚度、编码器量化 |

---

## 4. 代码统计

| 类别 | 文件数 | 行数 |
|---|---|---|
| **C 源码与头文件**（不含 third_party） | 89 | 25,939 |
| **Python 源码** | 35 | 10,551 |
| **文档（Markdown）** | 21 | 19,372 |
| README（中 + 英） | 2 | ~2,000 |
| **第三方源码**（STM32F4 HAL / CMSIS / FreeRTOS，逐字节校验） | 123 | — |
| **合计（自有代码 + 文档）** | **147** | **~57,900** |

### 目标固件尺寸（`arm-none-eabi-size`，真实构建产物）

```
   text    data     bss     dec     hex   filename
  57820      52   25140   83012   14444   build/stm32/tea_picking_robot.elf
```

| 区域 | 占用 | 芯片容量 | 占比 |
|---|---|---|---|
| Flash (text + data) | 57,872 B | 512 KB | **11.0 %** |
| SRAM (data + bss) | 25,192 B | 192 KB | **12.8 %** |

---

## 5. 测试情况

### 5.1 汇总

| 测试套件 | 数量 | 结果 | 命令 |
|---|---|---|---|
| **C 单元测试** | **125** | ✅ 全部通过 | `python scripts/build_host.ps1 -Target test` |
| **Python 测试** | **22** | ✅ 全部通过 | `python tests/run_tests.py` |
| **C 固件场景** | **7** | ✅ 7/7 通过 | `./scripts/run_scenarios.ps1` |
| **Python 仿真场景** | **7** | ✅ 7/7 通过 | `python simulator/main.py --scenario all` |
| **上位机自检** | 7 | ✅ 7/7 通过 | `python host/selftest.py` |
| **协议自检** | 4 | ✅ 4/4 通过 | `python host/protocol.py` |

### 5.2 C 测试覆盖（按套件）

`pid` · `crc` · `frame` · `fsm` · `fault` · `force_sensor` · `filters_mavg/lpf/median/slew/spike` · `ringbuf` · `trajectory` · `cascade` · `plant_joint` · `plant_gripper` · `plant_sensor` · `driver_motor` · `driver_encoder` · `driver_gripper` · `driver_limit_switch`

覆盖四类输入：**正常**、**边界**、**非法**、**异常状态**。

### 5.3 测试发现并修复的真实缺陷（11 项）

见 README「关键发现」第 4 节与 `docs/12_results_analysis.md`。摘要：

| # | 缺陷 | 影响 |
|---|---|---|
| 1 | 占空比斜率限制过慢（满量程 250 ms） | 速度环主导滞后，关节振荡 |
| 2 | 力尖峰检测阈值过低且尖峰时不更新参考值 | 真实撞击后误锁存传感器故障 |
| 3 | 夹持控制器在 RELEASE 态未被服务 | 流程永久挂起 |
| 4 | FSM 守卫收到错误类型的上下文指针 | `START` 时好时坏地拒绝启动 |
| 5 | 堵转建模为大转矩而非运动约束 | 数值振荡，堵转检测失效 |
| 6 | 限位否写回调用者的设定值 | 夹爪触限后永不再动 |
| 7 | 过流折返按满量程而非当前占空比计算 | 电流限制无效，保护误触发 |
| 8 | 禁用通道停止刷新电流读数 | 故障永久无法清除 |
| 9 | ADC 钉轨落在合理窗口内 | 断线被当作合法负力 |
| 10 | `clear_all` 把未触发故障计为已清除 | 健康机器人报告 9 个故障 |
| 11 | **跨实现线上不兼容（3 处长度 + 3 处命令码 + 环 ID 映射对调）** | 主机调错控制环；`target_id` 错读 |

**第 11 类缺陷只能由跨实现测试发现**——单语言测试套件无论多完整都无法察觉。

---

## 6. 仿真结果

### 6.1 场景 A（正常采摘）关键数据

| 指标 | 结果 |
|---|---|
| 目标检测 | t = 1.20 s，id=1，x=0 mm，y=15 mm，z=185 mm，置信度 95 % |
| 接触检测 | t = 2.57 s，间隙 **5.824 mm**，**F = 0.148 N** |
| 达到目标力 | t = 3.14 s，**2.17 N**（目标 2.20 N ± 0.10 N） |
| **力过冲** | **0.00 N** |
| 整定时间 | 49 个控制周期 |
| 夹持期间平均力 | **2.26 N** |
| **峰值夹持力** | **2.386 N**（硬限 5.00 N） |
| 周期完成 | t = 4.78 s，`cycle_count = 1` |
| **控制周期超时** | **0 次**（6000 控制 / 3000 传感器 / 600 通信 / 600 机器人 / 600 安全） |
| 主机链路解码 | 2200 B，100 帧，**0 CRC 错误** |

### 6.2 七场景判据表

| 场景 | 扰动 | 结果 | 故障码 | 峰值力 |
|---|---|---|---|---|
| A 正常采摘 | 无 | 周期完成 | 无 | 2.386 N |
| B 目标位置误差 | 视觉 X +8 mm | 周期中止 | `ERR_TARGET` | 0.167 N |
| C 力传感器噪声 | σ = 80 mN | 周期完成 | 无 | 2.426 N |
| D 夹持力过大 | 位置控制 + 茎秆 16 mm | 保护触发 | `ERR_FORCE_OVERLOAD` | — |
| E 通信超时 | 心跳后静默 | 周期停止 | `ERR_COMM_TIMEOUT` | — |
| F 电机堵转 | 8 N·m + 电流限 800 mA | 保护触发 | `ERR_MOTOR_STALL` | — |
| G 急停 | t = 1.2 s | 立即停机 | `ERR_ESTOP` | — |

### 6.3 最重要的三条结论

1. **力控使"茎秆比预期更硬"不再是问题。** 场景 D 最初在力控开启下把茎秆改硬，结果周期**正常完成**、峰值仅 3.2 N。这不是场景缺陷，而是项目的中心结论：力控夹爪不关心茶芽有多硬。要演示过力保护必须**移除力反馈**——也就是第一代机器所做的。
2. **硬堵转时过流保护先于堵转保护触发，这是正确行为。** 堵转电机满转矩需求在 50 ms 内超过连续电流额定，而堵转检测刻意等待 300 ms。堵转检测的真正职责是**低电流堵转**。
3. **目标位置误差直接限制节拍。** +8 mm 的视觉横向误差使夹爪在错误距离减速，耗尽时间预算并触发接触超时。视觉精度是节拍约束，不是"锦上添花"。

---

## 7. 已实现功能

| 类别 | 功能 |
|---|---|
| **驱动** | 电机（占空比斜率限制、过流折返、使能/禁用、堵转上报）；编码器（定增量法测速、零位、故障锁存）；力传感器（三级滤波、零点与量程标定、饱和/尖峰/越界诊断）；ADC（DMA 块、抽取、超限计数）；UART/RS485/CAN；夹爪（位置环、限位否决、归零）；限位开关（饱和计数去抖） |
| **控制** | 位置/速度双闭环 PID；前馈（轨迹速度加在内环参考而非输出）；输出与积分限幅；反算与条件积分两种抗积分饱和；微分先行 + 微分低通；运行时改增益与限幅；`Reset`；梯形 / S 曲线 / 阶跃三种参考型线；中止并从当前位置重规划；发散检测与保持最后有效输出 |
| **柔性夹持** | 六段状态机；接触去抖确认；力环输出为夹爪速度；软限降额；硬限立即张开；接触超时与整定超时；调节相与整定相统计分离；过冲与整定周期指标；自适应阈值框架（默认关闭） |
| **状态机** | 12 状态 / 14 事件表驱动 FSM；守卫与动作；转移轨迹环形缓冲；`ForceState` 应急路径；`tp_fsm_step` 时间驱动安全网 |
| **协议** | 23 条命令 + 6 份回报；CRC-16/MODBUS（含增量式）；流式解析器（重同步、超长立即拒绝、CRC 失败不吞帧）；逐条命令的编码解码器；链路监督与丢包估计；Modbus RTU 帧层与方向控制；CAN 传输层（邮箱管理、验收滤波、bus-off 诊断） |
| **安全** | 九类保护（过力、堵转、过流、编码器、通信、传感器、限位、急停、看门狗）；故障注册表（active/latched/count 三态）；四级严重度与反应策略；硬件切断与软件释放锁存；防击败保护（条件存在时拒绝清除）；活跃时长统计；故障历史环形缓冲 |
| **系统** | OSAL 双后端（FreeRTOS / 裸机协作式）；七个任务（1 kHz / 500 Hz / 4×100 Hz / 20 Hz / 5 Hz）；队列、互斥量、计数信号量、任务通知；单点喂狗；分级日志（限流 + 丢包计数）；零依赖格式化器（避免 newlib 浮点格式化缺失） |
| **仿真与工具** | C 固件主机仿真（确定性 1 ms 调度 + 物理模型）；七个场景与判据评估；CSV 轨迹；Python 独立仿真（18 模块）；五个时间序列图 + 场景汇总图 + 对比图；视觉模拟器（真实丢包、置信度分布、误检与重复检测）；监控 GUI（无界面回退）；日志分析器（阶跃响应指标、故障时间线、状态驻留直方图） |
| **工程** | 根 Makefile（host/test/stm32/size/clean/help）；CMake 等价配置与工具链文件；GNU 启动文件与链接脚本；寄存器级 CAN 与 IWDG（因本地无对应 HAL）；FreeRTOS 缺移植时的明确可执行报错；PowerShell 脚本（构建、场景扫描） |

---

## 8. 未实现功能

| 项目 | 原因 | 影响的交付物 |
|---|---|---|
| **实物样机** | 未制板、未采购器件 | 全部硬件文档均为参考设计；无任何实测数据 |
| **FreeRTOS 目标构建** | 本机缺少 GCC Cortex-M4F 移植且无网络获取 | `osal_freertos.c`、`FreeRTOSConfig.h`、`freertos_hooks.c` **从未被任何构建编译过** |
| **CMake 构建** | 本机未安装 cmake | `CMakeLists.txt` 与工具链文件**从未配置或构建**；Makefile 是已验证构建 |
| **Modbus RTU 从站寄存器映射** | 帧层已完成，寄存器表与从站应答尚未实现 | 工业 PLC 互联能力未落地 |
| **力传感器量程标定（`kind=1`）** | 需要实物砝码与人工加载 | 协议命令返回 `TP_ERR_UNSUPPORTED` |
| **参数非易失存储** | 未引入 NVM 层 | 标定值复位即丢 |
| **第二关节与真实逆运动学** | 机构为单关节 | 直角坐标到关节角使用小角度近似 |
| **手眼标定与真实视觉** | 视觉系统由 `vision_mock.py` 模拟 | 无真实检测模型 |
| **破损率量化方法** | 需要真实茶芽与分级标准 | "≤ 1 %" 只能作为设计目标 |
| **自适应阈值标定** | 无真实茶芽数据 | 框架存在但默认关闭 |
| **M/T 法测速** | 需要硬件定时器捕获验证 | 极低速下速度估计窗口拉长 |

---

## 9. 当前限制

### 9.1 不可验证的部分（本机无硬件）

- ELF 已证明**可链接、内存布局正确、向量表正确**；但 **HSE 起振、PLL 锁定 168 MHz、ADC/DMA 实际时序、编码器实际计数、CAN 真实位定时、IWDG 真实超时（LSI 未修调）均未在硅片上验证**。
- 无示波器、无 EMC 预测试、无温升测试、无机械刚度测试。
- 力传感器标定流程**已设计但从未在实物上执行**。

### 9.2 已知的次要代码缺口

| 项 | 说明 |
|---|---|
| `TP_PEAK_CURRENT_MA` | 仅在仿真模型中生效，固件未执行峰值限流 |
| `TP_GRIPPER_BACKOFF_MM`、`TP_FORCE_RELEASE_N`、`TP_MOTOR_OC_RECOVER_MS` | 已定义但无使用点 |
| `tp_platform_adc_read_block()` | 目标后端已实现，但应用层走的是单次读取路径 |
| ADC 新样本率 vs 500 Hz 传感器任务 | 需在实物上确认每个传感器周期恰有一个新样本 |

### 9.3 模型参数的不确定性

茎秆刚度 `k1 = 0.30 N/mm`、`k2 = 0.06 N/mm²`、损伤阈值 4 N、摩擦系数——**全部是模型假设而非测量值**。仿真结论对这组参数敏感，实物标定后需重新评估。

### 9.4 数据诚实性

| 类别 | 数量 |
|---|---|
| Measured（实测） | **0 项** |
| Simulated（仿真，可复现） | 全部数值结果 |
| Target（设计目标，未验证） | 破损率 ≤ 1 %、效率提升 40 % |
| Estimated（数据手册推导的设计计算） | 8.0 mN 分辨率、±16.4 N 量程、带宽、CAN 位定时、IWDG 定时 |

---

## 10. GitHub 地址

```
https://github.com/meminehobe24435-cmyk/second-gen-tea-picking-robot
```

**仓库描述**：
> Embedded control system for an intelligent tea-picking robot with cascaded PID, force feedback, FreeRTOS and multi-bus communication.

**Topics**：`stm32` `embedded` `robotics` `freertos` `pid-control` `control-system` `robot` `agriculture` `smart-agriculture` `can` `rs485` `c-language` `python` `simulation`

---

## 11. 如何运行

### 11.1 一条命令（推荐）

```bash
python scripts/run_demo.py
```

自动完成：编译主机固件 → 运行 C 固件采摘周期 → 打印采摘序列 → 运行 Python 独立模型 → **交叉校验两者** → 绘制曲线。

预期结尾：

```
  [PASS] peak grip force inside the hard limit                firmware 2.387 N < 5.00 N
  [PASS] the joint is stationary at the end of the cycle      firmware -68.1 mrad/s
  [PASS] both models bound the grip force the same way        firmware 2.387 N vs simulator 2.156 N
  [PASS] no control deadline was missed                       0 misses
  DEMO PASSED
```

### 11.2 分步运行

```bash
# 依赖
python -m pip install -r requirements.txt

# C 固件仿真 + C 单元测试
python scripts/build_host.ps1 -Target both

# C 固件场景（七个，带判据表）
./scripts/run_scenarios.ps1 -Csv

# Python 仿真
python simulator/main.py
python simulator/main.py --scenario all

# Python 测试（含跨实现一致性）
python tests/run_tests.py

# 上位机
python host/robot_monitor.py --demo --nogui --duration 5 --send-demo-sequence
python tools/vision_mock.py --stdout --count 20 --seed 1
python tools/analyze_log.py data/logs/demo_firmware_A.csv
```

---

## 12. 如何编译

### 12.1 主机目标（PC 仿真固件与 C 单元测试）

```bash
make host        # → build/host/tea_picking_robot_host.exe
make test        # 构建并运行 125 个 C 单元测试
```

无 `make` 的 Windows：

```powershell
./scripts/build_host.ps1 -Target both
```

**实测结果**：主机固件 667,533 字节；C 单元测试 125/125 通过。

### 12.2 STM32F407 目标固件

```bash
make stm32       # → build/stm32/tea_picking_robot.{elf,bin,hex}
make size
```

**实测结果（真实构建产物）**：

```
text 57820 / data 52 / bss 25140 / dec 83012
ELF 已链接，向量表位于 0x08000000，98 个字
```

**已验证**：`arm-none-eabi-objdump -h` 确认 `.isr_vector` 位于 0x08000000 且长度为 0x188（98×4）；`arm-none-eabi-nm` 确认 `g_pfnVectors`、`Reset_Handler`、`SystemInit`、`main` 均存在；`make clean` 后重建仍然成功（验证无陈旧对象依赖）。

### 12.3 FreeRTOS 目标（需要额外移植）

```bash
make stm32 RTOS=freertos
```

本机缺少 GCC Cortex-M4F 移植（`portable/GCC/ARM_CM4F`），该目标会**打印明确的修复指令并以非零码退出**，不静默降级：

```
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
make stm32 RTOS=freertos FREERTOS_DIR=... FREERTOS_PORT_DIR=.../portable/GCC/ARM_CM4F
```

原因：树内唯一的 Cortex-M 移植是 `RVDS/ARM_CM3`——面向 ARM 编译器，且 CM3 移植**不保存浮点上下文**，而本目标使用 `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`，会导致 s0–s31 在每次上下文切换时被破坏。

### 12.4 烧录（已文档化，未执行）

```bash
./scripts/flash_openocd.sh
```

无实物板、无 ST-Link、无 openocd，**从未执行**。

---

## 13. 如何用于面试

### 13.1 三分钟讲法

1. **问题**（30 秒）：茶芽是柔软活体，固定位置夹持的力取决于茎秆刚度，而刚度未知。仿真复现：位置控制下茎秆外移 8 mm，预测力 12.2 N，远超 5 N 硬限，**而控制器全程不知道**。
2. **方案**（60 秒）：夹持阶段改为**力反馈闭环**——力环输出是夹爪速度而非位置。接近分快速（40 mm/s）与慢速（6 mm/s）两段，接触需 3 次去抖确认；六段状态机；三级保护（软限降额 / 硬限立即张开 / 传感器故障拒绝力控）。
3. **架构**（60 秒）：7 层分层，唯一硬件缝隙 `platform_api.h`，OSAL 双后端。因此**同一份固件源码在 PC 上运行**——125 个 C 单元测试覆盖的就是上板跑的生产代码路径。
4. **诚实结论**（30 秒）：场景 D 最初在力控开启下把茎秆改硬，周期正常完成、峰值仅 3.2 N。这不是缺陷——**力控夹爪不关心茶芽有多硬**，这正是项目的中心结论。项目没有实物，所有数字都是仿真，破损率 ≤ 1 % 是设计目标而非实测。

### 13.2 高频追问与准备位置

| 追问 | 答案位置 |
|---|---|
| 为什么双闭环？为什么位置环在外？ | README「控制算法」三条理由 + `docs/05_control_algorithm.md` |
| PID 怎么整定？有数据吗？ | 由被控对象参数推导（`Kp = τ/(K·Tc)`），**在线仿真验证，实物待验**；`docs/05` |
| 积分饱和是什么？怎么解决？ | 两种策略都在代码里；`firmware/Control/pid.c` + `docs/05` |
| 力传感器怎么标定？ | `docs/force_sensor_calibration.md`（流程已设计，**未在实物执行**） |
| 为什么 ADC 要滤波？为什么用 DMA？ | README「硬件架构」三级滤波分工 + `docs/03` |
| 编码器低速测速怎么办？ | README「控制算法」表格（含 770 mrad/s 临界值推导）+ `firmware/BSP/encoder.h` |
| 通信丢包怎么办？ | README「通信协议」丢包估计与超时策略 |
| 为什么用 FreeRTOS？优先级怎么分？ | README「FreeRTOS 任务架构」表 + `docs/08_freertos_design.md` |
| 机械臂堵转怎么办？ | README「安全机制」（含过流与堵转的**真实竞争**发现） |
| 为什么不用单纯位置控制？ | README「柔性夹持」+ 场景 D 的真实教训 |
| 系统最大瓶颈是什么？ | 目标位置精度直接限制节拍（场景 B）；实物验证尚未进行 |
| **破损率 ≤ 1 % 怎么测的？** | **这是设计目标，不是实测结果**——项目中不存在该实测数据 |

### 13.3 面试文档

- [`docs/interview_guide.md`](docs/interview_guide.md) —— **135 道**问答，每题含「一句话回答 / 30 秒展开 / 结合本项目的数字与代码 / 可能的追问 / 加分点与陷阱」
- [`docs/resume_project.md`](docs/resume_project.md) —— 100 / 200 / 300 字版 + 嵌入式软件 / FAE / 机器人控制 / 硬件联调四个版本

---

## 14. 下一阶段建议

| 优先级 | 任务 | 理由 | 风险 |
|---|---|---|---|
| **P0** | 实物制板与逐外设上电验证 | 项目当前最大的空白；所有结论都建立在仿真之上 | 硬件调试周期长，可能需要 2–3 轮改板 |
| **P0** | 拉取 FreeRTOS-Kernel 并完成 GCC ARM_CM4F 移植 | 使 `make stm32 RTOS=freertos` 可用，验证抢占式调度下的实时性 | 需要实测栈水位与抖动，可能需要调整栈深 |
| **P1** | 力传感器标定与实物夹持力验证 | 把 8.0 mN 分辨率从设计计算变为实测 | 需采购标准砝码与测力计 |
| **P1** | 破损率量化方法设计 | 把"≤ 1 %"从目标变成可测量 | 需要真实茶芽、分级标准与样本量设计 |
| **P1** | ADC 新样本率与传感器任务匹配确认 | 当前目标后端在应用层走单次读取路径 | 需要在实物上用定时器触发确认 |
| **P2** | 第二关节与真实逆运动学 | 去掉小角度近似（工作区边缘误差 < 2 %） | 需要新的机械设计与运动学推导 |
| **P2** | 力/位混合控制 | 替代当前的阈值切换，接触过渡更平滑 | 需要平滑切换策略与稳定性分析 |
| **P2** | 自适应阈值实测标定 | 框架已就位，只缺真实数据 | 数据采集周期长 |
| **P3** | 手眼标定与真实视觉 | 用真实检测模型替换 `vision_mock.py` | 需要相机与标定板 |
| **P3** | 参数非易失存储 | 标定值当前复位即丢 | 需要 Flash 磨损均衡 |
| **P3** | 静态任务分配与栈分析 | 给出确定性内存图 | 需要把任务表改为编译期常量 |

---

## 15. 终端输出

```
========= PROJECT COMPLETE =========

Repository:
https://github.com/meminehobe24435-cmyk/second-gen-tea-picking-robot

C source files:
89 (25,939 lines, excluding vendored HAL/CMSIS/FreeRTOS)

Python source files:
35 (10,551 lines)

Documentation:
23 files (19,372 + ~2,000 lines)

Tests:
125 C unit tests             PASS
 22 Python tests             PASS
  7 C firmware scenarios     PASS
  7 Python simulator scenarios PASS

Build:
host firmware (gcc)          PASS  (667,533 bytes)
STM32F407 firmware (arm-none-eabi-gcc)
  text 57820 / data 52 / bss 25140   PASS

Simulation:
PASS  (full pick cycle, 2.17 N vs 2.20 N target, 0.00 N overshoot,
       peak 2.386 N, 0 deadline misses)

Documentation:
PASS

Git:
PASS

GitHub:
PASS

====================================
```

---

**报告结束**

*本项目所有结果均按来源分类标注。未实测的数字不会被写成成果；仿真结论不会被写成实测数据。*
