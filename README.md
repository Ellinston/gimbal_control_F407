# gimbal_1：STM32F407 双 GM6020 云台

更新：2026-10-14。当前工程为 `D:/STM32IDE_workplace/gimbal_1`，已在 CubeMX 中创建并生成基础代码。目标是在硬件组 F407 开发板上逐步完成双 GM6020 云台，最终满足本文所列电控组考核要求。

## 当前状态与资料

**当前源码模式（2026-10-14 IMU自稳）：main 调度 IMU_Angle_Test()、IMU_Stabilize_Task 和 vofa_task_update()。上电静止标定后自动启动：位置环使用顶部IMU，PITCH目标1.483491°（用户水平样本），YAW保持启用时朝向；速度环继续使用编码器，原参数区不变。USART1 115200/8N1，六路JustFloat每100ms输出；STOP加换行停机，复位重新启动。编译、主机回归、烧录校验及实采通过，用户确认前后倾斜时能向水平恢复；任意横滚与精密水平误差未验收。详见 [IMU自稳抗扰动](说明书/IMU自稳抗扰动.md)。**

历史固定1000单轴试验曾采集到正向9..12rpm，用户确认两次独立电机上电均自行启动，见 [固定1000运行记录](说明书/YAW固定1000运行.md)。这些证据仅适用于历史单轴固件。

- 当前配置：[gimbal_1.ioc](gimbal_1.ioc)。实际代码以 [main.c](Core/Src/main.c)、[MSP](Core/Src/stm32f4xx_hal_msp.c)、[中断文件](Core/Src/stm32f4xx_it.c)和 [CMake 源码清单](cmake/stm32cubemx/CMakeLists.txt)为准。
- 自行构建操作：[CLion 编译与烧录操作指南](说明书/CLion编译与烧录操作指南.md)，包含日常构建、结果判断、新增源文件和可选烧录步骤。
- 硬件依据：[F407 电路板原理图](说明书/F407电路板原理图.pdf)，覆盖供电、引脚及板载连接。
- 电机依据：[RoboMaster GM6020 使用说明，2023-10-13 / v1.4](说明书/RoboMaster%20GM6020直流无刷电机使用说明20231013.pdf)，覆盖 CAN 协议、拨码 ID、控制模式与使用条件。
- 考核依据：[鲸麟战队 26 招新完整形态考核](说明书/鲸麟战队26招新%20完整形态考核.docx)，以其中电控组要求为最终验收目标。
- 已完成串口和 BMI270 驱动；2026-10-12 使用 ARM GCC 13.3.1、CMake 和 Ninja 编译链接，并通过 ST-Link/OpenOCD 烧录、校验和复位，产物为 `build/Debug/gimbal_1.elf`。串口及 IMU 已通过 COM6 实物检查，详细证据见 [IMU 模块验收](说明书/IMU模块验收.md)。
- USART1 中断收发及 `PING\n → PONG\n` 测试源码保留，当前调度 IMU 角度任务及 `Task/usart_test` 中的 VOFA 任务，不调度 PING 测试。BMI270 完整驱动位于 [device/BMI270](device/BMI270)。串口接口见 [串口模块验收](说明书/串口模块验收.md)。当前输出六通道 JustFloat，不混入启动诊断或周期文本；自稳任务接收 STOP，原串口位置任务的 SET / STOP 保留供切回。

电机驱动与任务接口见 [电机模块接口](说明书/电机模块接口.md)：`motor.c` 提供 GM6020 电压组帧和反馈解析，`motor_control.c` 提供速度/角度串级、速度前馈及 `Motor_PID_Control` 接口；[固定目标位置任务](说明书/PID固定目标位置环.md)、旧开环、固定1000、单轴 PID 和双轴往返测试源码保留；当前调度 IMU 自稳任务，原串口双轴位置任务保留。

## 协作方法

1. 用户规定每个模块的接口定义，包括函数名称、参数、返回值、数据含义与单位；文件位置和模块边界也按用户要求确定。缺少会影响实现的接口信息时，先补齐定义，不自行恢复旧工程框架。
2. AI 根据上述两份说明书和当前工程，说明当前模块需要怎样接线、怎样设置 CubeMX、时钟、引脚、中断及相关参数。说明书未覆盖的内容应明确指出，并查阅相应芯片官方资料；例如 BMI270 寄存器、初始化配置数据、SPI 时序不能仅靠电路板原理图确定。
3. 接口确定后，AI 负责具体实现、构建接入和调试。每次修改前说明本步目标、涉及文件和调用关系；保留 CubeMX USER CODE 区域，新增源文件时同步维护 CMake。
4. 每次只推进当前模块，按“配置核对 → 实现 → 编译链接 → 烧录校验 → 实机采集与测试 → 记录结论”完成验证。用户已授权 AI 在本工程范围内直接执行必要的烧录和实机验证，无需每次重复确认；接线、供电、机械固定等无法由 AI 操作的部分由用户配合。
5. AI 优先使用 ST-Link/OpenOCD、串口日志、反馈数据、VOFA 波形和可用仪器直接验证。设备不可访问或缺少观测手段时，状态保持“待实机验证”，说明缺少的条件，不跳到下一模块。寄存器翻转不能单独证明 LED 真的亮，CAN 发送成功不能单独证明电机已经转动。
6. 当前模块实机验收通过并记录证据后，再进入下一模块；下一模块仍先由用户规定接口。记录实际源码、构建产物、实验条件、通过项与未通过项。不得以编译、烧录成功替代功能验收。

## 模块顺序与验收

按用户指定的顺序推进：**串口测试 → IMU → CAN → 转动单电机 → 转动双电机 → PID → 合并模块**。

| 顺序 | 模块    | 本阶段目标与实机验收                                                           | 当前状态 |
|----|-------|----------------------------------------------------------------------|------|
| 1  | 串口测试  | USART1 双向通信；AI 发送已知数据并核对板端回复，验证重复收发及约定的异常输入处理                        | 编译、烧录、主机测试及 COM6 实测通过，25 项串口检查通过 |
| 2  | IMU   | BMI270 通信、初始化、原始加速度/角速度读取及标定；验证静止和已知方向转动的数据，检查采样周期与连续性               | 驱动、烧录、连续采样、静止零偏估计及手动转动响应通过；轴方向对应和完整标定待验证 |
| 3  | CAN   | 配置过滤器、启动和通知，解析电机反馈；确认 ID、帧长度、字节序、角度、速度与超时处理，并通过串口/VOFA 观测；本阶段不主动驱动转动 | 通用接口、双路原始缓存及 COM6 打印实测通过；角度/速度解析与 VOFA 待后续接口设计 |
| 4  | 转动单电机 | 当前按用户最新要求上电固定1000，持续发送与串口反馈 | 编译、烧录和转动反馈通过；用户确认两次独立电机上电均自行启动；速度闭环和全位置负载验收待后续 |
| 5  | 转动双电机 | 验证两轴分别及同时响应，一帧完整控制报文正确分配各轴槽位，未使用槽归零，两轴反馈与停机保护有效                      | 双轴代码、编译与模拟检查通过；未烧录、未实测 |
| 6  | PID   | 先速度闭环与正弦目标跟随，再双轴位置闭环；记录参数、周期、限幅、目标/实际曲线和停机测试结果                       | 按本次要求提前实现双轴位置测试及速度接口；硬件验收待完成 |
| 7  | 合并模块  | 集成姿态解算、自稳、手动/视觉目标接口与模式切换，验证模块协作及统一保护；逐项完成考核演示和证据整理                   | 未开始  |

IMU 阶段先证明传感器工作；基础自稳需要在合并阶段结合姿态解算、双轴控制和实际机械关系完成。最终速度/位置闭环、自稳和视觉联动均需独立验收，单个基础模块通过不代表整机通过。

首次电机转动前必须确认电机控制模式、机构固定、方向、输出限幅、测试时长和可用的急停/断电方式。停机、反馈超时、发送故障与测试结束时控制输出归零；具体阈值按当轮接口和实机条件确定。

## 硬件配置

以下参数已根据当前 `gimbal_1.ioc`、初始化代码与板卡原理图核对。当前基础代码由本工程生成；后续按模块增加配置和功能，不复制旧工程代码、IOC 或构建缓存。

### 1. 芯片、晶振与调试引脚

芯片选择 **STM32F407VET6，LQFP100 封装**。

| CubeMX 中的位置 | 设置 | 简单说明 |
|---|---|---|
| System Core → RCC → High Speed Clock (HSE) | Crystal/Ceramic Resonator | 板上有 8 MHz 晶振，使用外部晶振模式 |
| PH0 / PH1 | RCC_OSC_IN / RCC_OSC_OUT | 晶振连接引脚，由 RCC 配置分配 |
| System Core → SYS → Debug | Serial Wire | 使用 ST-Link 的 SWD 下载和调试 |
| PA13 / PA14 | SWDIO / SWCLK | 留给 ST-Link，不改成普通 GPIO |
| SYS → Timebase Source | SysTick | 保持当前系统毫秒计时来源 |

ST-Link 接 SWDIO、SWCLK 和 GND；参考电压按板卡实际连接核对。原理图的 SWD 接口未引出 NRST。[openocd.cfg](openocd.cfg) 使用 ST-Link、SWD、stm32f4x 和软件复位；已验证当前 ELF 的烧录与校验。

### 2. Clock Configuration：时钟

进入 **Clock Configuration**，按表核对。MHz 是时钟频率单位，8 MHz 输入经 PLL 倍频后得到 168 MHz 主时钟。

| 项目 | 数值 / 选择 |
|---|---|
| HSE 输入频率 | 8 MHz |
| PLL Source | HSE |
| PLLM | 8 |
| PLLN | 336 |
| PLLP | 2 |
| PLLQ | 4，保留当前值 |
| System Clock Mux | PLLCLK |
| SYSCLK | 168 MHz |
| AHB Prescaler | /1 |
| HCLK | 168 MHz |
| APB1 Prescaler | /4 |
| APB1 Peripheral Clock（PCLK1） | 42 MHz，CAN1 使用这个时钟 |
| APB2 Prescaler | /2 |
| APB2 Peripheral Clock（PCLK2） | 84 MHz，USART1 使用这个时钟 |

主时钟计算：8 ÷ 8 × 336 ÷ 2 = **168 MHz**。

注意看 APB1 的 **Peripheral Clock 42 MHz**，不要把旁边的 Timer Clock 84 MHz 用于 CAN 波特率计算。当前 PLLQ 分支为 84 MHz，USB 尚未启用；本轮保持配置，不为未来 USB 功能顺手修改时钟。

### 3. Connectivity → CAN1：两个电机共用的总线

启用 **CAN1 → Activated**，Mode 选择 **Normal**。两个电机接同一条 CAN1 总线。

| 项目 | 设置 | 简单说明 |
|---|---|---|
| CAN1_RX | PD0，AF9 | 单片机接收 CAN 数据的引脚 |
| CAN1_TX | PD1，AF9 | 单片机发送 CAN 数据的引脚 |
| GPIO Mode / Pull / Speed | Alternate Function Push Pull / No Pull / Very High | 与当前 MSP 初始化一致 |
| Prescaler | 3 | CAN 时钟分频 |
| Time Quanta in Bit Segment 1（BS1） | 11 TQ | 位时序第一段 |
| Time Quanta in Bit Segment 2（BS2） | 2 TQ | 位时序第二段 |
| Resynchronization Jump Width（SJW） | 1 TQ | 重同步宽度 |
| Auto Retransmission | Enable | 发送失败时允许自动重发 |
| Auto Bus-Off | Enable | 保留自动离线恢复 |
| Time Triggered Mode | Disable | 保持关闭 |
| Auto Wake-Up | Disable | 保持关闭 |
| Receive FIFO Locked | Disable | 保持关闭 |
| Transmit FIFO Priority | Disable | 保持关闭 |

波特率计算：42 MHz ÷ [3 × (1 + 11 + 2)] = **1 Mbps**，即 1000000 bit/s。括号中的 1 是固定同步段。

在 CAN1 的 **NVIC Settings** 中勾选接收 FIFO0 和发送中断，优先级见第 5 节。过滤器、CAN 启动和中断通知现由 `Bsp/can/can.c` 实现；设备编号、缓存分类与串口打印位于 `Task/can_test.c`。

接线保持原样：单片机 PD0/PD1 连接板上的 CAN 收发器，电机连接收发器侧 CAN_H / CAN_L；电机供电 24 V，板上 SIT1050T 收发器供电 5 V，信号共地。总线物理两端各保留一个 120 Ω 终端电阻。

| 轴 | 电机 ID | 反馈标准帧 ID | 控制帧位置 |
|---|---:|---|---|
| YAW | 2 | 0x206 | 0x1FF 的第 2 个 16 位槽，字节 2、3 |
| PITCH | 1 | 0x205 | 0x1FF 的第 1 个 16 位槽，字节 0、1 |

电机 ID 在电机端设置，不在 IOC 中设置；表中沿用当前源码分配（YAW ID2、PITCH ID1），实机接入时核对拨码和反馈 ID。按本项目指定的 v1.4 说明书，0x1FF 是 8 字节的电压原始指令帧，数值范围为 -25000～25000，不是伏特；剩余两个槽置零。说明书也规定了电流控制帧；进入驱动阶段前先确认电机实际控制模式，不能混用电压帧和电流帧。反馈为标准数据帧，DLC=8，标称发送频率 1 kHz，机械角度范围 0～8191，速度单位 rpm。

### 4. Connectivity → USART1：电脑串口

启用 **USART1 → Asynchronous**。现有接线既能向电脑输出 VOFA 数据，也能从电脑接收数据；具体命令协议之后重新规定。

| 项目 | 设置 |
|---|---|
| USART1_TX | PA9，AF7 |
| USART1_RX | PA10，AF7 |
| GPIO Mode / Pull / Speed | Alternate Function Push Pull / No Pull / Very High |
| Baud Rate | 115200 Bits/s |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit（TX / RX） |
| Hardware Flow Control | None |
| Oversampling | 16 |
| NVIC → USART1 global interrupt | Enable，抢占优先级 2，子优先级 0 |
| DMA | 当前未配置，保持不启用 |

**接线交叉连接**：USB 转串口模块 TX → 板上 PA10（RX）；模块 RX → 板上 PA9（TX）；GND → GND。使用与 MCU 电平匹配的 TTL 串口。

VOFA 端的串口参数同样设为 **115200、8N1**。COM 号按当前电脑识别结果选择。输出格式和命令协议由用户在串口模块接口中规定，不属于 IOC 选项。

### 5. System Core → NVIC：中断核对表

优先级分组保持 **NVIC_PRIORITYGROUP_4**。表中的数字越小，抢占优先级越高。

| 中断 | 是否启用 | 抢占优先级 | 子优先级 | 用途 |
|---|---|---:|---:|---|
| CAN1 RX0 interrupt | 是 | 0 | 0 | 接收 FIFO0 数据 |
| CAN1 TX interrupt | 是 | 0 | 0 | 发送完成处理与诊断 |
| USART1 global interrupt | 是 | 2 | 0 | 串口中断收发 |
| System tick timer（SysTick） | 是 | 15 | 0 | 系统毫秒计时 |

### 6. LED、IMU 与后续外设

当前 IOC 启用了 **RCC、SYS、NVIC、CAN1、USART1、SPI1**。当前 IMU/VOFA 任务使用 SPI1 与 USART1；CAN1 与 IMU 自稳任务同时调度，使用现有串级速度环及前馈。

**LED：原理图第 1 页的 LED4、LED5、LED6 分别直接连接 3.3 V、5 V、24 V 电源，经限流电阻接地，未连接 MCU GPIO，属于电源指示灯。不能通过程序控制它们亮灭。** LED 模块需由用户确定外接 LED 的可用 GPIO、限流电阻及有效电平，或者提供实物上另外存在的可控 LED 接线；确定后再给出 CubeMX 配置。

BMI270 已启用 SPI1：CS PA3、INT1 PA4、SCK PA5、MISO PA6、MOSI PA7。SPI 使用 Mode 3、8 位、MSB first、软件片选、84 MHz / 64 = 1.3125 MHz；PA3 上电输出高，PA4 保留为普通输入，不启用 EXTI/DMA。IMU 驱动配置为加速度 ±4g、陀螺仪 ±2000°/s、均为 100 Hz，当前主循环持续调度 IMU 与自稳任务。CAN2、USB、第二路控制器 / 视觉串口和激光控制未启用。

旧资料中有“PB10/PB11 对应 USART2”的错误记录，不能按它添加 USART2；第二路串口的外设名称与引脚映射须另行确认。这不影响现有 USART1。

### 7. 工程生成与构建

当前 IOC 记录为 CubeMX **6.15.0**、STM32Cube FW_F4 **V1.28.3**，导出方式为 **CMake**，工程名为 **gimbal_1**。

- 后续在本工程 IOC 中逐模块调整外设，使用 **Keep User Code when re-generating** 保留用户代码；当前此选项已启用。
- 启动文件、HAL/CMSIS、初始化、中断和链接脚本使用本工程生成的版本。
- 使用 **CLion + CMake + ARM GCC** 构建 F407 固件，当前预设使用 **Ninja**。`CMakePresets.json` 提供 Debug/Release，工具链为 `cmake/gcc-arm-none-eabi.cmake`；CMake、Ninja 和 ARM GCC 路径按本机环境配置。
- 常规命令为 `cmake --preset Debug`、`cmake --build --preset Debug`，前提是相关工具已配置到环境或 CLion 工具链中。
- 用户新增的 `.c` 文件需加入根目录 `CMakeLists.txt` 的 `target_sources`；新增头文件目录需加入 `target_include_directories`，随后重新配置并构建。

## 考核要求

按本目录考核原文电控组要求推进：基础项共 **50 分**，提高项最高 **30 分**。

| 考核项 | 分值 | 验收要求 |
|---|---:|---|
| CAN 通信、反馈与 VOFA | 10 分 | 两轴稳定接收并解析角度、速度等状态，VOFA 连续显示反馈曲线 |
| 电机控制 / 正弦速度 | 10 分 | 代码中写定正弦目标转速，电机正确运行并提供 VOFA 波形；本工程用速度闭环实现 |
| 双轴位置闭环 | 10 分 | 下发 yaw、pitch 目标角度后稳定到位，无持续振荡或严重超调 |
| 与视觉上位机通信 | 10 分 | 正确接收上位机目标数据；下发 yaw、pitch 角度后，电机云台能正确响应至对应位置 |
| 工程化 | 5 分 | HAL、通信协议、电机驱动、控制算法、应用分层清楚，参数集中管理 |
| IMU 基础自稳 | 10 分 | 完成 BMI270 驱动、标定与姿态解算；举起包含 YAW、PITCH 电机的云台时，仍能保持顶部平台水平；未规定水平误差 |
| 迁移至硬件组开发板 | 25 分 | 在自制 F407 板上完成外设适配与实机运行，接线、反馈和基本保护有效 |

位置控制测试方式：使上位机向下位机发送 yaw、pitch 角度数据，电机云台能正确响应至对应位置。

自稳测试以云台顶部平台相对地面保持水平为准；举起或倾斜底座后，仅保持电机编码器角度不能证明自稳。IMU 安装方向、坐标关系、机械零点和限位在本工程实机阶段重新标定，底座倾斜方向与范围尚未量化。

考核原文还描述了手动瞄准、激光笔与视觉自瞄的理想演示形态；激光笔未单列分值，其必做性待确认，不擅自新增评分要求。

角度误差、到位时间、正弦周期及连续运行时长：考核指标待定。

考核以实机表现与波形为依据；编译、软件测试和烧录校验分别记录，不能替代实机验收。

## 检查与排错约定

- 以本目录实际源码、IOC、构建目录和 ELF 为依据；历史工程只供参考，不恢复旧演示状态机、命令协议或调参值。
- F407 使用 `HAL_CAN_*`（bxCAN）。核对 IOC、MSP、IRQHandler、HAL 通知与用户回调的完整链路；每种用户回调只保留一个有效定义，电机控制帧保持单一发送入口。
- 当前为 CMake 显式源码清单。出现 `undefined reference` 时先检查实现是否存在、是否加入目标以及是否重新配置；不能用编辑器能跳转证明链接成功。
- 核对 `USE_HAL_DRIVER`、`STM32F407xx`、HAL 模块开关和 HAL/CMSIS 头文件路径。PC 上的测试与 ARM 固件构建分别记录。
- 异步串口发送缓冲区在发送结束前保持有效且不改写；AI 采集工具与 VOFA 不同时占用同一个 COM 端口。
- OpenOCD 连接失败时先核对板卡供电、SWD 接线、目标芯片、适配器速度和当前配置。不能从连接失败推断业务代码异常；不结束用户现有调试会话来试错。
- 读取日志先核对编码；重新链接后用当前 ELF 符号定位调试变量，不复用旧 RAM 地址。
- 分别记录计算输出、HAL 入队、CAN 发送完成、实际反馈与电机运动。总线上其他节点也可能 ACK，不能仅凭 ACK 判定目标电机执行。
- 当前目录没有 Git 元数据；修改按实际文件核对范围，需要回退时使用有来源的文件备份，不假定可用 Git 恢复。

## 当前下一步

纯 SPI CHIP_ID、CS GPIO 回读、DWT 微秒延时及 SysTick 交叉检查均已由用户确认通过，驱动 ID 预测试通过板端检查。当前完整 `BMI270_init()` 返回成功，9 项配置回读全部符合预期，新增串口输出待用户确认。之后按用户要求推进原始样本读取。完整 IMU 验收记录属于此前实验，不能替代后续分步验收。

