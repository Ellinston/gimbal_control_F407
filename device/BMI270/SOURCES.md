# BMI270 模块来源

分层依据：本工程 `说明书/模块思路.md` 与 `E:/DM_basic_frame/device/BMI088`。BMI270 的单芯片寄存器、统一片选、SPI dummy byte、配置上传及初始化流程按 Bosch 官方资料重新实现，没有将 BMI088 的双片选时序直接套用。

- [Bosch BMI270 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
- [Bosch BMI270 SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI)
- 配置数据：`bmi270.c` v2.86.1 中的完整 8192 字节 `bmi270_config_file`；以 GitHub 文件 blob SHA `20f27bb0aad4b9c41160759fff9dde989f4d90e9` 为来源标识。
- 初始化与分块上传参考：官方 `bmi2.c`，blob SHA `bbbf186e2d99aade20b3253711972bac14b1ecc4`。
- 寄存器与配置值核对：官方 `bmi2_defs.h`，blob SHA `0bc3f913033bb65632a3de83b96c778c422a847e`。

下载/核对日期：2026-10-12。`BMI270config.h` 保留原始 BSD-3-Clause 版权与许可声明，配置数组未修改。编译期检查数组为 8192 字节，主机测试逐字节核对上传数据及分块地址。

对外接口：`BMI270_init()`、`BMI270_read()`；`bmi270_diag` 为 Task 和调试器提供只读诊断约定。修改或复制本模块时一并保留本文件及配置头文件中的许可文本。
