# 多模式循迹与航向闭环小车

基于 TI MSPM0G3507 的多模式循迹与航向闭环小车固件。系统融合 8 路灰度传感器、MPU6050 DMP 航向角、双路编码器和分层 PID 控制，在直线盲走、弧线循迹及 8 字路径之间切换。

## 功能概览

- MSPM0G3507 裸机控制，使用 TI DriverLib 与 SysConfig。
- 双电机编码器速度闭环，20 ms 定时中断更新速度控制。
- MPU6050 航向角闭环，用于直线和对角线段的方向保持。
- 8 路灰度传感器循迹，通过位置误差 PID 实现弧线跟踪。
- OLED 显示任务编号、航向角和故障状态。
- 编码器失速保护、主动刹车及蜂鸣器/LED 状态提示。

## 四种运行模式

| 模式 | 路径与行为 |
|---|---|
| Task 1 | A → B，航向闭环盲走，检测到目标黑线后停车。 |
| Task 2 | A → B → C → D → A，直线航向控制与弧线循迹交替运行。 |
| Task 3 | 8 字路径 A → C → B → D → A，对角线采用粗定位与黑线精确捕获两阶段策略。 |
| Task 4 | 连续 4 圈 8 字路径，使用独立高速 PID 参数和逐圈航向补偿。 |

## 控制结构

- `main.c`：系统初始化、按键扫描、故障处理和任务调度。
- `User/control.c`：四种任务的状态机。
- `User/track_ctrl.c`：航向闭环、灰度循迹、任务启动和停车逻辑。
- `User/pid.c`：速度、航向和循迹 PID。
- `User/encoder.c`、`User/kinematics.c`：编码器测速与里程估计。
- `Drivers/MPU6050/`：MPU6050 与 DMP 姿态解算。
- `Drivers/OLED_Hardware_I2C/`：OLED 硬件 I2C 驱动。
- `mpu6050-oled-hardware-i2c.syscfg`：引脚、外设、定时器和中断配置源文件。

## 演示视频

- [循迹与航向闭环小车演示](media/demos/line-tracking-demo.mp4)

## 开发环境

- TI Code Composer Studio 20.5
- TI Arm Clang 4.0.4.LTS
- MSPM0 SDK 2.10.0.04
- SysConfig 1.27.0
- 目标器件：MSPM0G3507
- 当前目标配置：`targetConfigs/MSPM0G3507.ccxml`（SEGGER J-Link）

## 编译与烧录

在 Code Composer Studio 中导入仓库根目录的工程，或使用仓库附带的 VS Code 任务。具体步骤见 [CCS 使用说明](CCS使用说明.md)。

`Debug/` 中的 `.out`、`.map`、`.o` 和自动生成的 SysConfig 文件属于本机构建产物，不提交到仓库。构建时以 `mpu6050-oled-hardware-i2c.syscfg` 为配置源重新生成。

烧录前请确认实际调试器与 `targetConfigs/MSPM0G3507.ccxml` 一致，并在小车悬空或赛道清空的情况下首次运行。固件启动任务后会驱动电机。

## 验证说明

仓库上传前已完成 MSPM0 SysConfig 静态检查，工程配置、生成初始化函数和应用调用一致。现有工程说明记录了 CCS 完整构建通过；本次整理未连接硬件重新烧录或进行实车复测。

