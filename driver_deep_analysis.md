# MPU6050 + OLED (SSD1306) 驱动深度解析

> 平台: MSPM0 (TI Cortex-M0+)
> 项目: mpu6050-oled-hardware-i2c
> 日期: 2026-05-05

---

## 目录

1. [物理与硬件边界](#1-物理与硬件边界)
2. [核心数据结构解析](#2-核心数据结构解析)
3. [生命周期与控制流](#3-生命周期与控制流)
4. [核心难点/精妙设计剖析](#4-核心难点精妙设计剖析)
5. [总结性调用链架构图](#5-总结性调用链架构图)

---

## 1. 物理与硬件边界

### 1.1 MPU6050 模块

**驱动硬件**: InvenSense MPU6050 —— 6 轴惯性测量单元（3 轴陀螺仪 + 3 轴加速度计），内置 DMP（Digital Motion Processor，数字运动处理器）。

**所解决业务问题**: 读出传感器的原始数据（角速度、加速度），进一步利用 DMP 计算出**四元数（quaternion）**，最终解算出 **Pitch（俯仰角）、Roll（横滚角）、Yaw（航向角）**。

**依赖的底层外设资源**:

| 资源 | SysConfig 名字 | 说明 |
|---|---|---|
| **I2C 控制器** | `I2C_MPU6050` | I2C 总线连接 MPU6050，硬件 I2C 地址 `0x68`，速率 Fast Mode (400kHz) |
| **GPIO 输入中断** | `PIN_MPU6050_INT` | 下降沿触发，优先级 Level 3。MPU6050 的 INT 引脚数据准备好时通知 MCU |
| **SysTick 时钟** | SysTick | 提供 `tick_ms` 变量，给 I2C 操作提供超时计数器，也给 `delay_ms` 使用 |

### 1.2 OLED（SSD1306）模块

**驱动硬件**: SSD1306 OLED 显示屏，128×64 像素，单色。

**所解决业务问题**: 在屏幕上显示字符、数字、汉字、图片等文本/图形信息。

**依赖的底层外设资源**:

| 资源 | SysConfig 名字 | 说明 |
|---|---|---|
| **I2C 控制器** | `I2C_OLED` | I2C 总线连接 OLED，设备地址 `0x3C`，速率 Fast Mode (400kHz) |
| **SysTick 时钟** | SysTick | 同上，为延时和超时服务 |

### 1.3 两个 I2C 外设——相互独立

MPU6050 使用 `I2C_MPU6050_INST`，OLED 使用 `I2C_OLED_INST`，**两个独立的 I2C 控制器**（硬件层面），不存在总线争用。它们在 SysConfig 中各配各的引脚和速率。

---

## 2. 核心数据结构解析

### 2.1 MPU6050 驱动层（inv_mpu.c）

整个 MPU6050 的驱动被抽象为 3 层:

```
应用层:  mpu6050.c         ← 用户代码
中间层:  inv_mpu.c/h       ← Invensense 官方寄存器层
硬件层:  mspm0_i2c.c/h     ← MSPM0 平台 I2C 实现
```

#### (1) `struct gyro_reg_s` —— MPU6050 寄存器映射表

```c
// 文件: Drivers/MPU6050/inv_mpu.c
struct gyro_reg_s {
    unsigned char who_am_i;       // 0x75, 读出来固定 0x68, 用于验证连接
    unsigned char rate_div;       // 0x19, 采样率分频器: 采样率 = 1000/(1+rate_div)
    unsigned char lpf;            // 0x1A, 数字低通滤波器 (DLPF) 配置
    unsigned char user_ctrl;      // 0x6A, 主控寄存器: 控制 FIFO/DMP/I2C Master 使能
    unsigned char fifo_en;        // 0x23, FIFO 使能: 决定哪些传感器数据入 FIFO
    unsigned char gyro_cfg;       // 0x1B, 陀螺仪配置: 满量程范围 (±250/500/1000/2000 °/s)
    unsigned char accel_cfg;      // 0x1C, 加速度计配置: 满量程 (±2/4/8/16 g)
    unsigned char fifo_count_h;   // 0x72, FIFO 计数值高字节
    unsigned char fifo_r_w;       // 0x74, FIFO 读写寄存器（先入先出）
    unsigned char raw_gyro;       // 0x43, 陀螺仪原始数据起始寄存器 (连续6字节: XH/XL/YH/YL/ZH/ZL)
    unsigned char raw_accel;      // 0x3B, 加速度计原始数据起始寄存器 (连续6字节)
    unsigned char int_enable;     // 0x38, 中断使能寄存器
    unsigned char int_status;     // 0x3A, 中断状态寄存器
    unsigned char pwr_mgmt_1;     // 0x6B, 电源管理 1: 复位、休眠、时钟源选择
    unsigned char pwr_mgmt_2;     // 0x6C, 电源管理 2: 各个轴的待机控制
    unsigned char int_pin_cfg;    // 0x37, 中断引脚配置: 电平/边沿、推挽/开漏
    unsigned char bank_sel;       // 0x6D, DMP 内存 bank 选择
    unsigned char mem_start_addr; // 0x6E, DMP 内存起始地址
    unsigned char mem_r_w;        // 0x6F, DMP 内存读写口
    unsigned char prgm_start_h;   // 0x70, DMP 程序启动地址
};
```

**工程意义**: 这不是代码逻辑结构，而是**硬件寄存器布局在软件中的镜像**。每个成员变量对应一个物理寄存器地址，`i2c_write` 和 `i2c_read` 通过这个结构体产生实际的 I2C 时序来操作这些寄存器。代码中直接使用 `st.reg->pwr_mgmt_1` 而非硬编码 `0x6B`，提高了可读性。

#### (2) `struct hw_s` —— 硬件特性表

```c
// inv_mpu.c (MPU6050 实例)
const struct hw_s hw = {
    .addr       = 0x68,          // I2C 从机地址
    .max_fifo   = 1024,          // FIFO 最大 1024 字节
    .num_reg    = 118,           // 有效寄存器数量
    .temp_sens  = 340,           // 温度传感器灵敏度: 每度变化 340 LSB
    .temp_offset= -521,          // 温度传感器偏移
    .bank_size  = 256            // DMP 内存每 bank 大小
};
```

#### (3) `struct chip_cfg_s` —— 运行时芯片配置状态

```c
// inv_mpu.c
struct chip_cfg_s {
    unsigned char gyro_fsr;      // 当前陀螺仪满量程档位 (0=250/1=500/2=1000/3=2000 dps)
    unsigned char accel_fsr;     // 当前加速度计满量程 (0=2g/1=4g/2=8g/3=16g)
    unsigned char sensors;       // 当前开启的传感器掩码
    unsigned char lpf;           // 数字低通滤波器配置值
    unsigned char clk_src;       // 时钟源 (PLL/内部振荡器)
    unsigned short sample_rate;  // 实际采样率 (Hz)
    unsigned char fifo_enable;   // 哪些传感器数据送入 FIFO
    unsigned char int_enable;    // 中断使能状态
    unsigned char bypass_mode;   // 旁路模式: 允许外部主机直连辅助 I2C
    unsigned char lp_accel_mode; // 低功耗加速度计模式标志
    unsigned char dmp_on;        // DMP 是否已开启
    unsigned char dmp_loaded;    // DMP 固件是否已加载
    unsigned short dmp_sample_rate; // DMP 模式下的固定采样率
};
```

**工程意义**: 这些值在 `MPU6050_Init()` 时与硬件同步。以后每次 `set_*()` 函数先检查缓存值，如果和硬件一致就**跳过 I2C 写入** —— 这是极常见的嵌入式优化，减少不必要的 I2C 写操作。

#### (4) 全局状态机 `st`

```c
static struct gyro_state_s st = {
    .reg = &reg,
    .hw = &hw,
    .test = &test
};
```

**`struct gyro_state_s`** 把所有寄存器映射、硬件参数、芯片配置、自检参数集中到一起，整个 inv_mpu.c 内部函数全部通过 `st` 变量访问硬件，无需传参。

#### (5) mpu6050.c 中的顶层全局变量

```c
// mpu6050.c
extern short gyro[3], accel[3];    // 全局暴露给 main 使用的原始传感器数据
extern float pitch, roll, yaw;     // 主程序最终使用的欧拉角
#define q30  (1073741824.0f)       // 2^30, 四元数定标因子
long quat[4];                      // DMP 输出的 32 位四元数 (Q30 格式)
```

这些全局变量通过 `main.h` 中的 `extern` 声明暴露给 main.c，main 中的 `while(1)` 直接读取它们显示到 OLED。

---

### 2.2 OLED 驱动层

OLED 驱动相对简单，没有复杂的结构体，核心是一套功能函数。

最关键的结构是**显存布局**（概念上的），由驱动函数隐式操作:

```
SSD1306 内部 GDDRAM 布局:
  页0 → 列0~127
  页1 → 列0~127
  ...
  页7 → 列0~127

每列 = 8 个像素点 (一个字节)，所以总共 8 页 × 128 字节 = 1024 字节
```

`OLED_Set_Pos(x, y)` 设置当前写入位置:
- `y` 是页地址（0~7）
- `x` 是列地址（0~127）

---

## 3. 生命周期与控制流

### 3.1 MPU6050 初始化链

#### `MPU6050_Init()` → 调用路径:

```
main()
  └─ MPU6050_Init()                           [mpu6050.c:124]
       ├─ mpu6050_i2c_sda_unlock()            [mspm0_i2c.c:37]
       │    SDA 解锁: 模拟 100 个 SCL 时钟脉冲直到 SDA 释放
       │
       ├─ mpu_init()                          [inv_mpu.c:714]
       │    ├─ 写 PWR_MGMT_1 = 0x80 → 硬件复位, delay(100ms)
       │    ├─ 写 PWR_MGMT_1 = 0x00 → 唤醒芯片
       │    ├─ mpu_set_gyro_fsr(2000)         ← 设陀螺仪 ±2000°/s
       │    ├─ mpu_set_accel_fsr(2)           ← 设加速度计 ±2g
       │    ├─ mpu_set_lpf(42)                ← 设 DLPF = 42Hz
       │    ├─ mpu_set_sample_rate(50)        ← 采样率 50Hz
       │    └─ mpu_configure_fifo(0)          ← 初始关闭 FIFO
       │
       ├─ mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)  ← 开启 6 轴
       ├─ mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) ← 6 轴数据入 FIFO
       ├─ mpu_set_sample_rate(50)
       │
       ├─ dmp_load_motion_driver_firmware()   ← 将 DMP 固件烧录到 MPU 内部 RAM
       │    内部调 mpu_load_firmware(), 逐块写入 MPU 内存, 再读回校验
       ├─ dmp_set_orientation(...)            ← 安装安装矩阵
       ├─ dmp_enable_feature(...)             ← 开启 6 轴四元数+TAP+屏幕旋转+陀螺校准
       ├─ dmp_set_fifo_rate(50)
       └─ mpu_set_dmp_state(1)                ← 启动 DMP 引擎
```

#### 初始化的关键设计——DMP 固件加载

`dmp_load_motion_driver_firmware()` 的内部机制在 `mpu_load_firmware()`:

```c
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate)
{
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);          // 每次 16 字节
        mpu_write_mem(ii, this_write, &firmware[ii]);       // 写入 DMP RAM
        mpu_read_mem(ii, this_write, cur);                  // 读回校验
        if (memcmp(firmware+ii, cur, this_write))           // 比对
            return -2;  // ← 固件烧录失败!
    }
    // 设置程序启动地址
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp);
}
```

**写后读校验**是工业级驱动固件下载的标准做法，防止 I2C 噪声导致固件损坏。

#### 中断注册

```c
// mpu6050.c:201
enable_group1_irq = 1;
```

这个标志让中断控制器使能 GPIO 中断。当 MPU6050 完成一次数据采集，INT 引脚产生下降沿，触发中断处理函数。但从 `main.c` 可以看到，`Read_Quad()` 是**轮询调用**的（在 while(1) 中每次都读），而不是在中断里读的。

### 3.2 OLED 初始化链

```c
OLED_Init()                           [oled_hardware_i2c.c:272]
  ├─ oled_i2c_sda_unlock()           ← 同样先解锁 SDA
  ├─ delay_ms(200);                   ← 等待 OLED 内部上电稳定
  ├─ OLED_WR_Byte(0xAE, OLED_CMD)    ← 关闭显示
  ├─ → 发送约 20 个配置命令 ←
  │    0x81 + 0xCF     → 设置对比度
  │    0xA1             → 列地址映射方向（正常）
  │    0xC8             → 行扫描方向（正常）
  │    0xA8 + 0x3F      → 复用比 1/64
  │    0xD5 + 0x80      → 时钟分频/振荡器频率
  │    0xD9 + 0xF1      → 预充电周期
  │    0xDA + 0x12      → COM 引脚硬件配置
  │    0x8D + 0x14      → 开启电荷泵
  │    ...
  ├─ OLED_Clear()                     ← 清屏（全写 0x00）
  └─ OLED_WR_Byte(0xAF, OLED_CMD)    ← 打开显示
```

### 3.3 主循环数据流

```c
// main.c
int main(void) {
    SYSCFG_DL_init();     // 系统时钟+外设初始化 (TI DriverLib)
    SysTick_Init();       // SysTick 定时器, 提供 tick_ms
    MPU6050_Init();       // 初始化 MPU6050 和 DMP
    OLED_Init();          // 初始化 OLED

    Interrupt_Init();     // 使能中断

    // 先刷 UI 标题
    OLED_ShowString(0, 7, "MPU6050 Demo", 8);   // 底部标题
    OLED_ShowString(0, 0, "Pitch", 8);           // 行标签
    OLED_ShowString(0, 2, " Roll", 8);
    OLED_ShowString(0, 4, "  Yaw", 8);

    while (1) {
        // Step 1: 从 DMP FIFO 读取新数据
        if (Read_Quad() == 0) {   // ← 关键: 读取 + 解算

            // Step 2: 用 sprintf 格式化为字符串
            sprintf(oled_buffer, "%-6.1f", pitch);
            // Step 3: 刷新 OLED 显示
            OLED_ShowString(5*8, 0, oled_buffer, 16);
            // ... 同样刷新 roll, yaw, accel[3], gyro[3]
        }
    }
}
```

`Read_Quad()` 内部:

```c
// mpu6050.c:204
int Read_Quad(void) {
    do {
        // 步骤 A: 从 DMP FIFO 取出一帧数据
        result = dmp_read_fifo(gyro, accel, quat,
                               &sensor_timestamp, &sensors, &more);
    } while(more);  // 步骤 B: FIFO 中如果还有剩余帧, 继续读, 直到清空

    // 步骤 C: 四元数 Q30 → 浮点
    float q0 = quat[0] / q30;
    float q1 = quat[1] / q30;
    float q2 = quat[2] / q30;
    float q3 = quat[3] / q30;

    // 步骤 D: 四元数 → 欧拉角 (弧度 × 57.3 = 度)
    pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,
                  -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                  q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
}
```

> **重要**: 数据流是**纯轮询**的。`Read_Quad()` 直接调用 `dmp_read_fifo()`，它内部通过 I2C 读 MPU6050 的 FIFO 寄存器（不是中断驱动），`while(more)` 循环确保 FIFO 被清空，防止积压。因为采样率只有 50Hz，每次进循环最多 1~2 帧，不会阻塞。

---

## 4. 核心难点/精妙设计剖析

### 4.1 SDA 锁定恢复机制（两个模块共用同一个设计模式）

**为什么需要这个？**

I2C 总线的一个经典坑: 如果 MCU 正在读从机时突然复位，而从机没有复位，SDA 线可能被从机保持在低电平（从机在输出应答位）。此时主设备无法发起 START 信号，整条总线瘫痪。

**MPU6050 的 SDA 解锁（mspm0_i2c.c:37-52）:**

```c
void mpu6050_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;

    // (1) 把 I2C 外设关掉, 把 SCL 引脚强制改为 GPIO 推挽输出
    mspm0_i2c_disable();

    do {
        // (2) 手动拉低 SCL → 保持至少 1ms
        DL_GPIO_clearPins(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
        mspm0_delay_ms(1);

        // (3) 释放 SCL → 保持 1ms (完成一个完整的时钟脉冲)
        DL_GPIO_setPins(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
        mspm0_delay_ms(1);

        // (4) 检查SDA是否已被从机释放(高电平)
        if(DL_GPIO_readPins(GPIO_I2C_MPU6050_SDA_PORT, GPIO_I2C_MPU6050_SDA_PIN))
            break;  // SDA 变高了 → 解锁成功

    } while(++cycleCnt < 100);   // 最多打 100 个脉冲

    // (5) 恢复 I2C 外设功能
    mspm0_i2c_enable();
}
```

**设计要点**:
- **为什么要 `mspm0_i2c_disable()`？**: 把 I2C 外设的控制权夺回，将 SCL 切换为普通 GPIO，否则 I2C 外设可能在等待总线空闲状态，不允许你手动发脉冲。
- **为什么要打 100 个脉冲？**: 从机可能在传输中途被中断，为了能让从机内部状态机回到 IDLE，需要给它完整的时钟周期来完成当前字节。100 个脉冲足够覆盖最长的情况（9 个时钟 × 字节数）。
- **为什么解锁后要再 enable？**: 恢复 I2C 外设功能，否则后续的 `i2c_write/read` 不能使用硬件 I2C。

OLED 的 `oled_i2c_sda_unlock()` 在 `oled_hardware_i2c.c:53-68` 是**一模一样的逻辑**，只是操作的寄存器换成了 `I2C_OLED_INST` / `GPIO_I2C_OLED` 这组外设。

**调用时机**: 每次初始化时和每次 I2C 操作超时时都会调用解锁函数。这是一种非常健壮的嵌入式防护。

### 4.2 `mspm0_i2c_write` —— 带超时和 SDA 自恢复的 I2C 写

```c
// mspm0_i2c.c:54-90
int mspm0_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
    unsigned int cnt = length;
    unsigned char const *ptr = data;
    unsigned long start, cur;

    if (!length) return 0;

    mspm0_get_clock_ms(&start);     // 记录起始时间

    // (1) 把要写入的寄存器地址预加载到 TXFIFO
    DL_I2C_transmitControllerData(I2C_MPU6050_INST, reg_addr);
    // 清除上一次的 TX_DONE 中断标志
    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    // (2) 等待 I2C 控制器回到空闲状态
    while (!(DL_I2C_getControllerStatus(I2C_MPU6050_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE));

    // (3) 发起 I2C START + 从机地址 + 写方向,
    //     length+1 是因为 reg_addr 算一字节
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, slave_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, length+1);

    // (4) 循环: 填充 TXFIFO, 直到 TX_DONE 中断标志置位
    do {
        unsigned fillcnt;
        fillcnt = DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;

        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) {
            mpu6050_i2c_sda_unlock();     // 超时 → 尝试恢复总线
            return -1;
        }
    } while (!DL_I2C_getRawInterruptStatus(I2C_MPU6050_INST,
                                            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    return 0;
}
```

**精妙之处**:

1. **寄存器地址预加载**: `DL_I2C_transmitControllerData(reg_addr)` 在 `startControllerTransfer` 之前就把寄存器地址塞进 TXFIFO。这是 MSPM0 的 I2C 设计——START 条件发送后，TXFIFO 里的第一个字节会被自动作为内部寄存器地址发送，后续字节作为数据。这样一条 I2C 时序就是 `START + slave_addr(W) + ACK + reg_addr + ACK + data1 + ACK + ... + STOP`，不用分成两条消息。

2. **超时 + 自恢复**: 10ms 超时是 I2C 的典型值 —— 400kHz 速率下传几百字节理论只需要几毫秒。如果超时，不是简单地返回错误，而是调用 `mpu6050_i2c_sda_unlock()` 来**实际恢复总线**。这意味着即使总线挂了，下次调用时总线已经是好的。

3. **`length+1`**: `startControllerTransfer` 传入 `length+1`，因为 MSPM0 的 I2C 会把预加载的 `reg_addr` 也算入总传输长度。

### 4.3 `mspm0_i2c_read` —— 组合读写 + 中断标志轮询

```c
// mspm0_i2c.c:92-151
int mspm0_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
    // (1) 预加载寄存器地址 → 发送给从机
    DL_I2C_transmitControllerData(I2C_MPU6050_INST, reg_addr);

    // (2) ★关键★: 设置 MCTR 寄存器, 让 TX 完成后自动发起 RESTART + 读
    I2C_MPU6050_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    //     ↑ 位操作: 使能 TX FIFO 空时自动切换为读方向

    // (3) 等待 IDLE → 发起复合传输 (RESTART)
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, slave_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, length);

    // (4) 轮询 RXFIFO 逐个字节读取
    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
            uint8_t c = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
            if (i < length) { data[i] = c; ++i; }
        }
        // 超时检测
        if (cur >= (start + I2C_TIMEOUT_MS)) {
            mpu6050_i2c_sda_unlock();
            return -1;
        }
    } while(!DL_I2C_getRawInterruptStatus(..., RX_DONE));

    // (5) 收尾: 防止 RXFIFO 还有滞留数据
    if (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) { ... }

    // (6) 清除 MCTR 配置, 刷 TXFIFO
    I2C_MPU6050_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
}
```

**为什么 `MCTR` 是核心**:

MPU6050 的 I2C 读操作是**复合格式（Combined Format）**:
```
START + 0xD0(W) + ACK + REG_ADDR + ACK + RESTART + 0xD1(R) + ACK + DATA1 + ACK + ... + DATA_N + NACK + STOP
```

设置 `MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE` 让 MSPM0 的 I2C 硬件在**发完 reg_addr 后自动产生 RESTART** 并把方向切为读。如果不设这一位，你需要在软件中先发写消息、停下来、再发起读消息——多一次仲裁，时序上还可能被其他中断干扰。

### 4.4 DMP 四元数 → 欧拉角转换（mpu6050.c）

```c
// mpu6050.c:229-236
float q0 = quat[0] / q30;   // Q30 定点 → 浮点
float q1 = quat[1] / q30;
float q2 = quat[2] / q30;
float q3 = quat[3] / q30;

// 标准四元数 → 欧拉角公式 (Tait-Bryan ZYX 顺序)
pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 弧度→度
roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,
              -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
yaw   = atan2(2 * (q1 * q2 + q0 * q3),
              q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
```

**为什么用 Q30 格式?** 从 DMP 输出的四元数是 **32 位定点数 Q30 格式**，即最高 2 位是符号位（实际只用到 30 位小数）。用整型传输可以避免浮点精度损失和浮点运算开销，等到 MCU 端再一次性转为浮点除一次。这里的 `#define q30 1073741824.0f` 正是 2^30。

**乘以 57.3 是因为**: `asin()` 和 `atan2()` 返回弧度值，57.3 ≈ 180/π。

### 4.5 `OLED_WR_Byte` —— OLED I2C 命令/数据发送

```c
// oled_hardware_i2c.c:101-133
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    unsigned char ptr[2];

    // 如果 mode=OLED_CMD(0):
    //   ptr[0] = 0x00  → SSD1306 将此字节解释为"后续是命令"
    //   ptr[1] = dat
    // 如果 mode=OLED_DATA(1):
    //   ptr[0] = 0x40  → SSD1306 将此字节解释为"后续是显示数据"
    //   ptr[1] = dat
    if(mode) {
        ptr[0] = 0x40;     // Co=1, D/C#=0 → Data stream
        ptr[1] = dat;
    } else {
        ptr[0] = 0x00;     // Co=0, D/C#=0 → Command
        ptr[1] = dat;
    }

    // 和 MPU6050 的写函数不同, OLED 没有预加载 reg_addr
    // 原因: SSD1306 的 I2C 协议第一个字节永远是 Co+D/C 控制字节
    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, ptr, 2);

    // 等待 I2C 空闲 → 发起传输
    DL_I2C_clearInterruptStatus(I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_OLED_INST, 0x3C,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    // 等待传输完成, 超时则解锁
    while (!DL_I2C_getRawInterruptStatus(I2C_OLED_INST,
                                          DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if (cur >= (start + I2C_TIMEOUT_MS)) {
            oled_i2c_sda_unlock();
            break;
        }
    }
}
```

**需注意**: SSD1306 的 I2C 协议不是标准寄存器地址方式。I2C 写入总是:

```
START + 0x3C(W) + ACK + 控制字节 + ACK + 数据字节 + ACK + STOP
```

控制字节的 bit0 是 `D/C#`: 0 = 后续字节是命令，1 = 后续字节是数据。所以 `0x00` 和 `0x40` 正是这个控制字节的两种取值。

与 MPU6050 不同，MPU6050 用的 `DL_I2C_transmitControllerData` 预加载寄存器地址做复合写。而 OLED 的驱动直接先 `fillControllerTXFIFO` 把两个字节都填好，再发起传输——因为第一个字节不是寄存器地址，而是控制字节。

### 4.6 OLED 字体渲染详解

```c
// oled_hardware_i2c.c:176-192
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c=0, sizex=sizey/2;
    uint16_t i=0, size1;

    if(sizey==8) size1=6;                       // 6x8 字体: 每个字符 6 字节
    else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2); // 8x16 字体: 每个字符 16 字节

    c = chr - ' ';  // 得到偏移后的值 (空格 ASCII=32, 数组索引从0开始)

    OLED_Set_Pos(x, y);
    for(i=0; i<size1; i++) {
        if(i%sizex==0 && sizey!=8)
            OLED_Set_Pos(x, y++);     // 8x16: 每写一半(8字节)换到下一行
        if(sizey==8)
            OLED_WR_Byte(asc2_0806[c][i], OLED_DATA); // 6x8 字号
        else if(sizey==16)
            OLED_WR_Byte(asc2_1608[c][i], OLED_DATA); // 8x16 字号
    }
}
```

**字体表结构** (`oledfont.h`):
- `asc2_0806[95][6]`: 95 个 ASCII 字符（从空格 ` ` 到 `~`），每个 6 字节宽
- `asc2_1608[95][16]`: 95 个 ASCII 字符，每个 16 字节（上半 8 字节 + 下半 8 字节）
- `Hzk[][32]`: 中文字库，每个汉字 32 字节（16×16 点阵）

**渲染原理**: 对于 8x16 字体，一个字符需要 16 字节描述，其中前 8 字节是上半部分（8 列 × 8 行 = 64 像素 = 8 字节），后 8 字节是下半部分。`OLED_Set_Pos` 在写完上半部分后自动跳到下一页，继续写下半部分。

---

## 5. 总结性调用链架构图

```
main()
 │
 ├── MPU6050_Init()
 │    ├── mpu6050_i2c_sda_unlock()     // 解锁可能锁死的 I2C 总线
 │    │    └── mspm0_i2c_disable/enable // GPIO 模式切换
 │    ├── mpu_init()                    // 复位+配置寄存器 (inv_mpu.c)
 │    │    └── i2c_write → mspm0_i2c_write() // 底层 I2C (mspm0_i2c.c)
 │    ├── mpu_set_sensors()             // 开启 6 轴传感器
 │    ├── mpu_configure_fifo()          // 配置 FIFO 数据源
 │    ├── dmp_load_motion_driver_firmware() // 烧录 DMP 固件
 │    │    └── mpu_write_mem + mpu_read_mem (写后读校验)
 │    └── mpu_set_dmp_state(1)          // 启动 DMP
 │
 ├── OLED_Init()
 │    ├── oled_i2c_sda_unlock()         // 同上, 另一套 I2C 总线
 │    │    └── mspm0_i2c_disable/enable
 │    ├── delay_ms(200)                 // 等待 OLED 上电稳定
 │    └── ~20 条 OLED_WR_Byte(CMD)     // 配置 SSD1306 寄存器
 │
 └── while(1)
      ├── Read_Quad()                   // 读 DMP FIFO
      │    ├── dmp_read_fifo()          // 通过 i2c_read 读 FIFO
      │    └── 四元数 → 欧拉角          // 数学转换
      ├── sprintf()                     // 格式化
      └── OLED_ShowString()             // 显示到屏幕
           └── OLED_ShowChar()
                └── OLED_Set_Pos()
                     └── OLED_WR_Byte()  // 发送 I2C 命令/数据
```

---

## 附: 各文件职责速查

| 文件 | 职责 |
|---|---|
| `main.c` | 顶层应用逻辑: 初始化, 主循环读传感器 + 显示 |
| `mpu6050.c` | MPU6050 高层封装: Init, Read_Quad (含四元数→欧拉角) |
| `inv_mpu.c` | Invensense 官方寄存器层: 所有寄存器操作, DMP 固件加载 |
| `inv_mpu_dmp_motion_driver.c` | DMP 固件镜像 + FIFO 数据解析 |
| `mspm0_i2c.c` | MSPM0 平台 I2C 读写实现 + SDA 总线解锁 |
| `oled_hardware_i2c.c` | SSD1306 OLED 驱动: 初始化,I2C 字节发送,字符/字符串/图片显示 |
| `oledfont.h` | ASCII 和中文字库点阵数据 |
| `clock.c/h` | SysTick 时钟服务: ms 延时, 时间戳 |
| `interrupt.c/h` | 中断控制器: GPIO 中断使能 |
