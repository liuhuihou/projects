# S28A + C07A 小车硬件通路与接线说明

## 1. 适用范围

本文档对应以下组合：

- C07A 核心板：MSPM0G3507SPTR，优先使用 C07A V1.1。
- S28A 底板：资源分配表和适配原理图中的 C07A 插接方式。
- 电机驱动：D103A 精简版 TB6612 模块。
- 姿态模块：MPU6050。
- 显示屏：S28A OLED 接口对应的串行 OLED 模块。
- 无线串口：蓝牙模块，当前工程配置 9600 8N1。
- 待增加：荣洋电子 RYZD 八路红外板（使用中间六路物理通道 CH2、CH3、CH4、CH5、CH6、CH7）、K230D box。

已审阅的资料包括 `相关资料/3.原理图` 下 C07A V1.0、V1.1、48-pin 原理图、封装 JSON、版本和按键说明，以及 `相关资料/7.S28A底板相关资料` 下 S28A 资源分配表、适配原理图、丝印/实物资料、S28A 压缩包、MPU6050、P03B 电源、TB6612 模块原理图和 TB6612FNG 手册。

## 2. 核心板与底板

C07A 插入 S28A 的 H1/H2 两排接口。S28A H1/H2 的信号按原理图如下，插接时必须按丝印方向确认 1 脚，不能仅按线缆颜色判断：

| 接口 | 引脚 | 信号 |
|---|---:|---|
| H1 | 1 | 5V |
| H1 | 2 | GND |
| H1 | 3 | PA25 |
| H1 | 4 | PA26 |
| H1 | 5 | PA1 |
| H1 | 6 | PA0 |
| H1 | 7 | PB17 |
| H1 | 8 | PB16 |
| H1 | 9 | PA12 |
| H1 | 10 | PA8 |
| H1 | 11 | PA9 |
| H1 | 12 | PA27 |
| H1 | 13 | PA24 |
| H1 | 14 | GND |
| H1 | 15 | 3V3 |
| H2 | 1 | 3V3 |
| H2 | 2 | GND |
| H2 | 3 | PA7 |
| H2 | 4 | PB2 |
| H2 | 5 | PB3 |
| H2 | 6 | PA17 |
| H2 | 7 | PA16 |
| H2 | 8 | PA14 |
| H2 | 9 | PA13 |
| H2 | 10 | PB7 |
| H2 | 11 | PB6 |
| H2 | 12 | PA22 |
| H2 | 13 | PA15 |
| H2 | 14 | PB20 |
| H2 | 15 | PB24 |

重要：`H1/H2` 是 C07A 插入 S28A 的板间连接排。C07A 安装后这两个接口已经被占用，下面的 H1/H2 表仅用于查线，不能再把传感器线插到 H1/H2。

说明：C07A V1.1 的 J1 是额外 3 针 IO 插座，信号为 `PB8`、`PB18`、`PB19`。本方案使用 `PB18` 连接 K230；`PB8` 仍作为 S28A 按键输入，`PB19` 可作为额外扩展信号。若使用 C07A V1.0，必须先确认是否有同样的 J1 引出，不能默认存在。

## 3. 已连接硬件

### 3.1 TB6612 与两个电机

| 功能 | MSPM0G3507 | TB6612 逻辑端 | 说明 |
|---|---|---|---|
| 电机 A PWM / J1 右轮 | PB2 / TIMA1 CCP0 | PWMA | 由 `PWM_0` 通道 0 输出 |
| 电机 A 方向 1 | PA14 | AIN1 | GPIO 输出 |
| 电机 A 方向 2 | PA13 | AIN2 | GPIO 输出 |
| 电机 B PWM / J2 左轮 | PB3 / TIMA1 CCP1 | PWMB | 由 `PWM_0` 通道 1 输出 |
| 电机 B 方向 1 | PA16 | BIN1 | GPIO 输出 |
| 电机 B 方向 2 | PA17 | BIN2 | GPIO 输出 |
| 电机 A 两端 / J1 | - | AO1、AO2 | 接右轮电机 |
| 电机 B 两端 / J2 | - | BO1、BO2 | 接左轮电机 |
| 逻辑电源 | S28A 5V | VCC | 以 D103A 模块丝印和手册为准 |
| 电机电源 | S28A 12V/VIN_SW | VM | 不要把电机电源接到 MSPM0 GPIO |
| 地 | S28A GND | GND | MSPM0、驱动、电池和外设共地 |

S28A 的 TB6612 接口 H12 输出控制信号，H9 提供 TB6612 的电源、地和电机端子信号。TB6612 模块的 `STBY` 若没有引出，应按 D103A 原理图的板载默认电平处理；若实际模块有独立 `STBY`，必须将其拉到有效高电平后才能驱动电机。

实际第一次上电时先将两个 PWM 设为 0，确认电机端子没有短路，再低占空比测试。若前进方向相反，交换该电机的 AO1/AO2，或在软件中交换该电机的两个方向位，不要修改编码器通道定义。

### 3.2 MPU6050

| MPU6050 | MSPM0G3507 | 说明 |
|---|---|---|
| SDA | PA0 / I2C0 SDA | I2C 控制器模式，100 kHz |
| SCL | PA1 / I2C0 SCL | 需要总线有效上拉；模块若无上拉需外加 |
| INT | PA7 | GPIO 输入，当前 SysConfig 下拉 |
| VCC | 模块允许的 3.3V 或 5V | 以 MPU6050 模块丝印/原理图为准 |
| GND | GND | 必须共地 |

AD0 接地时设备地址为 `0x68`，AD0 接高时为 `0x69`。头文件默认 `HW_MPU6050_I2C_ADDRESS` 为 `0x68`。

### 3.3 OLED

当前 SysConfig 将 OLED 配成软件串行接口，不是 MPU6050 使用的 I2C0：

| OLED 信号 | MSPM0G3507 | 用途 |
|---|---|---|
| SCL/D0 | PA28 | 软件时钟 |
| SDA/D1 | PA31 | 软件数据/MOSI |
| DC | PB15 | 数据/命令选择 |
| RST/RES | PB14 | 复位 |
| VCC | S28A OLED 接口的 3V3 | 先核对屏幕模块额定电压 |
| GND | GND | 必须共地 |

OLED 模块的接口命名和是否有板载 CS 可能不同。若实物是 7 针 SPI OLED 且 CS 没有被板载固定，必须再提供 CS；当前 S28A/C07A 资源表没有为 CS 预留独立信号，不能直接假定该模块可用。

### 3.4 蓝牙

| 蓝牙模块 | MSPM0G3507 |
|---|---|
| TXD | PB7，MSPM0 UART1_RX |
| RXD | PB6，MSPM0 UART1_TX |
| VCC | 按模块标注接 3.3V 或 5V |
| GND | GND |

串口参数为 9600 baud、8 数据位、无校验、1 停止位。蓝牙模块的 RX 电平必须能承受 C07A 的输出电平；裸 HC-05/HC-06 模块和带底板模块的供电/电平要求可能不同。

### 3.5 电池电压检测

S28A 原理图为 `12V -> 10 kOhm -> PA15 -> 1 kOhm -> GND`，PA15 同时是 `ADC1 channel 0`。因此：

```text
V_PA15 = V_BATTERY * 1k / (10k + 1k)
V_BATTERY = V_PA15 * 11
```

不要给 PA15 直接接 12V。使用 ADC 原始值计算电池电压时，还需按实际 VDDA、分压电阻误差和 ADC 参考值校准。

## 4. RYZD 六路红外接线

使用荣洋电子 RYZD 八路红外板。以传感器板正面从左到右编号为 CH1~CH8，本方案接中间六路 CH2、CH3、CH4、CH5、CH6、CH7；CH1、CH8 不接。整块 RYZD 板需要 VCC、GND，并且必须与 C07A 共地。

### 4.1 底板实际空闲接口

底板上可直接用于外部信号扩展的是 `J6` 和 `J10`，不是 H1/H2：

| 接口 | 引脚 | 信号 | 当前状态 |
|---|---:|---|---|
| J6 | 1 | PA22 | 可用于传感器 |
| J6 | 2 | PA24 | 可用于传感器 |
| J6 | 3 | PA27 | 可用于传感器 |
| J6 | 4 | PA9 | 可用于传感器 |
| J10 | 1 | PA12 | 可用于传感器 |
| J10 | 2 | PB16 | 可用于传感器 |
| J10 | 3 | PB17 | K230 UART2_TX，不能再接传感器 |

因此，K230 接入后 `J6 + J10` 正好提供 6 路传感器信号；本方案不接 RYZD 的 CH1、CH8。J10-3 留给 K230，不能再接红外。

### 4.2 RYZD 六路最终映射

以下表格中的 CH 编号是 RYZD 从左到右的物理位置；工程中的 `IR1~IR6` 是紧凑的软件读取序号：

| RYZD 物理位置 | RYZD 输出 | S28A 接口 | MSPM0 信号 | 工程读取 |
|---|---|---|---|---|
| CH1 | OUT1 | 不接 | - | 不配置 |
| CH2 | OUT2 | J6-1 | PA22 | `IR1` / `RYZD_CH2_DETECTED()` |
| CH3 | OUT3 | J6-2 | PA24 | `IR2` / `RYZD_CH3_DETECTED()` |
| CH4 | OUT4 | J6-3 | PA27 | `IR3` / `RYZD_CH4_DETECTED()` |
| CH5 | OUT5 | J6-4 | PA9 | `IR4` / `RYZD_CH5_DETECTED()` |
| CH6 | OUT6 | J10-1 | PA12 | `IR5` / `RYZD_CH6_DETECTED()` |
| CH7 | OUT7 | J10-2 | PB16 | `IR6` / `RYZD_CH7_DETECTED()` |
| CH8 | OUT8 | 不接 | - | 不配置 |

RYZD 电源接线：

```text
RYZD VCC  -> S28A 可用电源，按 RYZD 丝印/手册确认电压
RYZD GND  -> S28A GND
RYZD OUT2 -> S28A J6-1
RYZD OUT3 -> S28A J6-2
RYZD OUT4 -> S28A J6-3
RYZD OUT5 -> S28A J6-4
RYZD OUT6 -> S28A J10-1
RYZD OUT7 -> S28A J10-2
RYZD OUT1、OUT8 -> 悬空并绝缘
```

电气要求：

1. RYZD 的 VCC 电压以 RYZD 实物丝印或手册为准，不能根据本底板资料臆定。
2. RYZD OUT 必须是 MSPM0 可承受的 GPIO 电平；如果 RYZD 由 5V 供电且 OUT 为 5V 推挽输出，必须增加电平转换或分压，不能直接接 MSPM0。
3. 当前软件保留传感器原始电平：黑线时 LED 灭且输出高电平 `1`，白底时 LED 亮且输出低电平 `0`，不做取反。应用代码可使用 `HW_IR_RAW(1)` 到 `HW_IR_RAW(6)` 读取原始电平。
4. 红外线缆应远离电机线和 TB6612 输出线；若输入抖动，优先在应用层做采样滤波。

本分配占用了 S28A 原本可用于手柄、巡线、雷达、超声波和其他扩展的若干引脚。尤其 `PA24`、`PA9`、`PA12`、`PB16`、以及任何最终选用的额外 GPIO 不再可与对应可选功能并用；资料明确超声波与雷达、超声波与巡线之间也存在不能共用的资源约束。

## 5. 新增 K230D box

使用 MSPM0 的 UART2：

| K230D box | MSPM0/C07A | 物理位置 |
|---|---|---|
| TXD | PB18 / UART2_RX | C07A V1.1 J1-2 |
| RXD | PB17 / UART2_TX | S28A J10-3 |
| GND | GND | S28A 任意 GND |
| VCC | 按 K230D 实物标注接 5V 或指定电源 | 禁止接 12V |

TX/RX 必须交叉连接：K230D TXD 接 MSPM0 RX，K230D RXD 接 MSPM0 TX。当前 SysConfig 配置为 115200 baud、8N1、无硬件流控。

物理接线：

```text
K230D TXD -> C07A V1.1 J1-2 -> PB18 / MSPM0 UART2_RX
K230D RXD -> S28A J10-3 -> PB17 / MSPM0 UART2_TX
K230D GND -> S28A GND
K230D VCC -> 按 K230D 实物标注接指定电源，禁止接 12V
```

K230D 的具体盒型、供电电压、IO 电平和默认波特率未在本工程资料中给出。正式接线前必须查看盒体丝印或型号手册：

- 若 K230D UART 输出为 3.3V TTL，可直接连接。
- 若 K230D 输出为 5V TTL，PB18 需要电平转换；MSPM0 的 PB17 输出到 K230D RX 也要确认 K230D 是否接受 3.3V 高电平。
- 若 K230D 是 RS-232、RS-485 或 USB 接口，不能直接接 PB17/PB18，必须使用对应收发器。
- 若 K230D 没有独立 5V 输入，不能按本表直接供电；先确认其额定电源。

PB17/PB18 已为 K230 UART2 保留，不能再给红外或其他 UART 使用。S28A J10-3 接入 K230 后不能再接 RYZD 传感器。

## 6. 工程文件与生成配置

- `config/board.syscfg`：MSPM0G350X、LQFP-64(PM)，80 MHz 时钟，PWM、编码器、I2C、3 路 UART、ADC、OLED GPIO、按键、LED 和 RYZD 中间 CH2/3/4/5/6/7 六路红外。
- `generated/ti_msp_dl_config.h` / `generated/ti_msp_dl_config.c`：由 SysConfig 生成，不要手工改动；重新生成会覆盖它们。
- `board/board_hardware.h`：应用层稳定接口和兼容宏。

生成命令：

```powershell
& 'E:\ti\sysconfig-1.20.0_3587\sysconfig_cli.bat' `
  -o '.\projects\MSPM0G3507_empty_keil\generated' `
  -s 'E:\ti\mspm0_sdk_2_01_00_03\.metadata\product.json' `
  --compiler keil `
  '.\projects\MSPM0G3507_empty_keil\config\board.syscfg'
```

应用代码中包含：

```c
#include "board_hardware.h"
```

然后调用 `SYSCFG_DL_init()`，再使用 `HW_IR_RAW(1)` 到 `HW_IR_RAW(6)`、`HW_MOTOR_A_SET_DUTY(x)`、`HW_MPU6050_I2C`、`HW_K230_UART` 等接口。

## 7. 上电检查顺序

1. 不插电池，先用万用表检查 5V、3V3 与 GND 是否短路。
2. 单独确认 C07A 与 S28A 的 1 脚方向和 H1/H2 没有错位。
3. 暂不接电机，确认 UART、OLED、MPU6050 和红外的供电电压。
4. 读取 PA15 ADC，确认电池分压节点不会超过 ADC 输入范围。
5. 连接 K230D 前先确认它的接口类型、电压和波特率。
6. 最后接 TB6612 和电机，PWM 设为 0 后再逐步增加占空比。
