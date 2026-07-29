# H题 - 车载平衡滚球运动控制系统

## 项目结构

```
Hproject/
├── board/
│   └── board_hardware.h           # 硬件引脚宏定义
├── drivers/
│   ├── motor_driver.c/h           # TB6612 电机驱动
│   ├── encoder_driver.c/h         # 正交编码器
│   ├── line_sensor.c/h            # 6路IR巡线传感器
│   ├── oled_driver.c/h            # OLED显示
│   ├── stepper_driver.c/h         # 步进电机驱动（摆杆控制）
│   ├── camera_uart.c/h            # K230D摄像头通信
│   ├── button_input.c/h           # 按键输入（单击/双击）
│   └── debug_uart.c/h             # 调试串口
├── control/
│   ├── control_config.h           # PID参数集中配置
│   ├── vehicle_controller.c/h     # 速度PI + 巡线PD
│   └── balance_controller.c/h     # 钢球位置PID → 步进电机
├── app/
│   ├── app_config.h               # 应用参数（速度/赛道/停车）
│   ├── app_main.c                 # 主入口
│   ├── competition_mode.c/h       # 比赛模式状态机
│   ├── line_follow_task.c/h       # 巡线任务
│   └── balance_task.c/h           # 平衡任务
└── ti_msp_dl_config.c/h           # SysConfig生成（需从工程导入）
```

## 操作说明

- **双击** BLS/RESET 按键：切换比赛模式 (Q2→Q3→Q4→Q5→Q6)
- **单击** BLS/RESET 按键：确认/启动
- 运行中单击：紧急停车

## 比赛模式

| 模式 | 描述 | 时间要求 |
|------|------|---------|
| Q2 | 纯巡线一圈 | ≤20s |
| Q3 | 静止控球 O→+5→-5 | ≤5s |
| Q4 | 巡线A→B + 球稳中心 | ≤8s |
| Q5 | 巡线一圈 + 球稳中心 | ≤30s |
| Q6 | 巡线一圈 + 球稳指定位置 | ≤30s |

## 硬件平台

- MCU: TI MSPM0G3507 (80MHz Cortex-M0+)
- 核心板: C07A V1.1
- 底板: S28A
- 电机驱动: TB6612FNG (D103A模块)
- IMU: MPU6050 (I2C)
- 摄像头: K230D Box (正点原子, UART)
- 巡线: 6路IR (RYZD)
- 显示: 0.96" OLED (SSD1306)
- 平衡: 步进电机 + 驱动板

## TODO

- [ ] 从 SysConfig 导入 ti_msp_dl_config 文件
- [ ] 确认步进电机引脚分配
- [ ] 对接 K230D 实际通信协议
- [ ] MPU6050 加速度前馈补偿
- [ ] 调试巡线PID参数
- [ ] 调试平衡PID参数
