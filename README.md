# gongchuang

基于 **STM32F407** 的麦轮全向底盘 + 总线舵机机械臂控制固件，使用 **STM32CubeMX + CMake + GCC ARM 工具链** 构建。

## 功能概览

- **四轮麦轮全向底盘**：车体速度 `(vx, vy, ω)` → 四轮运动学换算 → 安装符号补偿
- **4 台 MKS SERVO42&57D 闭环步进驱动**：RS485 半双工总线，速度模式控制
- **航向闭环**：HWT101CT 单轴陀螺仪（Z 轴偏航角）保持 / 转到目标朝向
- **两轴里程计**：两路 AS5600 磁编码器测量 X/Y 位移，支持回到上电原点
- **总线舵机机械臂**：CAN1 总线，4 轴机械臂 + 夹爪位置控制与周期回读
- **原子动作接口**：`Act_*` 只负责发起动作和查询结果，不含状态机；流程与异常处理由用户自己的 `Plan_Task()` 决定

## 外设与引脚

| 功能 | 外设 | 引脚 | 说明 |
|---|---|---|---|
| MKS 步进驱动 ×4 | USART2 | PA4 = DE | RS485 半双工，`0xFA/0xFB` 帧协议 |
| HWT101CT 陀螺仪 | USART1 | PA9 = TX, PA10 = RX | DMA2_Stream2 循环接收，100 Hz |
| 总线舵机 | CAN1 | PA11 = RX, PA12 = TX | 1 Mbps，舵机地址即 CAN 标识符 |
| AS5600（X 轴） | I2C1 | PB6 = SCL, PB7 = SDA | 地址 0x36 |
| AS5600（Y 轴） | I2C3 | PA8 = SCL, PC9 = SDA | 地址 0x36 |
| 调试串口 | UART5 | PC12 = TX, PD2 = RX | 115200 8N1，支持 `%f` 格式化 |

## 目录结构

```
App/        应用层：底盘、里程计、位姿、IMU、动作接口、用户规划
BSP/        板级驱动：RS485、调试串口
Motor/      MKS SERVO42&57D RS485 协议（组帧/校验/单位换算）
Encoder/    AS5600 磁编码器驱动
IMU/        HWT101CT 陀螺仪驱动
Servo/      CAN 总线舵机驱动
Core/       CubeMX 生成的外设初始化与主循环
cmake/      工具链与 CubeMX 生成的 CMake 配置
```

## 主循环

`Core/Src/main.c` 每拍依次执行：

```c
AppIMU_Update();    /* 陀螺仪采样 */
AppOdom_Update();   /* 两轴测量轮采样 */
Pose_Update();      /* 里程计 + 陀螺融合 → 世界坐标 */
Chassis_Task();     /* 速度指令 → 4 帧 RS485（必须每拍调用） */
ServoBus_Poll();    /* 舵机周期回传 → 更新关节角缓存 */
Act_Task();         /* 推进正在执行的原子动作 */
Plan_Task();        /* ★ 用户规划逻辑 */
```

> **注意**：MKS 驱动板工作在速度模式，主机不发指令时会保持上一次速度。
> 停车必须显式调用 `Act_Stop()` / `Chassis_Stop()`，不存在自动停车。

## 构建

依赖：

- CMake ≥ 3.22
- Ninja
- `arm-none-eabi-gcc`（GCC ARM 工具链）

```bash
cmake --preset Debug
cmake --build --preset Debug
```

生成固件：`build/Debug/gongchuang.elf`

Release 构建把 preset 换成 `Release` 即可。

## 移植 / 修改

- `gongchuang.ioc` 是 CubeMX 工程文件，外设改动请在其中修改后重新生成代码。
- 底盘轮径、轮距、减速比、安装符号等参数在 `App/app_chassis.c` 与 `App/app_odom.c` 中。
- 舵机 ID 与角度标定在 `Servo/servo_bus.h` 中。

## 文档

| 文档 | 内容 |
|---|---|
| [动作接口说明.md](动作接口说明.md) | 原子动作接口、单位与坐标系约定 |
| [底盘控制说明.md](底盘控制说明.md) | 运动学、航向闭环、返航逻辑 |
| [AS5600里程计说明.md](AS5600里程计说明.md) | 编码器安装、方向、距离换算 |
| [HWT101使用手册.md](HWT101使用手册.md) | 陀螺仪通信协议与配置 |
| [HWT101驱动说明.md](HWT101驱动说明.md) | 本工程驱动实现细节 |
| [总线舵机驱动说明.md](总线舵机驱动说明.md) | CAN 总线舵机协议与关节控制 |
| [总线舵机BSP使用手册.md](总线舵机BSP使用手册.md) | 舵机 BSP 接口 |
| [485步进电机控制板开发手册.md](485步进电机控制板开发手册.md) | MKS 驱动板接线与调试 |
| [485四轴步进电机驱动程序任务书.md](485四轴步进电机驱动程序任务书.md) | 四轴驱动需求说明 |
| [麦轮全向底盘应用接口任务书.md](麦轮全向底盘应用接口任务书.md) | 底盘应用接口需求 |
| [Schematic.pdf](Schematic.pdf) | 硬件原理图 |

> 厂商提供的《MKS SERVO42&57D_RS485 闭环步进电机 使用说明 V1.0.2》PDF 因版权原因未包含在本仓库中，请从 MKS 官方渠道获取。

## 许可证

暂未指定开源许可证。
