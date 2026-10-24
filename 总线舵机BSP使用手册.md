# 总线舵机 BSP 使用手册

> 本文只讲**驱动层（BSP）的使用方法**：硬件接口、数据结构、初始化、各 API 函数、报文协议与示教存储。
> **不含** `Get_num*` / `Put_num*` 等应用层动作组。
>
> 相关文件：
> - `SERVOARM/SERVO_MOTOR.c`、`SERVOARM/SERVO_MOTOR.h` —— 舵机 BSP 本体
> - `Core/Src/can.c` —— CAN 收发（`CAN1_Send_Msg` / `CAN1_Receive_Msg`）
> - `SERVOARM/W25Qxx.c`、`SERVOARM/W25Qxx.h` —— 示教数据存储（SPI Flash）
> - `delay/delay.c` —— `delay_ms`

---

## 1. 概述

- 舵机是**串行总线舵机**（不是 PWM 舵机），控制总线为 **CAN1**。
- **CAN 的标识符（ID）就是舵机地址**，一帧报文直接寻址某个舵机。
- BSP 提供四类能力：**初始化 / 位置控制 / 位置回读 / 示教与回放**。
- 一帧位置指令 = `命令码 + 角度 + 速度`，位置为 **4096 分辨率（0–4095 对应 0–360°）**。

---

## 2. 硬件接口

| 项 | 内容 |
|---|---|
| 总线 | `CAN1` |
| 引脚 | `PA11 = CAN1_RX`，`PA12 = CAN1_TX` |
| 波特率 | **1 Mbps**（Prescaler 6，BS1 4TQ，BS2 2TQ） |
| 拓扑 | 总线式，多舵机挂同一条 CAN 线，靠 ID 区分 |
| 终端电阻 | 总线两端各 120Ω |
| 供电 | 舵机电源独立（按舵机额定电压/电流），**与主控共地** |

> 舵机是功率负载，务必单独供电；上电顺序和电源地去耦没做好，会直接影响 CAN 通信稳定性。

---

## 3. 数据结构

```c
struct Servo_Motor
{
    uint8_t position[3];   // 位置反馈（低字节/高字节，0-4096）
    int      ID;           // 舵机 ID（= CAN 标识符）
    uint8_t  Status;       // 状态（预留，未使用）
    int      Speed;        // 速度（预留，未使用）
    uint8_t  Mode;         // 模式（预留，未使用）
};
```

BSP 预置了 6 个实例：`Servo1`–`Servo6`（定义在 `SERVO_MOTOR.c`）。

---

## 4. 初始化

### 4.1 `Init_Servo_Motor()` —— 设置 ID 并开启角度回传

```c
void Init_Servo_Motor(void)
{
    Servo1.ID = 0xA0;
    Servo2.ID = 0xA1;
    Servo3.ID = 0xA2;
    Servo4.ID = 0xA3;
    Servo5.ID = 0x09;
    Servo6.ID = 0x0A;

    servo_signal[0] = 0x19;   // 命令码：开启角度回传
    servo_signal[1] = 0x1d;
    servo_signal[2] = 0;
    CAN1_Send_Msg(Servo1.ID, servo_signal, 3);
    delay_ms(5);
    /* Servo2 / Servo3 / Servo4 同上，各 delay_ms(5) */
}
```

- 必须**先调用**它，之后位置回读才有数据。
- 只对 `Servo1–Servo4` 发送了回传使能命令。

### 4.2 `Reset_ServoMotor_Angle()` —— 设原点（可选项）

```c
void Reset_ServoMotor_Angle(void)
{
    servo_signal[0] = 0x01;                     // 触发一次位置回传
    CAN1_Send_Msg(Servo1.ID, servo_signal, 8);
    delay_ms(50);
    /* Servo2/3/4 同上 */

    Servo1.position[0] = 0x0B;                  // 命令码：设置当前位置为原点
    CAN1_Send_Msg(Servo1.ID, Servo1.position, 3);
    /* Servo2/3/4 同上 */
}
```

> ⚠️ 原代码注释标注该函数**"搁置，编码器绝对位置有偏差"**，可靠性存疑，谨慎使用。

---

## 5. 核心 API

### 5.1 `Servo_Control(ID, angle, speed)` —— 位置控制

```c
void Servo_Control(uint8_t ID, int angle, uint8_t speed)
{
    int i = 0x00000000 + ID;
    servo_signal[0] = 0x03;                     // 命令码：位置控制
    servo_signal[1] = (angle * 4096 / 360) % 256;  // 角度低字节
    servo_signal[2] = (angle * 4096 / 360) / 256;  // 角度高字节
    servo_signal[3] = 0x00 + speed;             // 速度
    CAN1_Send_Msg(i, servo_signal, 8);
    delay_ms(2);
    CAN1_Send_Msg(i, servo_signal, 8);          // 重复发送，提高可靠性
}
```

| 参数 | 说明 |
|---|---|
| `ID` | 舵机 ID / CAN 标识符（如 `0xA0`、`16`） |
| `angle` | 目标角度（度），0 为原点，**按 4096/360 换算成原始值** |
| `speed` | 速度（`0x00–0xFF`），越大越快 |

要点：

- 角度是**小端**放在第 1、2 字节。
- **同一帧连发两次**（间隔 2ms）是可靠性措施，别只发一次。
- `angle` 为 `int`，**传负角度会得到不可预期的结果**，使用非负角度。

### 5.2 `Servo_position(canbufRX)` —— 位置回读

放在 **CAN 接收处/中断里**调用，把回传位置按 ID 分发到对应舵机：

```c
void Servo_position(uint8_t canbufRX[])
{
    if (get_STID == Servo1.ID) {          // 用 CAN 标准 ID 判断是哪个舵机
        Servo1.position[1] = canbufRX[1]; // 位置低字节
        Servo1.position[2] = canbufRX[2]; // 位置高字节
    }
    /* Servo2 … Servo6 同上 */
    get_STID = 0x0000;                    // 用完清标志
}
```

典型调用（`TIM1` 中断，1kHz）：

```c
CAN1_Receive_Msg(canbufRX);
...
Servo_position(canbufRX);
```

- 反馈角度同样 `[1]` 低、`[2]` 高。
- 依赖全局 `get_STID`（在 `can.c` 的 `CAN1_Receive_Msg` 里赋值）。
- 中断里持续接收会占用 MCU 资源，原注释提示"调试完成后要注释"。

### 5.3 `Servo_Remember(team_num)` —— 示教记录

手动摆好舵机姿势，把位置序列存进 Flash，动作组编号 `team_num` 从 0 开始。

```c
void Servo_Remember(uint8_t team_num)
{
    servo_signal[0] = 0x2f;                        // 命令码：进入示教记录模式
    CAN1_Send_Msg(i_ID, servo_signal, 8);

    for (i = 0; i < 16; i++)                       // 擦除该动作组占用的 4KB
        BSP_W25Qx_Erase_Block(team_num * 0x1000 + i * 0x100);

    while (HAL_GPIO_ReadPin(GPIOC, Teach_Pin))     // 等按键（PC5）按下开始
        delay_ms(5);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    delay_ms(1000);

    for (lk = 0; lk < 16; lk++) {
        for (i = 0; i < 16; i++) {
            W25Q64.wData[1 + i*16] = Servo1.position[1];   // 每 20ms 采一组
            W25Q64.wData[2 + i*16] = Servo1.position[2];
            /* Servo2/3/4 同理写入 byte3..byte8 */
            delay_ms(20);
            if (!HAL_GPIO_ReadPin(GPIOC, Teach_Pin)) { /* 松手停止 */ break; }
        }
        W25Q64.wData[0] = i;                       // 本页有效组数
        BSP_W25Qx_Write(W25Q64.wData, team_num*0x1000 + 0x100*lk, 0x100);
        if (!HAL_GPIO_ReadPin(GPIOC, Teach_Pin)) break;
    }
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}
```

| 项 | 说明 |
|---|---|
| 触发 | `Teach_Pin`（`PC5`）按下开始、松开结束；`LED3`（`PE7`）指示 |
| 采样 | 每 **20ms** 采一组 4 个舵机的位置 |
| 采样源 | `ServoX.position[1]/[2]`（必须先开回传并有 `Servo_position` 在跑） |

### 5.4 `Servo_team(team_num, s_speed)` —— 回放动作组

```c
void Servo_team(uint8_t team_num, uint8_t s_speed)
{
    for (lk = 0; lk < 16; lk++) {
        for (i = 0; i < 16; i++) {
            BSP_W25Qx_Read(W25Q64.rData, team_num*0x1000 + lk*0x100, 0x100);
            servo_signal[0] = 0x03;               // 位置控制
            servo_signal[3] = s_speed;            // 统一回放速度
            servo_signal[1] = W25Q64.rData[1 + i*16];
            servo_signal[2] = W25Q64.rData[2 + i*16];
            CAN1_Send_Msg(Servo1.ID, servo_signal, 4);
            delay_ms(5);
            CAN1_Send_Msg(Servo1.ID, servo_signal, 4);
            /* Servo2/3/4 同理，各发两次 */
            if (W25Q64.rData[0] != 16) break;      // 本页未满 = 最后一组，结束
        }
    }
}
```

| 参数 | 说明 |
|---|---|
| `team_num` | 动作组编号（对应 `Servo_Remember` 存入的组） |
| `s_speed` | 回放速度（`0x00–0xFF`） |

要点：

- 逐页读取 Flash，逐组下发位置指令，每个舵机**重复发两次**。
- `rData[0] != 16` 表示这一页没记满，说明是最后一个姿态，随即停止。
- 靠 `delay_ms(5)` 节拍推进，不做到位判断。

---

## 6. 报文协议汇总

### 6.1 命令码

| 命令码 | 作用 | 数据格式 | 发送长度 | 使用函数 |
|---|---|---|---|---|
| `0x03` | 位置控制 | `03 角度L 角度H 速度` | 8（回放时 4） | `Servo_Control` / `Servo_team` |
| `0x01` | 触发位置回传 | `01 ...` | 8 | `Reset_ServoMotor_Angle` |
| `0x0B` | 设置当前位置为原点 | `0B ...` | 3 | `Reset_ServoMotor_Angle` |
| `0x19` | 开启角度回传 | `19 1D 00` | 3 | `Init_Servo_Motor` |
| `0x2F` | 进入示教记录模式 | `2F ...` | 8 | `Servo_Remember` |

### 6.2 角度编码

| 项 | 值 |
|---|---|
| 分辨率 | 4096 步 / 360°（≈ 11.38 步/度） |
| 换算 | `raw = angle * 4096 / 360` |
| 字节序 | 小端（`[1]`=低，`[2]`=高） |

### 6.3 CAN 帧类型规则

`can.c` 的 `CAN1_Send_Msg` 按 ID 大小选择帧类型：

```c
if (id > 0x0A) TxHeader.IDE = CAN_ID_EXT;   // 扩展帧
else           TxHeader.IDE = CAN_ID_STD;   // 标准帧
```

因此 `0xA0–0xA3`、`0x10` 用扩展帧；`0x09`、`0x0A` 用标准帧。

---

## 7. Flash 存储布局（示教数据）

| 项 | 值 |
|---|---|
| 介质 | `W25Qxx`（SPI1，CS = `PB0`） |
| 每个动作组 | 4 KB（起始 `team_num * 0x1000`） |
| 每页 | 256 字节（`0x100`） |
| 每页组数 | 16 组（每组 16 字节） |
| 每组内容 | `byte0`=本页有效组数；`byte1/2`=Servo1 低/高；`byte3/4`=Servo2；`byte5/6`=Servo3；`byte7/8`=Servo4；其余预留 |
| 每组容量 | 16 页 × 16 组 = **256 个姿态** |

---

## 8. 最小使用示例（BSP 级）

```c
/* 1. 初始化：设定 ID、开启角度回传 */
Init_Servo_Motor();

/* 2. 位置控制：让 0xA0 号舵机以速度 0x80 转到 90° */
Servo_Control(0xA0, 90, 0x80);

/* 3. 位置回读：在 CAN 接收/中断里调用 */
CAN1_Receive_Msg(canbufRX);
Servo_position(canbufRX);          // 成功后 Servo1.position[1]/[2] 更新

/* 4. 示教：手动摆姿势并存入 3 号动作组 */
Servo_Remember(3);

/* 5. 回放 3 号动作组，速度 0x80 */
Servo_team(3, 0x80);
```

---

## 9. 注意事项（踩坑清单）

1. **必须先 `Init_Servo_Motor()`** 开回传，`Servo_position` 才有数据，`Servo_Remember` 才有采样源。
2. **位置指令要重复发两次**，单次在总线上容易丢动作。
3. **`angle` 传非负值**，负角度会导致换算结果异常。
4. **示教依赖中断里的 `Servo_position`**：位置回读一旦关掉，示教存进去的就是旧值。
5. **`Servo_Remember` 的示教命令发给 `i_ID`（= 0x00）**，`0x00` 不是舵机 ID，这条 `0x2F` 很可能无效，建议核对。
6. **回读匹配用的是 `get_STID`（标准 ID）**，而 `0xA0–0xA3` 是按扩展帧发送的；若舵机也用扩展帧回传则匹配不上，需实测确认。
7. **`Servo6` 的位置存法与其它不一致**（存 `position[0]/[1]` 而非 `[1]/[2]`），是原有错位问题。
8. **`Reset_ServoMotor_Angle` 不可靠**（原注释"搁置，编码器绝对位置有偏差"）。
9. **中断里持续接收舵机回传会加重 MCU 负担**，正式运行可按需关闭 `Servo_position`。
10. **舵机独立供电并共地**，总线两端加 120Ω，否则通信会不稳。
