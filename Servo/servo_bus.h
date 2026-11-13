/**
  ******************************************************************************
  * @file    servo_bus.h
  * @brief   总线舵机（CAN1）最简驱动：初始化 + 位置控制 + 原始发送
  *
  *   总线：CAN1，PA11 = RX，PA12 = TX，1 Mbps（SYSCLK 168MHz / APB1 42MHz）
  *   舵机地址 = CAN 标识符；位置指令 = 命令码 + 角度 + 速度
  *   角度分辨率：4096 / 360°（0~4095 对应 0~360°）
  ******************************************************************************
  */
#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>
#include <stdbool.h>

/* 舵机地址（= CAN 标识符），已扫描确认在线 */
#define SERVO_ID_ARM0    0xA0u
#define SERVO_ID_ARM1    0xA1u
#define SERVO_ID_ARM2    0xA2u
#define SERVO_ID_ARM3    0xA3u
#define SERVO_ID_GRIPPER 0x10u

/* 角度分辨率 */
#define SERVO_RAW_MAX         4095u

/* 命令码（请求 -> 应答） */
#define SERVO_CMD_POSITION    0x03u   /* 转到指定位置 */
#define SERVO_CMD_ENABLE_RSP  0x19u   /* 开启周期位置报告 */
#define SERVO_CMD_STOP_FREE   0x2Fu   /* 停止并泄力（示教时释放力矩） */
#define SERVO_RSP_POSITION    0x02u   /* 查询位置应答 */
#define SERVO_RSP_REPORT      0x1Au   /* 周期报告数据 */

/** @brief 初始化：配 CAN 全通滤波器并启动 CAN（需在 MX_CAN1_Init() 之后调用） */
void ServoBus_Init(void);

/** @brief 让指定 ID 的舵机以 speed(0~255) 转到 angle_deg 度（非负） */
bool ServoBus_SetAngle(uint32_t id, float angle_deg, uint8_t speed);

/** @brief 发原始 CAN 帧（ID>0x0A 用扩展帧），供上层发 0x19/0x2F 等指令 */
bool ServoBus_SendRaw(uint32_t id, const uint8_t *data, uint8_t len);

/* ==================== 位置回读 ==================== */

/** @brief 周期回传周期 (ms)。29ms 是参考工程用的值 */
#define SERVO_REPORT_PERIOD_MS   29u

/**
  * @brief  让指定舵机开启周期位置回传（发 0x19 指令）
  * @param  period_ms 回传周期，单位 ms
  * @note   ServoBus_Init() 已经对 5 个已知地址调用过，一般不用自己再调。
  */
bool ServoBus_EnableReport(uint32_t id, uint8_t period_ms);

/**
  * @brief  把 CAN FIFO 里的回传帧取出来，更新各舵机的当前位置
  * @note   ★必须在主循环周期调用（建议每圈都调，开销很小）。
  *         不调用的话 FIFO 会被回传帧灌满，位置也不会更新。
  */
void ServoBus_Poll(void);

/** @brief 取最近一次收到的原始位置 (0~4095)。false = 该舵机还没回过帧 */
bool ServoBus_GetRaw(uint32_t id, uint16_t *raw);

/** @brief 取最近一次收到的角度 (°) = raw × 360 / 4096。false = 还没收到 */
bool ServoBus_GetAngle(uint32_t id, float *deg);

/** @brief 累计收到的回传帧数（诊断用：一直是 0 说明回传没开或接线不对） */
uint32_t ServoBus_GetRxCount(void);

#endif /* SERVO_BUS_H */
