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

#endif /* SERVO_BUS_H */
