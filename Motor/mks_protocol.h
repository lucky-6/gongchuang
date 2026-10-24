/**
  ******************************************************************************
  * @file    mks_protocol.h
  * @brief   MKS SERVO42D/57D RS485 串口协议：组帧 / 校验 / 单位换算
  *          依据《MKS SERVO42&57D_RS485 闭环步进电机 使用说明 V1.0.2》
  ******************************************************************************
  */
#ifndef MKS_PROTOCOL_H
#define MKS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* 帧头 */
#define MKS_HEAD_DOWN        0xFAu   /* 下行：主机 -> 驱动板 */
#define MKS_HEAD_UP          0xFBu   /* 上行：驱动板 -> 主机 */

/* 功能码 */
#define MKS_FC_SPEED         0xF6u   /* 速度模式运行/停止，下行 7 字节 */
#define MKS_FC_MOVE_PULSE    0xFDu   /* 位置模式1：按脉冲数相对运动，下行 11 字节 */
#define MKS_FC_ENABLE        0xF3u   /* 使能/失能，下行 5 字节 */
#define MKS_FC_QUERY_STATE   0xF1u   /* 查询运行状态，下行 4 字节 */
#define MKS_FC_READ_AXIS     0x31u   /* 读坐标（累加制多圈编码器值），下行 4 字节 */

/* 驱动板速度以 16 细分为基准标定：实际RPM = 速度值 × 16 / 当前细分 */
#define MKS_SPEED_BASE_MSTEP 16u
#define MKS_SPEED_MAX        3000u

/* 1.8° 两相步进电机：整步 200 步/圈 */
#define MKS_FULL_STEP_PER_REV 200u

/** @brief 8 位累加和校验：sum(前 len 字节) & 0xFF */
uint8_t  MKS_Checksum(const uint8_t *buf, uint16_t len);

/** @brief 实际转速(RPM) -> 协议速度值：value = rpm × 细分 / 16（四舍五入，上限 3000） */
uint16_t MKS_RpmToSpeedValue(uint32_t rpm_actual, uint16_t mstep);

/** @brief 协议速度值 -> 实际转速(RPM)：rpm = value × 16 / 细分，用于核对实际转速 */
uint32_t MKS_SpeedValueToRpm(uint16_t speed_value, uint16_t mstep);

/** @brief 圈数 -> 脉冲数：圈数 × 200 × 细分 */
uint32_t MKS_RevsToPulses(uint16_t revs, uint16_t mstep);

/** @brief 组速度模式帧（rpm_actual < 0 表示反转），返回帧长 7 */
uint16_t MKS_BuildSpeed(uint8_t addr, int32_t rpm_actual, uint16_t mstep,
                        uint8_t acc, uint8_t *out);

/** @brief 组位置模式1帧（pulses < 0 表示反转），返回帧长 11 */
uint16_t MKS_BuildMovePulses(uint8_t addr, int32_t pulses, int32_t rpm_actual,
                             uint16_t mstep, uint8_t acc, uint8_t *out);

/** @brief 组停止帧（mode 传 MKS_FC_SPEED 或 MKS_FC_MOVE_PULSE），返回帧长 7/11 */
uint16_t MKS_BuildStop(uint8_t addr, uint8_t mode, uint8_t acc, uint8_t *out);

/** @brief 组使能/失能帧，返回帧长 5 */
uint16_t MKS_BuildEnable(uint8_t addr, bool enable, uint8_t *out);

/** @brief 组查询帧（码值见 MKS_FC_*），返回帧长 4 */
uint16_t MKS_BuildQuery(uint8_t addr, uint8_t code, uint8_t *out);

#endif /* MKS_PROTOCOL_H */
