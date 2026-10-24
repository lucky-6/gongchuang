/**
  ******************************************************************************
  * @file    as5600.h
  * @brief   AS5600 12 位磁编码器驱动（里程计用）
  *
  *   两个传感器，I2C 地址都是 0x36，分挂在不同总线上：
  *     X 轴（前后方向）→ I2C1，PB6 = SCL，PB7 = SDA
  *     Y 轴（左右方向）→ I2C3，PA8 = SCL，PC9 = SDA
  *
  *   驱动只输出角度：单圈角度 + 多圈累计角度（rad，可正可负）。
  *   换算成距离由应用层做（需要测量轮半径），见 App/app_odom.c。
  *
  *   注意：AS5600 单圈 12 位，读一次只知道当前角度，圈数靠软件累计，
  *         所以必须周期性调用 AS5600_Update()，且两次调用之间的转动不能超过半圈。
  ******************************************************************************
  */
#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* I2C 从机地址：7 位 0x36，HAL 需要 8 位形式（左移一位） */
#define AS5600_ADDR_7BIT     0x36u
#define AS5600_ADDR          (AS5600_ADDR_7BIT << 1)

#define AS5600_CNT_PER_REV   4096u        /* 12 位：一圈 4096 个计数 */
#define AS5600_TWO_PI        6.2831853f

/* 轴编号 */
#define AS5600_AXIS_X        0u   /* 前后方向（I2C1） */
#define AS5600_AXIS_Y        1u   /* 左右方向（I2C3） */
#define AS5600_AXIS_NUM      2u

/* 寄存器 */
#define AS5600_REG_STATUS    0x0Bu   /* bit5 = MD 磁铁检测 */
#define AS5600_REG_RAW_HI    0x0Cu
#define AS5600_REG_RAW_LO    0x0Du
#define AS5600_REG_ANG_HI    0x0Eu
#define AS5600_REG_ANG_LO    0x0Fu
#define AS5600_REG_BLOCK     0x0Bu   /* 连续读 5 字节：STATUS/RAW2/ANG2 */
#define AS5600_BLOCK_LEN     5u

#define AS5600_I2C_TIMEOUT   10u     /* 单次读写超时 (ms) */
#define AS5600_FAIL_LIMIT    3u      /* 连续失败多少次判离线 */

/** @brief 单个编码器状态 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;      /* 所属 I2C 总线 */
  uint16_t raw;                 /* 原始角度（未滤波）0~4095 */
  uint16_t raw_filt;            /* 滤波后角度 0~4095 */
  uint16_t last_raw;            /* 上一次用于累计的值 */
  int32_t  accum_cnt;           /* 累计计数（4096 = 一圈） */
  float    accum_rad;           /* 累计角度 (rad)，可正可负、可超 2π */
  uint8_t  magnet;              /* 1 = 磁铁正常 */
  uint8_t  online;              /* 1 = 最近一次读取成功 */
  uint8_t  fail_cnt;            /* 连续失败计数 */
  uint32_t err_cnt;             /* 累计失败次数 */
} AS5600_t;

/** @brief 初始化两个编码器（依赖 MX_I2C1_Init / MX_I2C3_Init 已调用） */
void  AS5600_Init(void);

/** @brief 周期读取两个编码器并累计角度；调用间隔内转动不要超过半圈 */
void  AS5600_Update(void);

/** @brief 只更新其中一个轴（AS5600_AXIS_X / AS5600_AXIS_Y） */
void  AS5600_UpdateAxis(uint8_t axis);

/** @brief 取状态（只读），axis 越界返回 NULL */
const AS5600_t *AS5600_Get(uint8_t axis);

/** @brief 取累计角度 (rad) */
float AS5600_GetAccumRad(uint8_t axis);

/** @brief 把当前角度清零（累计量归零，单圈零点也重置到当前位置） */
void  AS5600_Zero(uint8_t axis);
void  AS5600_ZeroAll(void);

/* ---- 底层接口（诊断用） ---- */

/** @brief 读 12 位原始角度（0x0C/0x0D，一次事务读 2 字节） */
bool  AS5600_ReadRaw(I2C_HandleTypeDef *hi2c, uint16_t *raw);

/** @brief 读状态寄存器并判断磁铁是否正常 */
bool  AS5600_IsMagnetOk(I2C_HandleTypeDef *hi2c);

/** @brief 一次事务读出 STATUS/RAW/ANG 共 5 字节，buf 布局：STATUS,RAWH,RAWL,ANGH,ANGL */
bool  AS5600_ReadBlock(I2C_HandleTypeDef *hi2c, uint8_t *buf);

#endif /* AS5600_H */
