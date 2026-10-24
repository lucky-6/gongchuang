/**
  ******************************************************************************
  * @file    app_odom.h
  * @brief   两轴里程计：把 AS5600 的角度换算成前后 / 左右方向的位移 (mm)
  *
  *   X 轴 = 前后方向（AS5600 挂在 I2C1）
  *   Y 轴 = 左右方向（AS5600 挂在 I2C3）
  *
  *   每个方向配一个测量轮，位移 = 累计角度(rad) × 测量轮半径。
  *   半径、方向符号都在 App/app_odom.c 顶部按实车填。
  ******************************************************************************
  */
#ifndef APP_ODOM_H
#define APP_ODOM_H

#include <stdint.h>

#define ODOM_AXIS_NUM   2u    /* 0 = 前后(X)，1 = 左右(Y) */

/** @brief 初始化两个编码器并把里程清零 */
void  AppOdom_Init(void);

/** @brief 周期任务，主循环里反复调用（内部按固定周期采样） */
void  AppOdom_Update(void);

/** @brief 把当前位移清零 */
void  AppOdom_Reset(void);

/** @brief 取累计位移 (mm)，可为负 */
float AppOdom_GetX(void);     /* 前后方向：前进为正 */
float AppOdom_GetY(void);     /* 左右方向：右移为正 */

/* ---- 调试观察用 ---- */
extern volatile float    g_odom_x_mm;              /* 前后位移 (mm) */
extern volatile float    g_odom_y_mm;              /* 左右位移 (mm) */
extern volatile uint8_t  g_odom_online[ODOM_AXIS_NUM];  /* 1 = 该轴编码器在线 */
extern volatile uint8_t  g_odom_magnet[ODOM_AXIS_NUM];  /* 1 = 磁铁正常 */
extern volatile uint32_t g_odom_err[ODOM_AXIS_NUM];     /* 该轴累计读取失败次数 */

#endif /* APP_ODOM_H */
