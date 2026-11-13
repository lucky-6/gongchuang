/**
  ******************************************************************************
  * @file    app_odom.c
  * @brief   两轴里程计实现
  ******************************************************************************
  */
#include "app_odom.h"
#include "main.h"
#include "as5600.h"

/* ===== 按实车填写 ===== */

/* 采样周期 (ms)：10 ms = 100 Hz。
   注意 AS5600 靠软件累计圈数，两次采样之间转动不能超过半圈；
   100 Hz 下允许 50 圈/秒，测量轮远达不到，安全。 */
#define ODOM_UPDATE_PERIOD_MS   10u

/* 测量轮半径 (mm)：X = 前后方向测量轮，Y = 左右方向测量轮 */
#define ODOM_X_WHEEL_R_MM       20.0f
#define ODOM_Y_WHEEL_R_MM       20.0f

/* 方向符号：测量轮装在轴上的转向与"前进/右移"相反时改成 -1.0f */
#define ODOM_X_SIGN             (+1.0f)
#define ODOM_Y_SIGN             (+1.0f)

volatile float    g_odom_x_mm  = 0.0f;
volatile float    g_odom_y_mm  = 0.0f;
volatile uint8_t  g_odom_online[ODOM_AXIS_NUM] = {0};
volatile uint8_t  g_odom_magnet[ODOM_AXIS_NUM] = {0};
volatile uint32_t g_odom_err[ODOM_AXIS_NUM]    = {0};

void AppOdom_Init(void)
{
  AS5600_Init();
  AppOdom_Reset();
}

void AppOdom_Update(void)
{
  static uint32_t last_ms = 0u;
  const AS5600_t *ex;
  const AS5600_t *ey;

  if ((HAL_GetTick() - last_ms) < ODOM_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_ms = HAL_GetTick();

  AS5600_Update();

  ex = AS5600_Get(AS5600_AXIS_X);
  ey = AS5600_Get(AS5600_AXIS_Y);

  if (ex != NULL)
  {
    /* 位移 = 累计角度(rad) × 半径，结果就是弧长 (mm) */
    g_odom_x_mm         = ODOM_X_SIGN * ex->accum_rad * ODOM_X_WHEEL_R_MM;
    g_odom_online[0]    = ex->online;
    g_odom_magnet[0]    = ex->magnet;
    g_odom_err[0]       = ex->err_cnt;
  }
  if (ey != NULL)
  {
    g_odom_y_mm         = ODOM_Y_SIGN * ey->accum_rad * ODOM_Y_WHEEL_R_MM;
    g_odom_online[1]    = ey->online;
    g_odom_magnet[1]    = ey->magnet;
    g_odom_err[1]       = ey->err_cnt;
  }
}

void AppOdom_Reset(void)
{
  AS5600_ZeroAll();
  g_odom_x_mm = 0.0f;
  g_odom_y_mm = 0.0f;
}

float AppOdom_GetX(void)
{
  return g_odom_x_mm;
}

float AppOdom_GetY(void)
{
  return g_odom_y_mm;
}
