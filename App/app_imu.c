/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   陀螺仪接入：上电配置模块 + 主循环轮询取数
  ******************************************************************************
  */
#include "app_imu.h"
#include "main.h"
#include "hwt101.h"
#include "bsp_debug.h"

/* 上电是否给模块下发配置（Z 角清零 + 100Hz + 115200）。
   模块若还是出厂 9600，先按《HWT101驱动说明.md》第七节把波特率改过来，
   再把这里置 0，避免用错误的波特率发配置帧。 */
#define APP_IMU_CONFIG_ON_INIT   1

/* 配置用参数：0x09 = 100Hz，0x06 = 115200 */
#define APP_IMU_RATE_CODE        0x09u
#define APP_IMU_BAUD_CODE        0x06u

/* 从 UART5 打印陀螺仪数据的周期 (ms)；置 0 关闭（当前只打印编码器，故关闭） */
#define APP_IMU_PRINT_PERIOD_MS  0u

void AppIMU_Init(void)
{
  HWT101_Init();                     /* 先把接收挂起来，保证不丢帧 */

#if APP_IMU_CONFIG_ON_INIT
  HWT101_Config(APP_IMU_RATE_CODE, APP_IMU_BAUD_CODE);
  HAL_Delay(2000);                   /* 等模块按新参数稳定输出（手册流程） */
#endif
}

void AppIMU_Update(void)
{
#if (APP_IMU_PRINT_PERIOD_MS > 0u)
  static uint32_t s_last_print = 0u;
#endif

  HWT101_Poll();                     /* 消费 DMA 里的新数据 */

#if (APP_IMU_PRINT_PERIOD_MS > 0u)
  if ((HAL_GetTick() - s_last_print) >= APP_IMU_PRINT_PERIOD_MS)
  {
    HWT101_Data_t d;

    s_last_print = HAL_GetTick();
    (void)HWT101_GetData(&d);

    Debug_Printf("[IMU] yaw=%7.2f gz=%7.2f ver=0x%04X on=%u f=%lu e=%lu\r\n",
                 (double)d.yaw, (double)d.gz, d.version,
                 (unsigned)(HWT101_IsOnline() ? 1u : 0u),
                 (unsigned long)d.frame_cnt,
                 (unsigned long)HWT101_GetErrCount());
  }
#endif
}
