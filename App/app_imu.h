/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   陀螺仪接入层：初始化与周期轮询
  ******************************************************************************
  */
#ifndef APP_IMU_H
#define APP_IMU_H

/** @brief 初始化陀螺仪（需在 MX_DMA_Init() 与 MX_USART1_UART_Init() 之后调用） */
void AppIMU_Init(void);

/** @brief 周期任务：轮询陀螺仪数据，请在主循环里反复调用 */
void AppIMU_Update(void);

#endif /* APP_IMU_H */
