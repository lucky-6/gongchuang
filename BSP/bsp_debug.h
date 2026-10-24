/**
  ******************************************************************************
  * @file    bsp_debug.h
  * @brief   调试打印口（UART5，PC12 = TX，PD2 = RX，115200 8N1）
  * @note    依赖 main() 里已调用 MX_UART5_Init()；
  *          发送是阻塞式的，请在主循环里调用，不要在中断里调用。
  ******************************************************************************
  */
#ifndef BSP_DEBUG_H
#define BSP_DEBUG_H

#include <stdint.h>

/** @brief 发送原始字节 */
void Debug_Write(const uint8_t *data, uint16_t len);

/** @brief 发送字符串（不带换行） */
void Debug_Puts(const char *s);

/**
  * @brief  格式化打印，等价于 printf
  * @note   支持 %f（已在 CMake 里开 -u _printf_float），单行超长会截断
  */
void Debug_Printf(const char *fmt, ...);

/* 统计量：发送成功次数 / 超时次数 */
extern volatile uint32_t g_debug_frames;
extern volatile uint32_t g_debug_timeouts;

#endif /* BSP_DEBUG_H */
