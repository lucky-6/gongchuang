/**
  ******************************************************************************
  * @file    bsp_debug.c
  * @brief   调试打印口实现（UART5）
  ******************************************************************************
  */
#include "bsp_debug.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define DEBUG_BUF_SIZE     128u   /* 单行最大长度（含结尾） */
#define DEBUG_TX_TIMEOUT   100u   /* 发送超时 (ms) */

static char s_buf[DEBUG_BUF_SIZE];

volatile uint32_t g_debug_frames   = 0u;
volatile uint32_t g_debug_timeouts = 0u;

void Debug_Write(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0u))
  {
    return;
  }

  if (HAL_UART_Transmit(&huart5, (uint8_t *)data, len, DEBUG_TX_TIMEOUT) == HAL_OK)
  {
    g_debug_frames++;
  }
  else
  {
    g_debug_timeouts++;
  }
}

void Debug_Puts(const char *s)
{
  if (s == NULL)
  {
    return;
  }
  Debug_Write((const uint8_t *)s, (uint16_t)strlen(s));
}

void Debug_Printf(const char *fmt, ...)
{
  va_list ap;
  int n;

  if (fmt == NULL)
  {
    return;
  }

  va_start(ap, fmt);
  n = vsnprintf(s_buf, sizeof(s_buf), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }
  if (n >= (int)sizeof(s_buf))
  {
    n = (int)sizeof(s_buf) - 1;      /* 超长截断，保证以 '\0' 结尾 */
  }

  Debug_Write((const uint8_t *)s_buf, (uint16_t)n);
}
