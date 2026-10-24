/**
  ******************************************************************************
  * @file    bsp_rs485.c
  * @brief   RS485 半双工收发驱动
  ******************************************************************************
  */
#include "bsp_rs485.h"
#include "usart.h"

static uint8_t  s_rx_byte;                                  /* 中断接收暂存 */
static uint8_t  s_rx_buf[RS485_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;
static volatile uint16_t s_rx_cnt  = 0u;

volatile uint32_t g_rs485_tx_frames = 0u;
volatile uint32_t g_rs485_rx_bytes  = 0u;
volatile uint32_t g_rs485_rx_drops  = 0u;

/* 短延时：HCLK 16 MHz 下每次循环约 4 个周期 */
static void RS485_DelayUs(uint32_t us)
{
  uint32_t count = us * (SystemCoreClock / 4000000u);
  while (count-- > 0u)
  {
    __NOP();
  }
}

void RS485_Init(void)
{
  RS485_DE_RX();                 /* 上电先处于接收态，避免占住总线 */
  RS485_FlushRx();
  (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
}

void RS485_FlushRx(void)
{
  __disable_irq();
  s_rx_head = 0u;
  s_rx_tail = 0u;
  s_rx_cnt  = 0u;
  __enable_irq();
}

uint16_t RS485_Read(uint8_t *buf, uint16_t max_len)
{
  uint16_t n = 0u;

  while ((n < max_len) && (s_rx_cnt > 0u))
  {
    buf[n] = s_rx_buf[s_rx_tail];
    n++;
    __disable_irq();
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RS485_RX_BUF_SIZE);
    s_rx_cnt--;
    __enable_irq();
  }
  return n;
}

bool RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef st;

  if ((data == NULL) || (len == 0u))
  {
    return false;
  }

  RS485_DE_TX();                        /* 切到发送 */
  RS485_DelayUs(3u);                    /* 等收发器方向切换稳定 */

  st = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, timeout_ms);

  RS485_DE_RX();                        /* 无论成败都要切回接收，否则总线被占死 */

  if (st == HAL_OK)
  {
    g_rs485_tx_frames++;
    return true;
  }
  return false;
}

bool RS485_WaitAck(uint8_t addr, uint8_t code, uint8_t *data, uint32_t timeout_ms)
{
  uint8_t  f[8];
  uint8_t  len = 0u;
  uint8_t  b;
  uint8_t  sum;
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    if (RS485_Read(&b, 1u) == 0u)
    {
      continue;
    }

    if ((len == 0u) && (b != 0xFBu))
    {
      continue;                          /* 等帧头 FB */
    }

    f[len] = b;
    len++;

    if (len >= 4u)
    {
      sum = 0u;
      for (uint8_t i = 0u; i < (uint8_t)(len - 1u); i++)
      {
        sum = (uint8_t)(sum + f[i]);
      }

      if (sum == f[len - 1u])            /* 校验和正确 */
      {
        if ((f[1] == addr) && (f[2] == code))
        {
          if ((data != NULL) && (len >= 5u))
          {
            *data = f[3];
          }
          return true;
        }
        len = 0u;                        /* 不是期望的帧，重新找帧头 */
      }
      else if (len >= sizeof(f))
      {
        len = 0u;
      }
    }
  }
  return false;
}

/* USART2 接收中断：把字节搬进环形缓冲，不做解析 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (s_rx_cnt < RS485_RX_BUF_SIZE)
    {
      s_rx_buf[s_rx_head] = s_rx_byte;
      s_rx_head = (uint16_t)((s_rx_head + 1u) % RS485_RX_BUF_SIZE);
      s_rx_cnt++;
      g_rs485_rx_bytes++;
    }
    else
    {
      g_rs485_rx_drops++;
    }

    if (HAL_UART_Receive_IT(huart, &s_rx_byte, 1u) != HAL_OK)
    {
      /* HAL 偶发状态异常时强制复位再重新武装，避免接收永久停摆 */
      huart->RxState = HAL_UART_STATE_READY;
      __HAL_UNLOCK(huart);
      (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
    }
  }
}
