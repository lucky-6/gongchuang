/**
  ******************************************************************************
  * @file    hwt101.c
  * @brief   HWT101CT 驱动实现
  ******************************************************************************
  */
#include "hwt101.h"
#include "usart.h"
#include <string.h>

/*
 * 帧内字节位置（以帧首 0x55 为偏移 0）
 *
 *   0x53: [0x55][0x53][00][00][00][00][YawL][YawH][VL][VH][SUM]
 *   0x52: [0x55][0x52][00][00][RWzL][RWzH][WzL][WzH][00][00][SUM]
 *
 * 注意：《HWT101使用手册》里写的是从 i+7 / i+8 取角度，与实测不符。
 *       这里按 robot 工程实测修正后的格式（角度在 6/7 字节，低字节在前）。
 *       若现场抓帧发现角度不对，只改下面这几个偏移即可。
 */
#define IDX_YAW_L     6u
#define IDX_YAW_H     7u
#define IDX_RAWGZ_L   4u
#define IDX_RAWGZ_H   5u
#define IDX_GZ_L      6u
#define IDX_GZ_H      7u
#define IDX_VER_L     8u
#define IDX_VER_H     9u
#define IDX_TYPE      1u
#define IDX_SUM       (HWT101_FRAME_LEN - 1u)
#define TYPE_ANGLE    0x53u
#define TYPE_GYRO     0x52u

/* DMA 重启的最小间隔 (ms)，避免起不来时空转刷 CPU */
#define HWT101_RETRY_PERIOD_MS   100u

HWT101_Data_t g_hwt101;

static uint8_t  s_dma_buf[HWT101_DMA_BUF_LEN];   /* DMA 循环接收缓冲 */
static uint16_t s_write_pos  = 0u;               /* 上一次已消费到的写入位置 */
static uint8_t  s_frame[HWT101_FRAME_LEN];       /* 帧组装缓冲 */
static uint8_t  s_idx        = 0u;               /* 帧组装位置 */
static uint32_t s_err_cnt    = 0u;               /* 校验/格式错误计数 */
static uint32_t s_last_retry = 0u;

/* ---------- 内部函数 ---------- */

static int16_t HWT101_ReadInt16(const uint8_t *f, uint8_t idx_low)
{
  return (int16_t)((uint16_t)f[idx_low + 1u] << 8 | (uint16_t)f[idx_low]);
}

static void HWT101_Decode(const uint8_t *f)
{
  uint8_t sum = 0u;

  for (uint8_t i = 0u; i < IDX_SUM; i++)
  {
    sum = (uint8_t)(sum + f[i]);        /* 校验和 = 前 10 字节累加取低 8 位 */
  }
  if (sum != f[IDX_SUM])
  {
    s_err_cnt++;
    return;
  }

  switch (f[IDX_TYPE])
  {
    case TYPE_ANGLE:                    /* 0x53：偏航角 + 版本号 */
    {
      g_hwt101.yaw      = (float)HWT101_ReadInt16(f, IDX_YAW_L) * HWT101_ANGLE_LSB;
      g_hwt101.version  = (uint16_t)((uint16_t)f[IDX_VER_H] << 8 | f[IDX_VER_L]);
      g_hwt101.stamp_ms = HAL_GetTick();
      g_hwt101.frame_cnt++;
    } break;

    case TYPE_GYRO:                     /* 0x52：Z 轴角速度 */
    {
      g_hwt101.raw_gz = (float)HWT101_ReadInt16(f, IDX_RAWGZ_L) * HWT101_GYRO_LSB;
      g_hwt101.gz     = (float)HWT101_ReadInt16(f, IDX_GZ_L)    * HWT101_GYRO_LSB;
    } break;

    default:
      break;
  }
}

/* 逐字节喂入，按 0x55 同步、满 11 字节即校验解析 */
static void HWT101_Feed(const uint8_t *p, uint16_t n)
{
  for (uint16_t i = 0u; i < n; i++)
  {
    uint8_t b = p[i];

    if (s_idx == 0u)
    {
      if (b != 0x55u)
      {
        continue;                       /* 等帧头 */
      }
    }

    s_frame[s_idx] = b;
    s_idx++;

    if (s_idx >= HWT101_FRAME_LEN)
    {
      s_idx = 0u;
      HWT101_Decode(s_frame);
    }
  }
}

static void HWT101_SendCmd(uint8_t cmd, uint8_t param)
{
  uint8_t f[5] = {0xFFu, 0xAAu, cmd, param, 0x00u};

  (void)HAL_UART_Transmit(&huart1, f, 5u, 100u);
}

/*
 * 启动循环 DMA 接收。
 *
 * 直接操作 DMA 与 USART 寄存器，不走 HAL_UARTEx_ReceiveToIdle_DMA()：
 * 那条路径依赖 huart->RxState / ReceptionType 等状态，任一环节不对就静默不收数。
 *   1) HAL_DMA_Start_IT() 按 hdma->Init 里的 CIRCULAR 模式启动（NDTR = 缓冲长度）
 *   2) 关掉所有 DMA 中断（纯轮询，不需要中断）
 *   3) 打开 USART 的 DMA 接收请求（CR3.DMAR）
 * 前提：DMA2 时钟已由 MX_DMA_Init() 打开，否则寄存器写入会被丢弃且不报错。
 */
static HAL_StatusTypeDef HWT101_StartRx(void)
{
  HAL_StatusTypeDef st;

  st = HAL_DMA_Start_IT(&hdma_usart1_rx, (uint32_t)&huart1.Instance->DR,
                        (uint32_t)s_dma_buf, HWT101_DMA_BUF_LEN);
  if (st != HAL_OK)
  {
    return st;
  }

  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_TE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_FE);

  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  huart1.RxState = HAL_UART_STATE_BUSY_RX; /* 让 HAL 知道接收已占用 */
  return HAL_OK;
}

/* ---------- 对外接口 ---------- */

void HWT101_Init(void)
{
  memset(&g_hwt101, 0, sizeof(g_hwt101));
  memset(s_frame, 0, sizeof(s_frame));
  s_idx        = 0u;
  s_write_pos  = 0u;
  s_err_cnt    = 0u;
  s_last_retry = 0u;

  (void)HWT101_StartRx();
}

/*
 * 读 DMA 剩余计数（NDTR）算出写入位置，按位置增量消费。
 * 循环 DMA 下位置会回绕，分“未回绕”和“回绕”两种情况取数据，
 * 避免重复解析或漏数据。
 */
void HWT101_Poll(void)
{
  uint16_t ndtr;
  uint16_t pos;

  /* DMA 流没在跑（被中止/出错）就重新挂起来，避免一次异常后永久收不到数据 */
  if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) == 0u)
  {
    if ((HAL_GetTick() - s_last_retry) >= HWT101_RETRY_PERIOD_MS)
    {
      s_last_retry = HAL_GetTick();
      hdma_usart1_rx.State = HAL_DMA_STATE_READY;
      __HAL_UNLOCK(&hdma_usart1_rx);
      (void)HWT101_StartRx();
      s_write_pos = 0u;                 /* 位置归零，避免用旧位置取到错数据 */
    }
  }

  ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  if (ndtr == 0u)
  {
    return;                             /* 缓冲刚好写满或 DMA 未运行，无法判断位置 */
  }

  pos = (uint16_t)(HWT101_DMA_BUF_LEN - ndtr);

  if (pos >= s_write_pos)
  {
    HWT101_Feed(&s_dma_buf[s_write_pos], (uint16_t)(pos - s_write_pos));
  }
  else                                  /* 回绕：先收尾段，再收头部 */
  {
    HWT101_Feed(&s_dma_buf[s_write_pos], (uint16_t)(HWT101_DMA_BUF_LEN - s_write_pos));
    HWT101_Feed(s_dma_buf, pos);
  }

  s_write_pos = pos;
}

bool HWT101_GetData(HWT101_Data_t *out)
{
  if (out == NULL)
  {
    return false;
  }
  *out = g_hwt101;
  return (g_hwt101.frame_cnt > 0u);
}

float HWT101_GetYaw(void)
{
  return g_hwt101.yaw;
}

bool HWT101_IsOnline(void)
{
  return (g_hwt101.frame_cnt > 0u) &&
         ((HAL_GetTick() - g_hwt101.stamp_ms) < HWT101_OFFLINE_MS);
}

uint32_t HWT101_GetFrameCount(void)
{
  return g_hwt101.frame_cnt;
}

uint32_t HWT101_GetErrCount(void)
{
  return s_err_cnt;
}

void HWT101_SetZero(void)
{
  HWT101_SendCmd(0x76u, 0x00u);
}

void HWT101_SetRate(uint8_t hz)
{
  HWT101_SendCmd(0x03u, hz);
}

void HWT101_SetBaud(uint8_t baud)
{
  HWT101_SendCmd(0x04u, baud);
}

void HWT101_Config(uint8_t hz, uint8_t baud)
{
  HWT101_SetZero();
  HAL_Delay(20u);
  HWT101_SetRate(hz);
  HAL_Delay(20u);
  HWT101_SetBaud(baud);
  HAL_Delay(20u);
}
