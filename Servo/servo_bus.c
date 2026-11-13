/**
  ******************************************************************************
  * @file    servo_bus.c
  * @brief   总线舵机（CAN1）驱动：初始化 + 位置控制 + 位置回读
  ******************************************************************************
  */
#include "servo_bus.h"
#include "can.h"
#include "main.h"

/* ==================== 回读缓存 ==================== */
#define SERVO_POS_NUM   5u

static const uint32_t s_pos_id[SERVO_POS_NUM] =
{
  SERVO_ID_ARM0, SERVO_ID_ARM1, SERVO_ID_ARM2, SERVO_ID_ARM3, SERVO_ID_GRIPPER
};

static uint16_t s_pos_raw[SERVO_POS_NUM];     /* 最近一次收到的原始位置 */
static uint8_t  s_pos_valid[SERVO_POS_NUM];   /* 1 = 收到过 */
static uint32_t s_rx_cnt = 0u;                /* 累计帧数（诊断） */

/* 按手册规则：ID > 0x0A 用扩展帧，否则标准帧 */
bool ServoBus_SendRaw(uint32_t id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx;
  uint32_t mailbox;

  tx.RTR = CAN_RTR_DATA;
  tx.DLC = len;
  tx.TransmitGlobalTime = DISABLE;

  /* 两个 ID 字段都填，避免复用 TxHeader 时残留旧值 */
  tx.StdId = id;
  tx.ExtId = id;
  tx.IDE   = (id > 0x0Au) ? CAN_ID_EXT : CAN_ID_STD;

  return (HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)data, &mailbox) == HAL_OK);
}

bool ServoBus_EnableReport(uint32_t id, uint8_t period_ms)
{
  uint8_t frame[3];

  frame[0] = SERVO_CMD_ENABLE_RSP;   /* 0x19 */
  frame[1] = period_ms;              /* 周期 ms（参考工程用 0x1D = 29ms） */
  frame[2] = 0x00u;

  return ServoBus_SendRaw(id, frame, 3u);
}

void ServoBus_Poll(void)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t  data[8];
  uint32_t id;
  uint8_t  i;

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) != HAL_OK)
    {
      break;
    }
    if (rx.DLC < 3u)
    {
      continue;
    }

    /*
     * ★只按 CAN ID 匹配，**不检查命令码**。
     *   参考工程的做法就是这样；之前限定 data[0] 必须是 0x02/0x1A，
     *   结果回传帧的命令码对不上，位置永远是 0。
     *   位置字节：低字节在 data[1]，高字节在 data[2]。
     */
    id = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;

    for (i = 0u; i < SERVO_POS_NUM; i++)
    {
      if (id == s_pos_id[i])
      {
        uint16_t raw = (uint16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[1]);

        if (raw > SERVO_RAW_MAX)
        {
          raw = SERVO_RAW_MAX;
        }
        s_pos_raw[i]   = raw;
        s_pos_valid[i] = 1u;
        s_rx_cnt++;
        break;
      }
    }
  }
}

static int8_t ServoBus_FindIndex(uint32_t id)
{
  uint8_t i;

  for (i = 0u; i < SERVO_POS_NUM; i++)
  {
    if (id == s_pos_id[i])
    {
      return (int8_t)i;
    }
  }
  return -1;
}

bool ServoBus_GetRaw(uint32_t id, uint16_t *raw)
{
  int8_t idx = ServoBus_FindIndex(id);

  if ((idx < 0) || (s_pos_valid[(uint8_t)idx] == 0u))
  {
    return false;
  }
  if (raw != NULL)
  {
    *raw = s_pos_raw[(uint8_t)idx];
  }
  return true;
}

bool ServoBus_GetAngle(uint32_t id, float *deg)
{
  uint16_t raw;

  if (!ServoBus_GetRaw(id, &raw))
  {
    return false;
  }
  if (deg != NULL)
  {
    *deg = (float)raw * 360.0f / 4096.0f;
  }
  return true;
}

uint32_t ServoBus_GetRxCount(void)
{
  return s_rx_cnt;
}

/* ==================== 初始化 ==================== */

void ServoBus_Init(void)
{
  CAN_FilterTypeDef filter;
  uint8_t i;

  /* 全通滤波器：接收总线上所有帧 */
  filter.FilterIdHigh         = 0x0000u;
  filter.FilterIdLow          = 0x0000u;
  filter.FilterMaskIdHigh     = 0x0000u;
  filter.FilterMaskIdLow      = 0x0000u;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterBank           = 0u;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14u;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  for (i = 0u; i < SERVO_POS_NUM; i++)
  {
    s_pos_raw[i]   = 0u;
    s_pos_valid[i] = 0u;
  }
  s_rx_cnt = 0u;

  /*
   * 给 5 个地址开启周期位置回传。
   *   发送邮箱只有 3 个，所以每帧之间让出 2ms 等它发出去，
   *   否则后面的帧会因为邮箱满被丢掉（驱动板就收不到开启指令）。
   */
  for (i = 0u; i < SERVO_POS_NUM; i++)
  {
    (void)ServoBus_EnableReport(s_pos_id[i], (uint8_t)SERVO_REPORT_PERIOD_MS);
    HAL_Delay(2u);
  }
}

bool ServoBus_SetAngle(uint32_t id, float angle_deg, uint8_t speed)
{
  uint8_t  frame[8] = { 0u };
  uint16_t raw;

  /* 手册要求非负角度：负角度换算结果异常 */
  if (angle_deg < 0.0f)
  {
    angle_deg = 0.0f;
  }
  raw = (uint16_t)(angle_deg * 4096.0f / 360.0f);
  if (raw > SERVO_RAW_MAX)
  {
    raw = SERVO_RAW_MAX;
  }

  frame[0] = SERVO_CMD_POSITION;
  frame[1] = (uint8_t)(raw & 0xFFu);   /* 角度低字节 */
  frame[2] = (uint8_t)(raw >> 8);      /* 角度高字节 */
  frame[3] = speed;                    /* 速度（单字节 0~255） */

  /* 手册要求同一帧连发两次，单次在总线上容易丢动作 */
  if (!ServoBus_SendRaw(id, frame, 8u))
  {
    return false;
  }
  HAL_Delay(2u);
  (void)ServoBus_SendRaw(id, frame, 8u);
  return true;
}
