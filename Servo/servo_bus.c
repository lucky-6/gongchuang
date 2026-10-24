/**
  ******************************************************************************
  * @file    servo_bus.c
  * @brief   总线舵机（CAN1）最简驱动实现
  ******************************************************************************
  */
#include "servo_bus.h"
#include "can.h"

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

void ServoBus_Init(void)
{
  CAN_FilterTypeDef filter;

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
