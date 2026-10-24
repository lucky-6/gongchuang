/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
/*
 * 总线舵机用：PA11 = CAN1_RX，PA12 = CAN1_TX
 *
 * 位时序按 robot 工程（可用版本）：
 *   Prescaler 6 / BS1 4TQ / BS2 2TQ / SJW 1TQ，共 7 Tq
 *   t_q = 6/42MHz = 142.9ns，位时间 = 7 × 142.9ns = 1us -> 1 Mbps
 *   （SYSCLK 168MHz，APB1 = 42MHz）
 *
 * AutoRetransmission 关掉：没人 ACK 的帧不会反复重发占住发送邮箱。
 */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  /*
   * AutoBusOff 打开：发送错误计数累到 256 时进 bus-off 并自动恢复，
   * 避免总线上没人 ACK 时永久卡死。
   */
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
