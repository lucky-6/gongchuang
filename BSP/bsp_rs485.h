/**
  ******************************************************************************
  * @file    bsp_rs485.h
  * @brief   RS485 半双工收发驱动（USART2 + PA4 方向控制）
  *          只负责“发一串字节 / 收一串字节”，不涉及电机协议语义。
  ******************************************************************************
  */
#ifndef BSP_RS485_H
#define BSP_RS485_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* 收发器方向脚：高 = 发送，低 = 接收 */
#define RS485_DE_TX()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define RS485_DE_RX()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)

#define RS485_RX_BUF_SIZE   64u     /* 接收环形缓冲大小（字节） */

/**
  * @brief  初始化 RS485：方向脚置接收态、清空接收缓冲、启动接收中断
  * @note   必须在 MX_USART2_UART_Init() 之后调用
  */
void     RS485_Init(void);

/**
  * @brief  发送一帧数据（阻塞式）
  * @param  data        待发送数据
  * @param  len         字节数
  * @param  timeout_ms  发送超时
  * @retval true 发送完成 / false 超时或参数错误
  * @note   HAL 阻塞发送返回时最后一个字节已移出（内部等 TC 标志），
  *         因此返回后立即切回接收态是安全的。
  */
bool     RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
  * @brief  从接收缓冲取数据
  * @retval 实际取到的字节数
  */
uint16_t RS485_Read(uint8_t *buf, uint16_t max_len);

/** @brief 清空接收缓冲（发送前调用，丢掉总线上的残留字节） */
void     RS485_FlushRx(void);

/**
  * @brief  等待一帧应答：帧头 FB + 地址 + 功能码，且校验和正确
  * @param  addr        期望的从机地址
  * @param  code        期望的功能码
  * @param  data        非空时带出应答的第 1 个数据字节
  * @param  timeout_ms  超时
  * @retval true 收到合法应答 / false 超时
  */
bool     RS485_WaitAck(uint8_t addr, uint8_t code, uint8_t *data, uint32_t timeout_ms);

/* 调试用统计量（可在 IDE 调试器里直接观察） */
extern volatile uint32_t g_rs485_tx_frames;   /* 已发送帧数 */
extern volatile uint32_t g_rs485_rx_bytes;    /* 已接收字节数 */
extern volatile uint32_t g_rs485_rx_drops;    /* 因缓冲区满丢弃的字节数 */

#endif /* BSP_RS485_H */
