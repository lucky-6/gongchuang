/**
  ******************************************************************************
  * @file    hwt101.h
  * @brief   WitMotion HWT101CT 单轴陀螺仪（Z 轴 / 偏航角）驱动
  *
  *   硬件：USART1，PA9 = TX，PA10 = RX，模块 TX 接 PA10
  *   接收：DMA2_Stream2 循环接收，主循环轮询 NDTR 取数（不依赖中断）
  *   协议：帧头 0x55 + 类型 + 4×2 字节数据（低字节在前）+ 校验和
  *         0x53 = 偏航角 + 版本号，0x52 = Z 轴角速度
  *   依据：工程内《HWT101使用手册》+ robot 工程 hwt101ct.c 实测修正
  ******************************************************************************
  */
#ifndef HWT101_H
#define HWT101_H

#include <stdint.h>
#include <stdbool.h>

#define HWT101_FRAME_LEN     11u    /* 帧长：2 帧头 + 8 数据 + 1 校验 */
#define HWT101_DMA_BUF_LEN   256u   /* DMA 循环接收缓冲（100Hz 约 230ms 余量） */

/* 角度换算：原始值满量程 32768 对应 180° */
#define HWT101_ANGLE_LSB     (180.0f / 32768.0f)
/* 角速度换算：满量程 ±2000 °/s */
#define HWT101_GYRO_LSB      (2000.0f / 32768.0f)

/* 超过该时间没收到有效帧即认为掉线 (ms) */
#define HWT101_OFFLINE_MS    300u

/** @brief 传感器数据 */
typedef struct
{
  float    yaw;         /* 偏航角 (°)，范围 -180 ~ +180 */
  float    gz;          /* Z 轴角速度 (°/s)，已校准值 */
  float    raw_gz;      /* Z 轴角速度 (°/s)，原始值 */
  uint16_t version;     /* 固件版本号 */
  uint32_t stamp_ms;    /* 最近一帧有效数据的时间戳 */
  uint32_t frame_cnt;   /* 累计有效 0x53 帧数 */
} HWT101_Data_t;

extern HWT101_Data_t g_hwt101;    /* 全局数据，主循环读取 */

/**
  * @brief  启动接收（DMA 循环）
  * @note   必须在 MX_DMA_Init() 与 MX_USART1_UART_Init() 之后调用；
  *         只负责“收”，不改模块配置、不清零角度。
  */
void     HWT101_Init(void);

/** @brief 查询并消费 DMA 缓冲里的新数据，请在主循环里周期调用 */
void     HWT101_Poll(void);

/** @brief 取一份数据快照；返回 false 表示还没收到过有效帧 */
bool     HWT101_GetData(HWT101_Data_t *out);

/** @brief 取当前偏航角 (°) */
float    HWT101_GetYaw(void);

/** @brief 模块是否在线（HWT101_OFFLINE_MS 内有有效帧） */
bool     HWT101_IsOnline(void);

/** @brief 累计有效帧数 / 校验错误帧数 */
uint32_t HWT101_GetFrameCount(void);
uint32_t HWT101_GetErrCount(void);

/* ---- 配置指令：0xFF 0xAA 命令码 参数 0x00 ---- */

/** @brief Z 轴角度清零：把当前朝向设为 0°（FF AA 76 00 00） */
void     HWT101_SetZero(void);

/** @brief 设置主动上报速率：0x08 = 50Hz，0x09 = 100Hz（FF AA 03 HZ 00） */
void     HWT101_SetRate(uint8_t hz);

/** @brief 设置波特率：0x06 = 115200，0x07 = 230400（FF AA 04 B 00） */
void     HWT101_SetBaud(uint8_t baud);

/** @brief 一键配置：清零 + 上报速率 + 波特率（对应手册 HWT101_init） */
void     HWT101_Config(uint8_t hz, uint8_t baud);

#endif /* HWT101_H */
