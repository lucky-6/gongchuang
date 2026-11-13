/**
  ******************************************************************************
  * @file    adapt_ports.h
  * @brief   端口适配器：把 app_frame 的 Ports 接到本工程真实的驱动上
  *
  *   ★核心开关：g_adapt_motion_real
  *      0 = 仿真动作（默认）：nav/arm 只按耗时"假装"完成，车和臂都不动。
  *          用于先跑通业务流程、验证状态机，零风险。
  *      1 = 真实动作：调用真实驱动。★在 app_nav / app_pose 写出来之前，
  *          真实模式下的 nav/arm 仍然是 TODO（只打印告警），不会乱动。
  *
  *   ★视觉台账在适配器里强制：同一 (zone,item) 第二次请求直接返回 false，
  *     对应任务书 C1"每次拿取/放置只有一次视觉调整机会"。
  ******************************************************************************
  */
#ifndef ADAPT_PORTS_H
#define ADAPT_PORTS_H

#include <stdint.h>
#include "app_ports.h"

/** @brief 初始化适配器并返回端口表（供 Frame_Init 使用） */
const Ports *Adapt_GetPorts(void);

/** @brief 适配器自身需要周期处理的事（当前为空，留作扩展） */
void Adapt_Task(void);

/* ---------- 调试开关（可在调试器里直接改，或改下面默认值） ---------- */

/** 0 = 仿真动作（不动车/臂）；1 = 真实动作。默认 0，先跑流程 */
extern volatile uint8_t g_adapt_motion_real;

/** 1 = 打印每一步动作（调试期建议 1，赛后关掉省时间） */
extern volatile uint8_t g_adapt_verbose;

/** 1 = 发生过急停/致命错误（业务层可据此中止） */
extern volatile uint8_t g_adapt_fault;

/* ---------- 观测量（调试器里看） ---------- */
extern volatile uint32_t g_adapt_nav_cnt;      /* 导航指令次数 */
extern volatile uint32_t g_adapt_arm_cnt;      /* 臂动作次数 */
extern volatile uint32_t g_adapt_vis_cnt;      /* 视觉请求次数 */
extern volatile uint32_t g_adapt_vis_dup;      /* 视觉重复请求次数（应恒 0） */
extern volatile float    g_adapt_sim_x;        /* 仿真位姿（mm） */
extern volatile float    g_adapt_sim_y;
extern volatile float    g_adapt_sim_yaw;

#endif /* ADAPT_PORTS_H */
