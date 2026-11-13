/**
  ******************************************************************************
  * @file    app_mission.h
  * @brief   比赛业务流程（状态机）：对外只有 4 个入口
  ******************************************************************************
  */
#ifndef APP_MISSION_H
#define APP_MISSION_H

#include <stdint.h>
#include "app_ctx.h"

/**
  * @brief 上电是否自动开跑（1 = 上电 3s 后自动启动，便于串口看流程）
  * @note  ★正式上场前必须改成 0，并接真"一键启动"按键。
  */
#define MISSION_AUTOSTART      1

/** @brief 全流程硬超时（ms）：超过就直接回启停区，保住"回位 2 分" */
#define MISSION_HARD_ABORT_MS  150000u

/** @brief 初始化：绑定端口、清上下文、重置所有 Sequence（不启动） */
void Mission_Init(void);

/** @brief 启动一次比赛（等于按下"启动"键） */
void Mission_Start(void);

/** @brief 主循环调用：推进一步，绝不阻塞 */
void Mission_Task(void);

/** @brief 1 = 正在跑（未结束、未中止） */
uint8_t Mission_IsRunning(void);

/** @brief 取只读上下文（任务码、计数、降级等级） */
const Ctx *Mission_GetCtx(void);

/** @brief 当前顶层步骤序号（调试看流程走到哪） */
uint8_t Mission_GetStep(void);

/** @brief 当前顶层步骤名字（"nav_qr" / "place_1" …） */
const char *Mission_GetStepName(void);

#endif /* APP_MISSION_H */
