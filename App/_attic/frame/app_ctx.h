#ifndef APP_CTX_H
#define APP_CTX_H

#include <stdint.h>
#include "app_ports.h"

#define BATCH_ITEMS   3u
#define TEAM_TASK_N   4u
#define RING_N        3u

/* 降级等级（只升不降） */
#define LVL_NORMAL     0u
#define LVL_NO_VERIFY  1u
#define LVL_PART_STACK 2u
#define LVL_HOME_ONLY  3u

/** @brief 一次比赛的上下文/黑板：所有业务状态都在这里，不用散全局变量 */
typedef struct
{
  /* 任务 */
  int8_t   task[TEAM_TASK_N][RING_N];   /* 任务码，0 基 */
  uint8_t  task_ok;
  char     task_str[20];

  /* 游标 */
  uint8_t  item;                        /* 当前第几件 */
  ZoneId   zone;                        /* 当前环节 */
  uint8_t  act_base;                    /* 本子流程的视觉槽位基址 */
  uint8_t  act;                         /* 本件的视觉槽位 = act_base + item */

  /* 每次测量的结果（一次性） */
  VisBias  bias;
  uint8_t  bias_ok;

  /* 计数 */
  uint16_t pick_ok;
  uint16_t pick_fail;
  uint16_t reload_ok;                   /* 从粗加工区取回的次数（与 pick_ok 分开计） */
  uint16_t place_ok;
  uint16_t place_fail;
  uint8_t  vision_used;

  /* 码垛前置：第一批暂存区每环是否放成功 */
  uint8_t  ring_ok[RING_N];

  /* 时间与降级 */
  uint32_t t_start;
  uint8_t  level;
  uint8_t  abort_reason;
} Ctx;

#endif /* APP_CTX_H */
