/**
  ******************************************************************************
  * @file    app_mission.c
  * @brief   比赛业务流程：19 步顶层表 + 3 张子表（放置/抓取/取回）
  *
  *   写法约定的三条铁律（改这个文件时请守住）：
  *     1. 不准出现 HAL_Delay / while 等待 / 自己的 static 计数器
  *     2. 步骤内要"发起→等待→收尾"时，用 st->phase 记进度
  *     3. 这一步失败了整场还有没有意义？有 → 标 STEP_OPTIONAL
  *
  *   代码结构：
  *     [1] 配置（区域位姿、计划时间）
  *     [2] 子表：放置（9 平面 + 3 码垛 = 12 次复用）
  *     [3] 子表：抓取（转盘，含视觉）
  *     [4] 子表：取回（粗加工区，不用视觉）
  *     [5] 顶层：19 步表
  *     [6] 入口 Mission_*
  ******************************************************************************
  */
#include "app_mission.h"

#include "main.h"          /* HAL_GetTick() */
#include "app_frame.h"
#include "app_chassis.h"   /* Chassis_SetAutoHome / Chassis_Stop */
#include "adapt_ports.h"

/* ==================================================================== */
/* [1] 配置                                                            */
/* ==================================================================== */

/** @brief 各区域停靠位姿（mm / 0.1°）。
 *  ★TODO: 这些是占位值，必须按《标定表》实测替换（任务书 §7.8 的 W-CAL）。 */
typedef struct
{
  float x;
  float y;
  float yaw;
} ZonePose_t;

static const ZonePose_t k_zone[Z_NUM] =
{
  /* Z_QR    */ {  600.0f,    0.0f,    0.0f },
  /* Z_RAW   */ {    0.0f, -800.0f,  -90.0f },
  /* Z_ROUGH */ { -600.0f,    0.0f,  180.0f },
  /* Z_TEMP  */ { -300.0f,  600.0f,   90.0f },
  /* Z_STACK */ { -300.0f,  600.0f,   90.0f },
  /* Z_HOME  */ {    0.0f,    0.0f,    0.0f }
};

/* 各阶段计划累计耗时（ms），用于检查点与降级梯子 */
#define PLAN_CHECK_1_MS    7300u    /* 显示完任务码 */
#define PLAN_CHECK_2_MS   59300u    /* 第一批送进暂存区 */
#define PLAN_CHECK_3_MS   98300u    /* 第二批从粗加工取回 */
#define PLAN_CHECK_4_MS  114300u    /* 码垛完成 */
#define LATE_TOLERANCE_MS 20000u    /* 落后多少算"该降级" */

/* 单批件数 / 每批位置数 */
#define ITEM_N   BATCH_ITEMS

/* ==================================================================== */
/* 内部状态                                                            */
/* ==================================================================== */
static Ctx      s_ctx;
static uint8_t  s_started = 0u;
static uint8_t  s_done    = 0u;
static uint32_t s_boot_ms = 0u;

/* ==================================================================== */
/* [2] 放置子表：粗加工 / 暂存 / 码垛 共 12 次复用                        */
/* ==================================================================== */
enum
{
  PL_BEGIN = 0,
  PL_BIAS,
  PL_MOVE,
  PL_DOWN,
  PL_RELEASE,
  PL_LIFT,
  PL_VERIFY,
  PL_NEXT,
  PL_N
};

/* 当前这一件放几号环。
 * 注意：粗加工、暂存平放、码垛都用"组2"——码垛是叠在第一层物料上，
 *       第一层的位置由组2 决定；组4 只规定第二批在【粗加工区】的位置。 */
static int8_t RingOf(const Ctx *c)
{
  uint8_t grp = 1u;

  if (c->item >= ITEM_N)
  {
    return 1;
  }
  {
    int8_t r = c->task[grp][c->item];
    if ((r < 0) || (r >= (int8_t)RING_N))
    {
      return 1;                        /* 任务码异常 → 落到中位，不硬撞 */
    }
    return r;
  }
}

static Result pl_begin(Seq *s, Ctx *c)
{
  (void)s;
  c->item = 0u;
  return RES_OK;
}

/* 唯一一次视觉：请求偏置 → 拿偏置；拿不到就用 0 偏置，绝不请求第二次 */
static Result pl_bias(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    c->bias_ok     = 0u;
    c->bias.dx     = 0.0f;
    c->bias.dy     = 0.0f;
    c->bias.eta_ms = 0.0f;

    c->act = (uint8_t)(c->act_base + c->item);   /* 视觉台账唯一槽位 */
    if (!P->vis_bias_start(c->zone, c->act))
    {
      return RES_OK;                   /* 台账已用 / 有请求在途 → 降级为 0 偏置 */
    }
    c->vision_used++;
    st->phase = 1u;
  }

  if (P->vis_bias_poll(&c->bias))
  {
    c->bias_ok = 1u;
    return RES_OK;
  }
  return RES_RUNNING;
}

static Result pl_move(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_move(c->zone, RingOf(c), &c->bias, (c->zone == Z_STACK) ? 1u : 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result pl_down(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_descend(c->zone, RingOf(c), (c->zone == Z_STACK) ? 1u : 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result pl_release(Seq *s, Ctx *c)
{
  (void)c;
  if (s->steps[s->idx].phase == 0u)
  {
    P->grip_open();
    s->steps[s->idx].phase = 1u;
  }
  return RES_OK;
}

static Result pl_lift(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_lift(10.0f);                /* 抬 ≥10mm，避免碰倒刚放的物料 */
    st->phase = 1u;
    return RES_RUNNING;
  }
  if (P->arm_state() == 0u)
  {
    return RES_RUNNING;
  }
  if (c->level != LVL_NORMAL)
  {
    c->place_ok++;                     /* 降级时不做回看，按动作成功计数 */
  }
  return RES_OK;
}

/* 释放后确认：只记数，绝不修正（落地即计分，再动即判负） */
static Result pl_verify(Seq *s, Ctx *c)
{
  Step   *st = &s->steps[s->idx];
  uint8_t ok = 0u;

  if (c->level != LVL_NORMAL)
  {
    return RES_OK;                     /* 已在 pl_lift 计过数 */
  }
  if (st->phase == 0u)
  {
    if (!P->vis_verify_start(c->zone, RingOf(c)))
    {
      c->place_ok++;
      return RES_OK;
    }
    st->phase = 1u;
  }
  if (P->vis_verify_poll(&ok))
  {
    if (ok != 0u) { c->place_ok++; } else { c->place_fail++; }
    return RES_OK;
  }
  return RES_RUNNING;
}

static Result pl_next(Seq *s, Ctx *c)
{
  P->disp_counts(c->pick_ok, c->place_ok);
  c->item++;
  if (c->item < ITEM_N)
  {
    Seq_JumpTo(s, PL_BIAS);
  }
  return RES_OK;
}

SEQ_DEF(seq_place,
        STEP(pl_begin,   "begin",   200u, 0u, 0u),
        STEP(pl_bias,    "bias",    600u, 0u, STEP_OPTIONAL),
        STEP(pl_move,    "move",   1500u, 0u, 0u),
        STEP(pl_down,    "descend", 800u, 0u, STEP_NO_KICK),
        STEP(pl_release, "release", 500u, 0u, STEP_NO_KICK),
        STEP(pl_lift,    "lift",    800u, 0u, 0u),
        STEP(pl_verify,  "verify",  400u, 0u, STEP_OPTIONAL),
        STEP(pl_next,    "next",    200u, 0u, 0u));

/* ==================================================================== */
/* [3] 抓取子表：从转盘抓（有视觉）                                      */
/* ==================================================================== */
enum
{
  PK_BEGIN = 0,
  PK_BIAS,
  PK_MOVE,
  PK_DOWN,
  PK_GRIP,
  PK_CHECK,
  PK_LIFT,
  PK_TOCAR,
  PK_NEXT,
  PK_N
};

static Result pk_begin(Seq *s, Ctx *c)
{
  (void)s;
  c->item = 0u;
  return RES_OK;
}

static Result pk_bias(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    c->bias_ok     = 0u;
    c->bias.dx     = 0.0f;
    c->bias.dy     = 0.0f;
    c->bias.eta_ms = 0.0f;

    c->act = (uint8_t)(c->act_base + c->item);   /* 视觉台账唯一槽位 */
    if (!P->vis_bias_start(c->zone, c->act))
    {
      return RES_OK;
    }
    c->vision_used++;
    st->phase = 1u;
  }
  if (P->vis_bias_poll(&c->bias))
  {
    c->bias_ok = 1u;
    return RES_OK;
  }
  return RES_RUNNING;
}

static Result pk_move(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_move(c->zone, (int8_t)c->item, &c->bias, 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result pk_down(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_descend(c->zone, (int8_t)c->item, 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result pk_grip(Seq *s, Ctx *c)
{
  (void)c;
  if (s->steps[s->idx].phase == 0u)
  {
    P->grip_close();
    s->steps[s->idx].phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

/* 夹空判定：夹空就给一次"固定档位"盲重试（不消耗视觉机会），再空就跳过 */
static Result pk_check(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (P->grip_state() == 2u)           /* 2 = 夹空 */
  {
    if (st->xi[0] < 2)                 /* 最多盲重试 2 次 */
    {
      st->xi[0]++;
      Seq_JumpTo(s, PK_DOWN);          /* 回到下探，用同一份偏置 */
      return RES_OK;
    }
    c->pick_fail++;
    Seq_JumpTo(s, PK_NEXT);            /* 放弃这一件，绝不原地卡住 */
    return RES_OK;
  }
  return RES_OK;
}

static Result pk_lift(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  (void)c;
  if (st->phase == 0u)
  {
    P->arm_lift(30.0f);                /* 抓取抬高 30mm 再走 */
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result pk_tocar(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->grip_open();                    /* 放到车上料仓后松开 */
    st->phase = 1u;
    return RES_RUNNING;
  }
  if (P->arm_state() == 0u)
  {
    return RES_RUNNING;
  }
  c->pick_ok++;
  P->disp_counts(c->pick_ok, c->place_ok);
  return RES_OK;
}

static Result pk_next(Seq *s, Ctx *c)
{
  c->item++;
  if (c->item < ITEM_N)
  {
    Seq_JumpTo(s, PK_BIAS);
  }
  return RES_OK;
}

SEQ_DEF(seq_pick,
        STEP(pk_begin,  "p_begin",   200u, 0u, 0u),
        STEP(pk_bias,   "p_bias",    600u, 0u, STEP_OPTIONAL),
        STEP(pk_move,   "p_move",   1200u, 0u, 0u),
        STEP(pk_down,   "p_down",    600u, 0u, STEP_NO_KICK),
        STEP(pk_grip,   "p_grip",    500u, 1u, STEP_NO_KICK),
        STEP(pk_check,  "p_check",   200u, 0u, 0u),
        STEP(pk_lift,   "p_lift",    600u, 0u, 0u),
        STEP(pk_tocar,  "p_tocar",   600u, 0u, 0u),
        STEP(pk_next,   "p_next",    200u, 0u, 0u));

/* ==================================================================== */
/* [4] 取回子表：从粗加工区取（不用视觉，位置由标定表给出）              */
/* ==================================================================== */
enum
{
  RL_BEGIN = 0,
  RL_MOVE,
  RL_DOWN,
  RL_GRIP,
  RL_LIFT,
  RL_TOCAR,
  RL_NEXT,
  RL_N
};

static Result rl_begin(Seq *s, Ctx *c)
{
  (void)s;
  c->item = 0u;
  return RES_OK;
}

static Result rl_move(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    VisBias zero = { 0.0f, 0.0f, 0.0f };
    P->arm_move(c->zone, RingOf(c), &zero, 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result rl_down(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->arm_descend(c->zone, RingOf(c), 0u);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result rl_grip(Seq *s, Ctx *c)
{
  (void)c;
  if (s->steps[s->idx].phase == 0u)
  {
    P->grip_close();
    s->steps[s->idx].phase = 1u;
    return RES_RUNNING;
  }
  if (P->arm_state() == 0u)
  {
    return RES_RUNNING;
  }
  if (P->grip_state() == 2u)
  {
    s->steps[s->idx].xi[0] = 1;        /* 夹空：记账后跳过 */
  }
  return RES_OK;
}

static Result rl_lift(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  (void)c;
  if (st->phase == 0u)
  {
    P->arm_lift(30.0f);
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->arm_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result rl_tocar(Seq *s, Ctx *c)
{
  Step *st = &s->steps[s->idx];

  if (st->phase == 0u)
  {
    P->grip_open();
    st->phase = 1u;
    return RES_RUNNING;
  }
  if (P->arm_state() == 0u)
  {
    return RES_RUNNING;
  }
  if (st->xi[0] == 0)
  {
    c->reload_ok++;                    /* 与"从原料区抓取"分开计 */
  }
  return RES_OK;
}

static Result rl_next(Seq *s, Ctx *c)
{
  c->item++;
  if (c->item < ITEM_N)
  {
    Seq_JumpTo(s, RL_MOVE);
  }
  return RES_OK;
}

SEQ_DEF(seq_reload,
        STEP(rl_begin, "r_begin",  200u, 0u, 0u),
        STEP(rl_move,  "r_move",   900u, 0u, 0u),
        STEP(rl_down,  "r_down",   500u, 0u, 0u),
        STEP(rl_grip,  "r_grip",   500u, 1u, 0u),
        STEP(rl_lift,  "r_lift",   600u, 0u, 0u),
        STEP(rl_tocar, "r_tocar",  600u, 0u, 0u),
        STEP(rl_next,  "r_next",   200u, 0u, 0u));

/* ==================================================================== */
/* [5] 顶层：19 步                                                      */
/* ==================================================================== */
enum
{
  M_NAV_QR = 0,
  M_READ_QR,
  M_SHOW,
  M_NAV_RAW1,
  M_PICK1,
  M_NAV_ROUGH1,
  M_PLACE1,
  M_RELOAD1,
  M_NAV_TEMP1,
  M_PLACE_T1,
  M_NAV_RAW2,
  M_PICK2,
  M_NAV_ROUGH2,
  M_PLACE2,
  M_RELOAD2,
  M_NAV_TEMP2,
  M_STACK_T2,
  M_NAV_HOME,
  M_DONE,
  M_N
};

static ZoneId NavZoneOf(uint8_t idx)
{
  switch (idx)
  {
    case M_NAV_QR:     return Z_QR;
    case M_NAV_RAW1:
    case M_NAV_RAW2:   return Z_RAW;
    case M_NAV_ROUGH1:
    case M_NAV_ROUGH2: return Z_ROUGH;
    case M_NAV_TEMP1:
    case M_NAV_TEMP2:  return Z_TEMP;
    case M_NAV_HOME:
    default:           return Z_HOME;
  }
}

static uint32_t NavTmoOf(uint8_t idx)
{
  switch (idx)
  {
    case M_NAV_HOME: return 10000u;
    case M_NAV_QR:   return 7000u;
    default:         return 8000u;
  }
}

static Result m_nav(Seq *s, Ctx *c)
{
  Step  *st = &s->steps[s->idx];
  ZoneId z;

  (void)c;
  if (st->phase == 0u)
  {
    z = NavZoneOf(s->idx);
    P->nav_goto(k_zone[z].x, k_zone[z].y, k_zone[z].yaw, NavTmoOf(s->idx));
    st->phase = 1u;
    return RES_RUNNING;
  }
  return (P->nav_state() == 0u) ? RES_RUNNING : RES_OK;
}

static Result m_read_qr(Seq *s, Ctx *c)
{
  Step   *st = &s->steps[s->idx];
  uint8_t i;

  if (st->phase == 0u)
  {
    if (!P->vis_code_start())
    {
      return RES_FAIL;                 /* 交给框架的重试 */
    }
    st->phase = 1u;
  }
  if (P->vis_code_poll(c->task_str))
  {
    for (i = 0u; i < TEAM_TASK_N; i++)
    {
      c->task[i][0] = (int8_t)(c->task_str[(i * 4u) + 0u] - '0');
      c->task[i][1] = (int8_t)(c->task_str[(i * 4u) + 1u] - '0');
      c->task[i][2] = (int8_t)(c->task_str[(i * 4u) + 2u] - '0');
    }
    c->task_ok = 1u;
    c->vision_used++;                  /* 读码计 1 次（重试不重复计） */
    return RES_OK;
  }
  return RES_RUNNING;
}

static Result m_show(Seq *s, Ctx *c)
{
  (void)s;
  P->disp_code((c->task_ok != 0u) ? c->task_str : "CODE:FAIL");
  P->disp_counts(c->pick_ok, c->place_ok);
  return RES_OK;
}

/* ---- 子表通用跑法：reset + run + 异常隔离 ---- */
static Result RunSub(Seq *s, Ctx *c, Seq *sub, ZoneId zone,
                     uint16_t *fail_cnt, uint8_t act_base)
{
  Result r;

  if (s->steps[s->idx].phase == 0u)
  {
    Seq_Reset(sub);
    c->zone     = zone;
    c->act_base = act_base;
    s->steps[s->idx].phase = 1u;
  }

  r = Seq_Run(sub, c);

  if (r == RES_RUNNING)
  {
    return RES_RUNNING;
  }
  if (r != RES_OK)
  {
    /* 子机异常收尾：记账，但让顶层继续走——绝不让一个环节拖垮整场 */
    P->log("[M] sub(%u) r=%u last=%u\r\n", (unsigned)zone, (unsigned)r,
           (unsigned)Seq_LastResult(sub));
    if (fail_cnt != NULL) { (*fail_cnt)++; }
  }
  return RES_OK;
}

/* 视觉槽位基址：拿取 0~5，放置 6~17（共 18 次"要视觉的动作"，再加 1 次读码 = 19） */
#define ACT_PICK1   0u
#define ACT_PICK2   3u
#define ACT_PLACE1  6u
#define ACT_PLACE_T1 9u
#define ACT_PLACE2  12u
#define ACT_STACK_T2 15u

static Result m_pick1(Seq *s, Ctx *c)   { return RunSub(s, c, &seq_pick,   Z_RAW,   &c->pick_fail,   ACT_PICK1); }
static Result m_pick2(Seq *s, Ctx *c)   { return RunSub(s, c, &seq_pick,   Z_RAW,   &c->pick_fail,   ACT_PICK2); }
static Result m_place1(Seq *s, Ctx *c)  { return RunSub(s, c, &seq_place,  Z_ROUGH, &c->place_fail,  ACT_PLACE1); }
static Result m_place2(Seq *s, Ctx *c)  { return RunSub(s, c, &seq_place,  Z_ROUGH, &c->place_fail,  ACT_PLACE2); }
static Result m_place_t1(Seq *s, Ctx *c){ return RunSub(s, c, &seq_place,  Z_TEMP,  &c->place_fail,  ACT_PLACE_T1); }
static Result m_stack_t2(Seq *s, Ctx *c){ return RunSub(s, c, &seq_place,  Z_STACK, &c->place_fail,  ACT_STACK_T2); }
static Result m_reload1(Seq *s, Ctx *c) { return RunSub(s, c, &seq_reload, Z_ROUGH, &c->pick_fail,   0u); }
static Result m_reload2(Seq *s, Ctx *c) { return RunSub(s, c, &seq_reload, Z_ROUGH, &c->pick_fail,   0u); }

static Result m_done(Seq *s, Ctx *c)
{
  (void)s;
  P->chassis_stop();
  P->disp_code((c->task_ok != 0u) ? c->task_str : "CODE:FAIL");
  P->disp_counts(c->pick_ok, c->place_ok);
  P->disp_msg("DONE");
  s_done = 1u;
  P->log("[M] === 结束：抓 %u 取回 %u 放 %u 失抓 %u 失放 %u 视觉 %u 用时 %lums ===\r\n",
         (unsigned)c->pick_ok, (unsigned)c->reload_ok, (unsigned)c->place_ok,
         (unsigned)c->pick_fail, (unsigned)c->place_fail,
         (unsigned)c->vision_used,
         (unsigned long)(HAL_GetTick() - c->t_start));
  return RES_OK;
}

SEQ_DEF(seq_mission,
        STEP(m_nav,      "nav_qr",     7000u, 1u, 0u),
        STEP(m_read_qr,  "read_qr",    5000u, 1u, STEP_OPTIONAL),
        STEP(m_show,     "show_code",   500u, 0u, STEP_OPTIONAL),
        STEP(m_nav,      "nav_raw1",   8000u, 1u, 0u),
        STEP(m_pick1,    "pick_1",    12000u, 0u, 0u),
        STEP(m_nav,      "nav_rough1", 8000u, 1u, 0u),
        STEP(m_place1,   "place_1",   14000u, 0u, 0u),
        STEP(m_reload1,  "reload_1",  12000u, 0u, 0u),
        STEP(m_nav,      "nav_temp1",  7000u, 1u, 0u),
        STEP(m_place_t1, "place_t1",  14000u, 0u, 0u),
        STEP(m_nav,      "nav_raw2",   8000u, 1u, 0u),
        STEP(m_pick2,    "pick_2",    12000u, 0u, 0u),
        STEP(m_nav,      "nav_rough2", 8000u, 1u, 0u),
        STEP(m_place2,   "place_2",   14000u, 0u, 0u),
        STEP(m_reload2,  "reload_2",  12000u, 0u, 0u),
        STEP(m_nav,      "nav_temp2",  7000u, 1u, 0u),
        STEP(m_stack_t2, "stack_t2",  16000u, 0u, 0u),
        STEP(m_nav,      "nav_home",  10000u, 1u, 0u),
        STEP(m_done,     "done",        500u, 0u, STEP_OPTIONAL));

/* ==================================================================== */
/* [6] 检查点与降级                                                     */
/* ==================================================================== */
static void Mission_Checkpoint(Ctx *c, uint32_t plan_ms, const char *name)
{
  uint32_t el = HAL_GetTick() - c->t_start;

  P->log("[M] checkpoint %s: %lums (计划 %lums)\r\n",
         name, (unsigned long)el, (unsigned long)plan_ms);

  if ((el > (plan_ms + LATE_TOLERANCE_MS)) && (c->level < LVL_NO_VERIFY))
  {
    c->level = LVL_NO_VERIFY;          /* L1：取消放置后确认、减少重试 */
    P->log("[M] !! 落后，降级到 L1（取消回看）\r\n");
  }
}

/* 硬超时保护：到点直接回启停区，保住"回位 2 分" */
static void Mission_BudgetGuard(void)
{
  uint32_t el;

  if (s_started == 0u)
  {
    return;
  }
  if (seq_mission.idx >= M_NAV_HOME)
  {
    return;                            /* 已经在回程，不再干预 */
  }
  el = HAL_GetTick() - s_ctx.t_start;
  if (el > MISSION_HARD_ABORT_MS)
  {
    s_ctx.level = LVL_HOME_ONLY;
    Seq_JumpTo(&seq_mission, M_NAV_HOME);
    P->log("[M] !! 硬超时 %lums，放弃剩余任务直接回启停区\r\n", (unsigned long)el);
  }
}

/* ==================================================================== */
/* [7] 对外入口                                                         */
/* ==================================================================== */
void Mission_Init(void)
{
  uint8_t i, j;

  for (i = 0u; i < sizeof(s_ctx); i++)
  {
    ((uint8_t *)&s_ctx)[i] = 0u;
  }
  for (i = 0u; i < TEAM_TASK_N; i++)
  {
    for (j = 0u; j < RING_N; j++)
    {
      s_ctx.task[i][j] = 1;            /* 上电默认全 1 号环，防止未读码时除零/越界 */
    }
  }

  Frame_Init(Adapt_GetPorts());
  Seq_Reset(&seq_mission);
  Seq_Reset(&seq_place);
  Seq_Reset(&seq_pick);
  Seq_Reset(&seq_reload);
  Seq_SetId(&seq_mission, 1u);
  Seq_SetId(&seq_place,   2u);
  Seq_SetId(&seq_pick,    3u);
  Seq_SetId(&seq_reload,  4u);

  s_started = 0u;
  s_done    = 0u;
  s_boot_ms = HAL_GetTick();

  P->log("[M] 框架就绪：%u 步顶层 + 3 张子表，%s 模式\r\n",
         (unsigned)M_N,
         (g_adapt_motion_real != 0u) ? "真实动作" : "仿真动作");
}

void Mission_Start(void)
{
  if (s_started != 0u)
  {
    return;                            /* 只有一次启动机会 */
  }

  s_ctx.t_start = HAL_GetTick();
  s_ctx.level   = LVL_NORMAL;
  Seq_Reset(&seq_mission);
  Seq_Reset(&seq_place);
  Seq_Reset(&seq_pick);
  Seq_Reset(&seq_reload);

  /* ★关掉"自动回上电原点"，否则会和导航抢控制权 */
  Chassis_SetAutoHome(0u);

  s_started = 1u;
  s_done    = 0u;

  P->log("[M] === 启动（%lums）===\r\n", (unsigned long)s_ctx.t_start);
}

void Mission_Task(void)
{
  Result r;

#if (MISSION_AUTOSTART != 0)
  if ((s_started == 0u) && ((HAL_GetTick() - s_boot_ms) > 3000u))
  {
    Mission_Start();
  }
#endif

  if ((s_started == 0u) || (s_done != 0u))
  {
    return;
  }

  Frame_Task(&s_ctx);                  /* 存活看护 / 降级提升 */
  Mission_BudgetGuard();               /* 硬超时 → 强制回程 */

  r = Seq_Run(&seq_mission, &s_ctx);

  /* 检查点（在对应步骤完成时触发一次） */
  switch (Seq_Index(&seq_mission))
  {
    case M_NAV_RAW1:   Mission_Checkpoint(&s_ctx, PLAN_CHECK_1_MS, "C1");  break;
    case M_NAV_RAW2:   Mission_Checkpoint(&s_ctx, PLAN_CHECK_2_MS, "C2");  break;
    case M_NAV_TEMP2:  Mission_Checkpoint(&s_ctx, PLAN_CHECK_3_MS, "C3");  break;
    case M_NAV_HOME:   Mission_Checkpoint(&s_ctx, PLAN_CHECK_4_MS, "C4");  break;
    default: break;
  }

  if (r != RES_RUNNING)
  {
    if (r != RES_OK)
    {
      P->log("[M] !! 顶层中止 r=%u (step=%u %s)\r\n",
             (unsigned)r, (unsigned)Seq_Index(&seq_mission),
             Seq_StepName(&seq_mission));
      P->chassis_stop();
      Chassis_Stop();
    }
    s_done = 1u;
  }
}

uint8_t Mission_IsRunning(void)
{
  return ((s_started != 0u) && (s_done == 0u)) ? 1u : 0u;
}

const Ctx *Mission_GetCtx(void)
{
  return &s_ctx;
}

uint8_t Mission_GetStep(void)
{
  return Seq_Index(&seq_mission);
}

const char *Mission_GetStepName(void)
{
  return Seq_StepName(&seq_mission);
}
