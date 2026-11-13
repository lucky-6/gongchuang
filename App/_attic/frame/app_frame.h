#ifndef APP_FRAME_H
#define APP_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include "app_ports.h"
#include "app_ctx.h"

/* 运行器可调参数 */
#define FRAME_DEF_TIMEOUT_MS   1000u
#define FRAME_STEPS_PER_TICK   4u
#define FRAME_STARVE_KICK_MS   8000u
#define FRAME_STARVE_ABORT_MS  20000u
#define TRACE_DEPTH            64u

typedef enum
{
  RES_RUNNING = 0,
  RES_OK,
  RES_FAIL,
  RES_TIMEOUT,
  RES_ABORTED
} Result;

/* 步骤标志 */
#define STEP_OPTIONAL   0x01u   /* 允许降级：失败/超时也当完成，继续 */
#define STEP_NO_KICK    0x02u   /* 此步禁止 liveness 抖动（精动作） */
#define STEP_NO_TRACE   0x04u   /* 不记追踪（高频步骤） */

typedef struct Seq Seq;

/** @brief 步骤函数：返回 RES_RUNNING 表示还要再来一拍 */
typedef Result (*StepFn)(Seq *s, Ctx *c);

typedef struct
{
  StepFn      fn;
  const char *name;
  uint16_t    timeout_ms;    /* 0 = FRAME_DEF_TIMEOUT_MS */
  uint8_t     retry;         /* 失败/超时重试次数 */
  uint8_t     flags;

  /* ---- 运行时区：框架维护，表里不用初始化 ---- */
  uint8_t     phase;
  uint8_t     tries;
  uint32_t    t_enter;
  int32_t     xi[2];         /* 跨拍整型私有状态 */
  float       xf[2];         /* 跨拍浮点私有状态 */
} Step;

struct Seq
{
  Step       *steps;
  uint8_t     n;
  const char *name;
  /* 运行时 */
  uint8_t     id;            /* 追踪编号 */
  uint8_t     idx;
  uint8_t     active;        /* 当前步已"进入"过 */
  int16_t     pending;       /* >=0 表示下一步跳到这里 */
  uint8_t     last_res;      /* 上一步的结束原因 */
  uint32_t    t_seq;
};

/* 一个步骤的完整初始化（含运行时字段，避免 -Wmissing-field-initializers） */
#define STEP(fn_, name_, tmo_, retry_, flags_)                               \
  { (fn_), (name_), (tmo_), (retry_), (flags_), 0u, 0u, 0u, { 0, 0 },       \
    { 0.0f, 0.0f } }

/* 声明一个 Sequence（步骤表 + 描述符） */
#define SEQ_DEF(var, ...)                                                    \
  static Step var##_steps[] = { __VA_ARGS__ };                               \
  static Seq  var = { var##_steps,                                           \
                      (uint8_t)(sizeof(var##_steps) / sizeof(Step)),         \
                      #var, 0u, 0u, 0u, -1, 0u, 0u }

/* ---------- 框架 API ---------- */
void        Frame_Init(const Ports *ports);
uint32_t    Frame_Now(void);
void        Frame_Feed(void);
void        Frame_SetKick(bool allow);
void        Frame_Task(Ctx *c);

Result      Seq_Run(Seq *s, Ctx *c);
void        Seq_Reset(Seq *s);
uint8_t     Seq_Index(const Seq *s);
const char *Seq_StepName(const Seq *s);
void        Seq_JumpTo(Seq *s, uint8_t idx);
void        Seq_SetId(Seq *s, uint8_t id);
Result      Seq_LastResult(const Seq *s);

/* 追踪 */
typedef struct
{
  uint32_t t_ms;
  uint8_t  seq_id;
  uint8_t  step;
  uint8_t  res;
} TraceRec;

void        Trace_Push(uint8_t seq_id, uint8_t step, uint8_t res);
void        Trace_Dump(void);

#endif /* APP_FRAME_H */
