#include "app_frame.h"
#include <stddef.h>

const Ports *P = NULL;

static uint32_t s_last_feed;
static uint8_t  s_kick_ok = 1u;

static TraceRec s_trace[TRACE_DEPTH];
static uint8_t  s_trace_n    = 0u;
static uint8_t  s_trace_head = 0u;

/* ==================== 初始化与时钟 ==================== */

void Frame_Init(const Ports *ports)
{
  P = ports;
  s_last_feed = (ports != NULL && ports->now_ms != NULL) ? ports->now_ms() : 0u;
  s_kick_ok    = 1u;
  s_trace_n    = 0u;
  s_trace_head = 0u;
}

uint32_t Frame_Now(void)
{
  return (P != NULL && P->now_ms != NULL) ? P->now_ms() : 0u;
}

void Frame_Feed(void)
{
  s_last_feed = Frame_Now();
}

void Frame_SetKick(bool allow)
{
  s_kick_ok = allow ? 1u : 0u;
}

/* 进入某一步：清私有状态、重置计时。重试时也会走这里 */
static void StepEnter(Step *st, uint32_t now)
{
  st->phase   = 0u;
  st->t_enter = now;
  st->xi[0]   = 0;
  st->xi[1]   = 0;
  st->xf[0]   = 0.0f;
  st->xf[1]   = 0.0f;
}

/* ==================== 单步推进 ==================== */

static Result RunOnce(Seq *s, Ctx *c)
{
  Step    *st;
  uint32_t now, tmo;
  Result   r;
  int16_t  nxt;

  st  = &s->steps[s->idx];
  now = Frame_Now();

  if (s->active == 0u)
  {
    StepEnter(st, now);
    s->active = 1u;
  }

  tmo = (st->timeout_ms != 0u) ? (uint32_t)st->timeout_ms : FRAME_DEF_TIMEOUT_MS;
  if ((now - st->t_enter) > tmo)
  {
    r = RES_TIMEOUT;
  }
  else
  {
    r = st->fn(s, c);
  }

  if (r == RES_RUNNING)
  {
    return RES_RUNNING;
  }

  /* 重试：重新进入同一步 */
  if (((r == RES_FAIL) || (r == RES_TIMEOUT)) && (st->tries < st->retry))
  {
    st->tries++;
    StepEnter(st, now);
    return RES_RUNNING;
  }

  /* 允许降级的步骤：失败当完成 */
  if (((r == RES_FAIL) || (r == RES_TIMEOUT)) &&
      ((st->flags & STEP_OPTIONAL) != 0u))
  {
    r = RES_OK;
  }

  s->last_res = (uint8_t)r;

  if ((st->flags & STEP_NO_TRACE) == 0u)
  {
    Trace_Push(s->id, s->idx, (uint8_t)r);
  }
  Frame_Feed();                      /* 状态推进也算"有动作" */

  if (r != RES_OK)
  {
    return r;
  }

  nxt        = (s->pending >= 0) ? s->pending : (int16_t)(s->idx + 1);
  s->pending = -1;
  s->active  = 0u;
  s->idx     = (uint8_t)nxt;

  return (s->idx >= s->n) ? RES_OK : RES_RUNNING;
}

Result Seq_Run(Seq *s, Ctx *c)
{
  uint8_t guard = 0u;
  Result  r     = RES_RUNNING;

  if ((s == NULL) || (s->n == 0u))
  {
    return RES_FAIL;
  }
  if (s->idx >= s->n)
  {
    return RES_OK;
  }

  while (guard < FRAME_STEPS_PER_TICK)
  {
    guard++;
    r = RunOnce(s, c);
    if (r != RES_RUNNING)
    {
      return r;
    }
    if (s->active != 0u)             /* 当前步还在跑，等下一 tick */
    {
      return RES_RUNNING;
    }
  }
  return RES_RUNNING;
}

void Seq_Reset(Seq *s)
{
  uint8_t i;

  if (s == NULL)
  {
    return;
  }
  for (i = 0u; i < s->n; i++)
  {
    s->steps[i].phase   = 0u;
    s->steps[i].tries   = 0u;
    s->steps[i].t_enter = 0u;
    s->steps[i].xi[0]   = 0;
    s->steps[i].xi[1]   = 0;
    s->steps[i].xf[0]   = 0.0f;
    s->steps[i].xf[1]   = 0.0f;
  }
  s->idx      = 0u;
  s->active   = 0u;
  s->pending  = -1;
  s->last_res = (uint8_t)RES_OK;
  s->t_seq    = Frame_Now();
}

void Seq_SetId(Seq *s, uint8_t id)
{
  if (s != NULL) { s->id = id; }
}

Result Seq_LastResult(const Seq *s)
{
  return (s != NULL) ? (Result)s->last_res : RES_OK;
}

uint8_t Seq_Index(const Seq *s)
{
  return (s != NULL) ? s->idx : 0xFFu;
}

const char *Seq_StepName(const Seq *s)
{
  if ((s == NULL) || (s->idx >= s->n))
  {
    return "-";
  }
  return s->steps[s->idx].name;
}

void Seq_JumpTo(Seq *s, uint8_t idx)
{
  if ((s != NULL) && (idx < s->n))
  {
    s->pending = (int16_t)idx;
  }
}

/* ==================== 存活（15 s 红线） ==================== */

void Frame_Task(Ctx *c)
{
  uint32_t starved;

  if (P == NULL)
  {
    return;
  }

  starved = Frame_Now() - s_last_feed;

  if (starved > FRAME_STARVE_ABORT_MS)
  {
    P->estop(0x0Bu);
    return;
  }

  if ((starved > FRAME_STARVE_KICK_MS) && (s_kick_ok != 0u))
  {
    if (P->liveness_kick != NULL)
    {
      P->liveness_kick();
    }
    Frame_Feed();
  }

  /* 位姿不可信 → 提升降级等级（只升不降） */
  if ((c != NULL) && (P->pose_bad != NULL) && (P->pose_bad() != 0u))
  {
    if (c->level < LVL_HOME_ONLY)
    {
      c->level = LVL_HOME_ONLY;
    }
  }
}

/* ==================== 追踪 ==================== */

void Trace_Push(uint8_t seq_id, uint8_t step, uint8_t res)
{
  s_trace[s_trace_head].t_ms   = Frame_Now();
  s_trace[s_trace_head].seq_id = seq_id;
  s_trace[s_trace_head].step   = step;
  s_trace[s_trace_head].res    = res;

  s_trace_head = (uint8_t)((s_trace_head + 1u) % TRACE_DEPTH);
  if (s_trace_n < TRACE_DEPTH)
  {
    s_trace_n++;
  }
}

void Trace_Dump(void)
{
  uint8_t i, p;

  if ((P == NULL) || (P->log == NULL))
  {
    return;
  }
  for (i = 0u; i < s_trace_n; i++)
  {
    p = (uint8_t)((s_trace_head + TRACE_DEPTH - s_trace_n + i) % TRACE_DEPTH);
    P->log("[TR] t=%lu r=%u s=%u/%u\r\n",
           (unsigned long)s_trace[p].t_ms,
           (unsigned)s_trace[p].res,
           (unsigned)s_trace[p].seq_id,
           (unsigned)s_trace[p].step);
  }
}
