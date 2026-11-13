/**
  ******************************************************************************
  * @file    adapt_ports.c
  * @brief   端口适配器实现
  *
  *   职责边界：
  *     业务层（app_mission.c）只认 app_ports.h；
  *     本文件是"业务层 → 真实驱动"的唯一桥。想换驱动只改这里。
  *
  *   当前接好了的（真实可用）：
  *     now_ms       → HAL_GetTick()
  *     log          → Debug_Write（UART5，115200）
  *     disp_*       → 暂用 UART5 打印代替 TFT（屏还没装）
  *     chassis_stop → Chassis_Stop()
  *     grip_*       → ServoBus_SetAngle()（仅 motion_real 模式）
  *     feed_motion  → Frame_Feed()
  *     pose_bad     → 里程计/陀螺离线检测（仅 motion_real 模式）
  *     estop        → Chassis_Stop() + 置故障位
  *     vis_*        → 仿真：立即返回；真实：TODO
  *   还没接的（真实模式下只打印 TODO，不动执行器）：
  *     nav_goto / nav_state      → 等 app_nav 写出来
  *     arm_move / arm_descend / arm_lift / arm_state → 等臂逆解写出来
  ******************************************************************************
  */
#include "adapt_ports.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "app_frame.h"   /* Frame_Feed() */
#include "app_ctx.h"     /* BATCH_ITEMS */
#include "bsp_debug.h"
#include "app_odom.h"
#include "app_chassis.h"
#include "hwt101.h"
#include "servo_bus.h"

/* ==================== 调试开关 ==================== */
volatile uint8_t  g_adapt_motion_real = 0u;   /* ★0=仿真动作，先跑流程 */
volatile uint8_t  g_adapt_verbose     = 1u;
volatile uint8_t  g_adapt_fault       = 0u;

volatile uint32_t g_adapt_nav_cnt = 0u;
volatile uint32_t g_adapt_arm_cnt = 0u;
volatile uint32_t g_adapt_vis_cnt = 0u;
volatile uint32_t g_adapt_vis_dup = 0u;

volatile float g_adapt_sim_x   = 0.0f;
volatile float g_adapt_sim_y   = 0.0f;
volatile float g_adapt_sim_yaw = 0.0f;

/* ==================== 内部状态 ==================== */
#define ADAPT_SIM_SPEED_MMPS   250.0f   /* 仿真行驶速度 */
#define ADAPT_SIM_NAV_MARGIN   150u     /* 仿真到点附加耗时 */
#define ADAPT_ARM_MOVE_MS      800u
#define ADAPT_ARM_DOWN_MS      400u
#define ADAPT_ARM_LIFT_MS      300u
#define ADAPT_GRIP_MS          250u
#define ADAPT_VIS_MS           120u

static uint32_t s_nav_end   = 0u;
static uint8_t  s_nav_res   = 1u;
static uint32_t s_arm_end   = 0u;
static uint8_t  s_arm_res   = 1u;
static uint32_t s_vis_end   = 0u;
static uint8_t  s_vis_ready = 0u;
static uint8_t  s_vis_kind  = 0u;      /* 1=code 2=bias 3=verify */

/* 视觉台账：每个"要视觉的动作"一个唯一槽位（0..17），用一个挡一个 */
#define ADAPT_ACT_MAX   24u
static uint8_t s_bias_used[ADAPT_ACT_MAX];

/* ==================== 基础端口 ==================== */

static uint32_t adapt_now(void)
{
  return HAL_GetTick();
}

static void adapt_log(const char *fmt, ...)
{
  static char buf[128];
  va_list ap;
  int n;

  if (fmt == NULL)
  {
    return;
  }
  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }
  if (n >= (int)sizeof(buf))
  {
    n = (int)sizeof(buf) - 1;
  }
  Debug_Write((const uint8_t *)buf, (uint16_t)n);
}

/* ==================== 底盘 / 导航 ==================== */

static void adapt_nav_goto(float x_mm, float y_mm, float yaw_deg, uint32_t tmo_ms)
{
  float d;
  uint32_t dur;

  g_adapt_nav_cnt++;

  /* 仿真：按距离估耗时，到点后把仿真位姿跳到目标 */
  d   = (x_mm - g_adapt_sim_x);
  d   = (d < 0.0f) ? -d : d;
  {
    float dy = (y_mm - g_adapt_sim_y);
    d += (dy < 0.0f) ? -dy : dy;
  }
  dur = (uint32_t)((d / ADAPT_SIM_SPEED_MMPS) * 1000.0f) + ADAPT_SIM_NAV_MARGIN;
  if (dur > tmo_ms)
  {
    dur = tmo_ms;                       /* 仿真也要遵守超时 */
  }
  s_nav_end = HAL_GetTick() + dur;
  s_nav_res = 1u;

  if (g_adapt_verbose != 0u)
  {
    adapt_log("[NAV] -> (%.0f,%.0f,%.0f) %s dur=%lums\r\n",
              (double)x_mm, (double)y_mm, (double)yaw_deg,
              (g_adapt_motion_real != 0u) ? "REAL" : "SIM",
              (unsigned long)dur);
  }

  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 接 app_nav —— Nav_Goto(x_mm, y_mm, yaw_deg, tmo_ms);
     *       在 app_nav 写出来之前，真实模式下也按仿真计时完成，绝不动车。*/
    adapt_log("[NAV] TODO: app_nav 未实现，本次按仿真完成（车不会动）\r\n");
  }
}

static uint8_t adapt_nav_state(void)
{
  if (HAL_GetTick() >= s_nav_end)
  {
    /* 仿真到位：更新仿真位姿（真实模式下由 app_nav 自己维护位姿） */
    if (g_adapt_motion_real == 0u)
    {
      /* 位姿由最后一次 nav_goto 的目标决定，这里不重复计算，保持静止即可 */
    }
    return s_nav_res;
  }
  return 0u;
}

static void adapt_chassis_stop(void)
{
  Chassis_Stop();
}

static void adapt_pose_get(float *x_mm, float *y_mm, float *yaw_deg)
{
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 接 app_pose —— 现在用体坐标里程计近似（**不是世界坐标**，仅供参考） */
    if (x_mm)    { *x_mm = AppOdom_GetX(); }
    if (y_mm)    { *y_mm = AppOdom_GetY(); }
    if (yaw_deg) { *yaw_deg = HWT101_GetYaw(); }
  }
  else
  {
    if (x_mm)    { *x_mm = g_adapt_sim_x; }
    if (y_mm)    { *y_mm = g_adapt_sim_y; }
    if (yaw_deg) { *yaw_deg = g_adapt_sim_yaw; }
  }
}

static uint8_t adapt_pose_bad(void)
{
  if (g_adapt_motion_real == 0u)
  {
    return 0u;                          /* 仿真：永远认为位姿可用 */
  }
  /* 真实：任一路里程计离线，或陀螺离线 → 位姿不可信 */
  if ((g_odom_online[0] == 0u) || (g_odom_online[1] == 0u))
  {
    return 1u;
  }
  if (!HWT101_IsOnline())
  {
    return 1u;
  }
  return 0u;
}

/* ==================== 机械臂 ==================== */

static void adapt_arm_finish_later(uint32_t ms)
{
  s_arm_end = HAL_GetTick() + ms;
  s_arm_res = 1u;
}

static void adapt_arm_move(ZoneId zone, int8_t ring, const VisBias *bias, uint8_t stack)
{
  g_adapt_arm_cnt++;
  if (g_adapt_verbose != 0u)
  {
    adapt_log("[ARM] move zone=%u ring=%d bias=(%.1f,%.1f) stack=%u %s\r\n",
              (unsigned)zone, (int)ring,
              (bias != NULL) ? (double)bias->dx : 0.0,
              (bias != NULL) ? (double)bias->dy : 0.0,
              (unsigned)stack,
              (g_adapt_motion_real != 0u) ? "REAL" : "SIM");
  }
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 接臂逆解 —— Arm_MoveTo(标定表[zone][ring] + bias) */
    adapt_log("[ARM] TODO: 臂逆解未实现，按仿真完成（臂不会动）\r\n");
  }
  adapt_arm_finish_later(ADAPT_ARM_MOVE_MS);
}

static void adapt_arm_descend(ZoneId zone, int8_t ring, uint8_t stack)
{
  g_adapt_arm_cnt++;
  if (g_adapt_verbose != 0u)
  {
    adapt_log("[ARM] descend zone=%u ring=%d stack=%u\r\n",
              (unsigned)zone, (int)ring, (unsigned)stack);
  }
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 垂直下放（只走 Z，禁止水平分量） */
  }
  adapt_arm_finish_later(ADAPT_ARM_DOWN_MS);
}

static void adapt_arm_lift(float mm)
{
  g_adapt_arm_cnt++;
  if (g_adapt_verbose != 0u)
  {
    adapt_log("[ARM] lift %.1fmm\r\n", (double)mm);
  }
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 抬起到安全高度 */
  }
  adapt_arm_finish_later(ADAPT_ARM_LIFT_MS);
}

static void adapt_grip_close(void)
{
  if (g_adapt_motion_real != 0u)
  {
    /* 夹爪按总线舵机接（ID 见 servo_bus.h SERVO_ID_GRIPPER） */
    (void)ServoBus_SetAngle(SERVO_ID_GRIPPER, 120.0f, 0x80u);
  }
  adapt_arm_finish_later(ADAPT_GRIP_MS);
}

static void adapt_grip_open(void)
{
  if (g_adapt_motion_real != 0u)
  {
    (void)ServoBus_SetAngle(SERVO_ID_GRIPPER, 60.0f, 0x80u);
  }
  adapt_arm_finish_later(ADAPT_GRIP_MS);
}

static uint8_t adapt_grip_state(void)
{
  /* TODO: 接夹空判定（行程开关 / 负压 / 光电）。
   * 在做出判定之前，一律返回"夹紧"，避免误触发降级。*/
  return 1u;
}

static uint8_t adapt_arm_state(void)
{
  return (HAL_GetTick() >= s_arm_end) ? s_arm_res : 0u;
}

/* ==================== 视觉（含 C1 台账） ==================== */

static uint8_t adapt_vis_begin(uint8_t kind)
{
  g_adapt_vis_cnt++;
  s_vis_kind  = kind;
  s_vis_end   = HAL_GetTick() + ADAPT_VIS_MS;
  s_vis_ready = 1u;
  return 1u;
}

static bool adapt_vis_code_start(void)
{
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 接视觉模块 —— Vision_RequestCode() */
  }
  return (adapt_vis_begin(1u) != 0u);
}

static bool adapt_vis_code_poll(char out[20])
{
  if ((s_vis_ready == 0u) || (s_vis_kind != 1u) || (HAL_GetTick() < s_vis_end))
  {
    return false;
  }
  s_vis_ready = 0u;
  /* 仿真：给一个固定任务码，便于反复验证流程 */
  memcpy(out, "156+123+516+231", 16u);
  if (g_adapt_verbose != 0u)
  {
    adapt_log("[VIS] code = %s\r\n", out);
  }
  return true;
}

static bool adapt_vis_bias_start(ZoneId zone, uint8_t act)
{
  if (act >= ADAPT_ACT_MAX)
  {
    g_adapt_vis_dup++;
    adapt_log("[VIS] !! C1 违规：act=%u 越界，已拒绝\r\n", (unsigned)act);
    return false;
  }
  /* ★C1 强制：同一个动作槽位只给一次视觉 */
  if (s_bias_used[act] != 0u)
  {
    g_adapt_vis_dup++;
    adapt_log("[VIS] !! C1 违规：zone=%u act=%u 第二次请求，已拒绝\r\n",
              (unsigned)zone, (unsigned)act);
    return false;
  }
  s_bias_used[act] = 1u;
  return (adapt_vis_begin(2u) != 0u);
}

static bool adapt_vis_bias_poll(VisBias *out)
{
  if ((s_vis_ready == 0u) || (s_vis_kind != 2u) || (HAL_GetTick() < s_vis_end))
  {
    return false;
  }
  s_vis_ready = 0u;
  if (out != NULL)
  {
    out->dx     = 0.0f;                 /* 仿真：无偏置 */
    out->dy     = 0.0f;
    out->eta_ms = 0.0f;
  }
  return true;
}

static bool adapt_vis_verify_start(ZoneId zone, int8_t ring)
{
  (void)zone;
  (void)ring;
  return (adapt_vis_begin(3u) != 0u);
}

static bool adapt_vis_verify_poll(uint8_t *ok)
{
  if ((s_vis_ready == 0u) || (s_vis_kind != 3u) || (HAL_GetTick() < s_vis_end))
  {
    return false;
  }
  s_vis_ready = 0u;
  if (ok != NULL)
  {
    *ok = 1u;                           /* 仿真：永远"放好了" */
  }
  return true;
}

/* ==================== 显示（暂用串口代替 TFT） ==================== */

static void adapt_disp_code(const char *code)
{
  adapt_log("[DISP] CODE: %s\r\n", (code != NULL) ? code : "----");
}

static void adapt_disp_counts(uint16_t pick, uint16_t place)
{
  adapt_log("[DISP] pick=%u place=%u\r\n", (unsigned)pick, (unsigned)place);
}

static void adapt_disp_msg(const char *msg)
{
  adapt_log("[DISP] %s\r\n", (msg != NULL) ? msg : "");
}

/* ==================== 存活与安全 ==================== */

static void adapt_feed_motion(void)
{
  Frame_Feed();
}

static void adapt_liveness_kick(void)
{
  if (g_adapt_verbose != 0u)
  {
    adapt_log("[LIVE] kick（8s 无动作，做一次可见小动作）\r\n");
  }
  if (g_adapt_motion_real != 0u)
  {
    /* TODO: 真实模式下在这里做一次 3mm 往返 / 臂抬 5mm */
  }
}

static void adapt_estop(uint8_t code)
{
  g_adapt_fault = 1u;
  Chassis_Stop();
  adapt_log("[ESTOP] code=%u（已停车）\r\n", (unsigned)code);
}

/* ==================== 端口表 ==================== */

static const Ports s_ports =
{
  adapt_now,
  adapt_log,
  adapt_nav_goto,
  adapt_nav_state,
  adapt_chassis_stop,
  adapt_pose_get,
  adapt_pose_bad,
  adapt_arm_move,
  adapt_arm_descend,
  adapt_arm_lift,
  adapt_grip_close,
  adapt_grip_open,
  adapt_grip_state,
  adapt_arm_state,
  adapt_vis_code_start,
  adapt_vis_code_poll,
  adapt_vis_bias_start,
  adapt_vis_bias_poll,
  adapt_vis_verify_start,
  adapt_vis_verify_poll,
  adapt_disp_code,
  adapt_disp_counts,
  adapt_disp_msg,
  adapt_feed_motion,
  adapt_liveness_kick,
  adapt_estop
};

const Ports *Adapt_GetPorts(void)
{
  return &s_ports;
}

void Adapt_Task(void)
{
  /* 目前无需周期处理；留给以后（例如低电压监测、看门狗喂狗） */
}
