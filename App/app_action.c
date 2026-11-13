/**
  ******************************************************************************
  * @file    app_action.c
  * @brief   原子动作层实现（无仿真，只走真实路径）
  *
  *   现状说明：
  *     · 位姿 / 夹爪 / 停车 / 日志 / 显示  —— 真实可用
  *     · 导航 / 机械臂 / 视觉              —— 底层模块还没有，这里是空壳，
  *                                            调了会打印"未实现"并返回 ACT_FAIL
  *
  *   补实现时直接改对应函数体即可，接口签名不用动。
  ******************************************************************************
  */
#include "app_action.h"

#include <stdarg.h>
#include <stdio.h>
#include <math.h>

#include "main.h"
#include "bsp_debug.h"
#include "app_pose.h"
#include "app_chassis.h"
#include "hwt101.h"
#include "servo_bus.h"

/* ==================== 调试开关 ==================== */
volatile uint8_t g_act_verbose = 1u;

/* ==================== 内部状态 ==================== */
static uint8_t s_grip = 0u;          /* 0 = 松开，1 = 夹紧（当前为"已下发的指令"） */

/* 上一次下发的关节目标角度，用于"没变就不重发"和到位判定 */
static float   s_arm_cmd[ACT_ARM_NUM];
static uint8_t s_arm_cmd_valid = 0u;

/* ==================== 关节机械回差补偿 ====================
 *
 * 现象：到达位置后，反向移动（或用手推）会有一段"旷量"，输出轴要等间隙
 *       吃掉才开始动，于是每次反向都会少走一个间隙量、停在偏一点的位置。
 *
 * 补偿原理：反向运动时，电机先把间隙空转掉，输出轴才动。所以**按运动方向**
 *           把指令多给一个回差量，让间隙提前被吃掉，输出轴最终落点仍等于目标：
 *               往正方向走：指令 = 目标 + b
 *               往负方向走：指令 = 目标 − b
 *           同向连续移动不受影响（间隙已经吃在同一个方向上）。
 *
 * ★这是"补偿"，不是"消除"：用手推仍然能推动那个机械间隙，那是物理量，
 *   软件不可能消除。补偿只保证"命令多少度，输出轴就真的停多少度"。
 *
 * ★标定方法（用位置回读，不需要量具）：
 *     1) 下发 a = {90, ...}，等 Act_ArmAtTarget() 为真，记下读数 P1
 *     2) 下发 a = {100, ...}（同向 +10），等到位，记 P2   → 同向应当 P2-P1 ≈ 10
 *     3) 再下发 a = {90, ...}（反向 −10），等到位，记 P3
 *     4) 回差 b ≈ P3 − 90        （反向少走多少，就是间隙多少）
 *   重复 2)~4) 两三次取平均。每个关节单独标。
 *
 * ★默认 0.0f = 不补偿（保持原行为）。标出来再填。
 */
#define ACT_ARM_BL_0     0.0f     /* ARM0 回差（度） */
#define ACT_ARM_BL_1     0.0f     /* ARM1 回差（度） */
#define ACT_ARM_BL_2     0.0f     /* ARM2 回差（度） */
#define ACT_ARM_BL_3     0.0f     /* ARM3 回差（度） */

static const float s_arm_backlash[ACT_ARM_NUM] =
{
  ACT_ARM_BL_0, ACT_ARM_BL_1, ACT_ARM_BL_2, ACT_ARM_BL_3
};

/* 导航（P 控制位置 + PD 控制航向，走世界坐标） */
#define NAV_PERIOD_MS    10u
#define NAV_KP_POS       2.0f     /* 位置误差(mm) -> 速度(mm/s) 的比例 */
#define NAV_VMAX_MMS     250.0f   /* 平移速度上限 */
#define NAV_VMIN_MMS     3.0f    /* 平移最小速度（防静摩擦） */

/*
 * 航向环：ω = KP·角度误差 − KD·陀螺角速度
 *
 * 为什么 KP 从 2.5 提到 7.0：
 *   横移时麦轮侧向摩擦会给车体持续约 9~10°/s 的偏航扰动。
 *   纯 P 的稳态误差满足  KP × 误差 = 扰动率，
 *   原来 KP=2.5 时误差停在 9.5/2.5 ≈ 3.8°（实测峰值 4.73°，吻合），
 *   而 x 方向的偏移基本等于 "y·sin(误差)"，于是 90mm 横移就偏出 7mm 多。
 *   KP=7.0 后同样的扰动只留 9.5/7.0 ≈ 1.4°，x 偏移降到 2mm 量级。
 *
 * 为什么同时加 KD：
 *   KP 提到 7.0 后单靠 P 会过冲/来回摆。用陀螺的 gz 做角速度阻尼，
 *   扰动一出现就有反向力矩压住，不必等角度误差攒起来。
 *   ★如果还是一片一片地晃 → 加大 KD；如果回正时来回摆 → 减小 KP。
 */
#define NAV_KP_YAW       5.0f     /* 角度误差(°) -> 角速度(°/s) */
#define NAV_KD_YAW       0.40f    /* 角速度(°/s) -> 角速度(°/s) 的阻尼 */
#define NAV_WMAX_DPS     120.0f   /* 自转角速度上限 */
#define NAV_TOL_MM       10.0f    /* 到位判据：距目标 */
#define NAV_TOL_DEG      2.0f     /* 到位判据：朝向 */
/*
 * 到位判据还要求**转速已经降下来**。
 *   以前只看角度误差，于是车还在 100°/s 旋转时（因为执行器滞后），
 *   角度一度进过 ±2° 就判 DONE 并停车，之后靠惯性冲出去 11.6° ——
 *   实测原地转 90° 冲到 101.6° 就是这么来的。
 */
#define NAV_TOL_RATE_DPS 30.0f    /* 到位判据：角速度 */
/*
 * 朝向误差大于这个值时，**先原地转向、不平移**（两阶段运动）。
 *
 * 为什么需要：
 *   如果边走边转，目标在车尾时导航会下令"倒车过去"。数学上没错，
 *   但麦轮倒车最容易侧滑，动作也不可预测。
 *   典型例子：车在 (0,50) 且朝 +90°，目标 (0,100) —— 目标正好在车尾，
 *   不分阶段时就是倒车 50mm；分阶段后会先转到 0° 再横移过去。
 *
 *   调大 → 更常边走边转（快，但会斜着飘）
 *   调小 → 更常先转后走（稳，多花一点时间）
 *   设 0  → 完全恢复"边走边转"的老行为
 */
#define NAV_TURN_FIRST_DEG   20.0f
#define NAV_DEG2RAD      (3.14159265f / 180.0f)

static uint8_t  s_nav_active = 0u;
static uint8_t  s_nav_res    = (uint8_t)ACT_DONE;
static float    s_nav_x      = 0.0f;
static float    s_nav_y      = 0.0f;
static float    s_nav_yaw    = 0.0f;
static uint32_t s_nav_t      = 0u;

static float Act_Wrap180(float d)
{
  while (d > 180.0f)  { d -= 360.0f; }
  while (d < -180.0f) { d += 360.0f; }
  return d;
}

static void nav_step(void);


/* ==================== 系统 ==================== */

void Act_Init(void)
{
  s_nav_active = 0u;
  s_nav_res    = (uint8_t)ACT_DONE;
  s_nav_x      = 0.0f;
  s_nav_y      = 0.0f;
  s_nav_yaw    = 0.0f;
  s_grip       = 0u;
  s_arm_cmd_valid = 0u;      /* 上电后第一次 Act_ArmSetAngles 会全部下发 */

  Act_Log("[ACT] 动作层就绪\r\n");
}

void Act_Task(void)
{
  nav_step();                        /* 导航闭环（没在导航时立即返回） */
}

uint32_t Act_Now(void)
{
  return HAL_GetTick();
}

void Act_Log(const char *fmt, ...)
{
  static char buf[128];
  va_list ap;
  int n;

  if ((fmt == NULL) || (g_act_verbose == 0u))
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

/* ==================== 导航 ==================== */

void Act_Move(float vx_mms, float vy_mms, float w_dps)
{
  /* 每次检查：只要当前不是速度模式就切过去（幂等，不会每拍清零目标） */
  if (g_chassis_mode != (uint8_t)CHASSIS_MODE_VELOCITY)
  {
    Chassis_SetMode(CHASSIS_MODE_VELOCITY);
  }
  Chassis_SetVelocity(vx_mms, vy_mms, w_dps);
}

void Act_NavGoto(float x_mm, float y_mm, float yaw_deg, uint32_t tmo_ms)
{
  (void)tmo_ms;                      /* 不设超时兜底：要限时请在自己的规划里记时间 */

  float tyaw = Act_Wrap180(yaw_deg);

  /*
   * 到位后又用同一个目标反复调用 → 不再重新驱动。
   * 否则每拍都会重新激活导航、发现"已在容差内"再调一次 Act_Stop()，
   * 串口就会刷一串 "[ACT] stop"。
   * 但如果确实漂出容差了（惯性冲过头），仍然要重新驱动去纠正。
   */
  if ((x_mm == s_nav_x) && (y_mm == s_nav_y) && (tyaw == s_nav_yaw) &&
      (s_nav_res == (uint8_t)ACT_DONE))
  {
    float px, py, pth;

    Pose_Get(&px, &py, &pth);
    if ((fabsf(px - s_nav_x) < (2.0f * NAV_TOL_MM)) &&
        (fabsf(py - s_nav_y) < (2.0f * NAV_TOL_MM)) &&
        (fabsf(Act_Wrap180(pth - s_nav_yaw)) < (2.0f * NAV_TOL_DEG)))
    {
      return;
    }
  }

  /* 目标变了才重置结果；重复设同一个目标是幂等的（可以在 Plan_Task 里每拍调） */
  if ((x_mm != s_nav_x) || (y_mm != s_nav_y) || (tyaw != s_nav_yaw))
  {
    s_nav_res = (uint8_t)ACT_RUNNING;
  }
  s_nav_x = x_mm;
  s_nav_y = y_mm;
  s_nav_yaw = tyaw;
  s_nav_active = 1u;
  s_nav_t = HAL_GetTick();
}

ActResult_t Act_NavResult(void)
{
  return (ActResult_t)s_nav_res;
}

/* 每拍跑一次：位置误差(世界系) → 转到车体 → 速度；朝向误差 → 角速度 */
static void nav_step(void)
{
  float x, y, th, ex, ey, d, verr, v, w;
  float c, s, exb, eyb;

  if (s_nav_active == 0u)
  {
    return;
  }
  if ((HAL_GetTick() - s_nav_t) < NAV_PERIOD_MS)
  {
    return;
  }
  s_nav_t = HAL_GetTick();

  Pose_Get(&x, &y, &th);

  ex = s_nav_x - x;
  ey = s_nav_y - y;
  d  = sqrtf((ex * ex) + (ey * ey));
  verr = Act_Wrap180(s_nav_yaw - th);

  /* 到位：距离、朝向、**转速**三者都进容差才算完成 */
  if ((d < NAV_TOL_MM) && (fabsf(verr) < NAV_TOL_DEG) &&
      (fabsf(g_hwt101.gz) < NAV_TOL_RATE_DPS))
  {
    Act_Stop();
    s_nav_active = 0u;
    s_nav_res = (uint8_t)ACT_DONE;
    return;
  }

  /* 世界系误差 -> 车体系（x 前、y 左、θ 逆时针为正，标准右手系）
   *   ★必须和 app_pose.c 的旋转矩阵互为转置：
   *       dx_b =  cosθ·dx_w + sinθ·dy_w
   *       dy_b = -sinθ·dx_w + cosθ·dy_w
   */
  c = cosf(th * NAV_DEG2RAD);
  s = sinf(th * NAV_DEG2RAD);
  exb = (c * ex) + (s * ey);
  eyb = (-s * ex) + (c * ey);

  /* 平移：按误差方向给速度，限幅 + 最小速度 */
  v = NAV_KP_POS * d;
  if (v > NAV_VMAX_MMS) { v = NAV_VMAX_MMS; }
  if ((d > NAV_TOL_MM) && (v < NAV_VMIN_MMS)) { v = NAV_VMIN_MMS; }

  if (d > 0.001f)
  {
    exb = exb / d * v;
    eyb = eyb / d * v;
  }
  else
  {
    exb = 0.0f;
    eyb = 0.0f;
  }

  /* 两阶段：朝向误差大时先原地转向，不平移（否则会倒车过去或斜着飘） */
  if (fabsf(verr) > NAV_TURN_FIRST_DEG)
  {
    exb = 0.0f;
    eyb = 0.0f;
  }

  /* 转向：P（角度误差）+ D（陀螺角速度阻尼） */
  w = (NAV_KP_YAW * verr) - (NAV_KD_YAW * g_hwt101.gz);
  if (w > NAV_WMAX_DPS)  { w = NAV_WMAX_DPS; }
  if (w < -NAV_WMAX_DPS) { w = -NAV_WMAX_DPS; }

  Act_Move(exb, eyb, w);
}

void Act_Stop(void)
{
  s_nav_active = 0u;                 /* 停车同时取消导航，否则下一拍又会开回去 */
  if (s_nav_res == (uint8_t)ACT_RUNNING)
  {
    s_nav_res = (uint8_t)ACT_FAIL;   /* 导航被外部中止 */
  }
  Chassis_Stop();
  Act_Log("[ACT] stop\r\n");
}

void Act_PoseGet(float *x_mm, float *y_mm, float *yaw_deg)
{
  /* 世界坐标（绝对坐标）：里程计经陀螺仪航向旋转后的累计值。
   * 原点 = 上电位置，x 前 / y 右，yaw 为连续角度。 */
  Pose_Get(x_mm, y_mm, yaw_deg);
}

/* ==================== 机械臂：位置回读 ==================== */

/* 臂舵机 ID 表（下标 = Act_ArmAngle 的 idx） */
static const uint32_t s_arm_id[ACT_ARM_NUM] =
{
  SERVO_ID_ARM0, SERVO_ID_ARM1, SERVO_ID_ARM2, SERVO_ID_ARM3
};

bool Act_ArmAngle(uint8_t idx, float *deg)
{
  if (idx >= ACT_ARM_NUM)
  {
    return false;
  }
  return ServoBus_GetAngle(s_arm_id[idx], deg);
}

bool Act_GripAngle(float *deg)
{
  return ServoBus_GetAngle(SERVO_ID_GRIPPER, deg);
}

uint32_t Act_ArmRxCount(void)
{
  return ServoBus_GetRxCount();
}

/* ==================== 机械臂：动作 ==================== */

void Act_ArmSetAngles(const float deg[ACT_ARM_NUM], uint8_t speed)
{
  uint8_t i;
  uint8_t sent = 0u;

  if (deg == NULL)
  {
    return;
  }

  for (i = 0u; i < ACT_ARM_NUM; i++)
  {
    float target = deg[i];
    float cmd    = target;

    /* 上电后第一次全部下发；之后只有变化超过 0.2° 才发 */
    if ((s_arm_cmd_valid != 0u) && (fabsf(target - s_arm_cmd[i]) < 0.2f))
    {
      continue;
    }

    /*
     * 回差补偿：按运动方向把指令多给一个回差量（见 s_arm_backlash 的说明）。
     * 上电后第一次不知道该往哪个方向逼近，就按目标相对"当前位置"的方向判断；
     * 若读不到当前位置，则不做补偿，直接发目标。
     */
    if (s_arm_backlash[i] > 0.01f)
    {
      float base;

      if ((s_arm_cmd_valid != 0u) && (fabsf(s_arm_cmd[i]) > 0.0001f))
      {
        base = s_arm_cmd[i];                    /* 上一次的目标角度 */
      }
      else if (!ServoBus_GetAngle(s_arm_id[i], &base))
      {
        base = target;                          /* 读不到就读不让，不补偿 */
      }

      if (target > (base + 0.01f))
      {
        cmd = target + s_arm_backlash[i];       /* 正向逼近 */
      }
      else if (target < (base - 0.01f))
      {
        cmd = target - s_arm_backlash[i];       /* 反向逼近 */
      }
    }

    (void)ServoBus_SetAngle(s_arm_id[i], cmd, speed);
    s_arm_cmd[i] = target;                      /* 记住的是真实目标，不是加了偏置的指令 */
    sent++;
  }

  s_arm_cmd_valid = 1u;

  if (sent != 0u)
  {
    Act_Log("[ARM] cmd %.1f %.1f %.1f %.1f  spd=%u\r\n",
            (double)s_arm_cmd[0], (double)s_arm_cmd[1],
            (double)s_arm_cmd[2], (double)s_arm_cmd[3],
            (unsigned)speed);
  }
}

bool Act_ArmAtTarget(float tol_deg)
{
  uint8_t i;

  if (s_arm_cmd_valid == 0u)
  {
    return false;                      /* 还没设过目标 */
  }

  for (i = 0u; i < ACT_ARM_NUM; i++)
  {
    float now;

    if (!ServoBus_GetAngle(s_arm_id[i], &now))
    {
      return false;                    /* 收不到该关节的位置回传 */
    }
    if (fabsf(now - s_arm_cmd[i]) > tol_deg)
    {
      return false;
    }
  }
  return true;
}

void Act_ArmMove(ActZone_t zone, int8_t ring, const ActBias_t *bias, uint8_t stack)
{
  (void)zone;
  (void)ring;
  (void)bias;
  (void)stack;

  /* TODO: 调你的臂逆解，例如：
   *   Arm_MoveTo(标定表[zone][ring] + bias);
   */
  Act_Log("[ARM] 未实现：Act_ArmMove 里还没接臂逆解\r\n");
}

void Act_ArmDescend(ActZone_t zone, int8_t ring, uint8_t stack)
{
  (void)zone;
  (void)ring;
  (void)stack;

  /* TODO: 垂直下放（只走 Z，不要带水平分量） */
  Act_Log("[ARM] 未实现：Act_ArmDescend\r\n");
}

void Act_ArmLift(float mm)
{
  (void)mm;

  /* TODO: 抬起到指定高度 */
  Act_Log("[ARM] 未实现：Act_ArmLift\r\n");
}

ActResult_t Act_ArmResult(void)
{
  /* TODO: 换成你的臂到位判定，例如 return (ActResult_t)Arm_GetArrive(); */
  return ACT_FAIL;
}

/* 夹爪走总线舵机，这条是真实实现 */
void Act_GripOpen(void)
{
  (void)ServoBus_SetAngle(SERVO_ID_GRIPPER, 60.0f, 0x80u);
  s_grip = 0u;
}

void Act_GripClose(void)
{
  (void)ServoBus_SetAngle(SERVO_ID_GRIPPER, 120.0f, 0x80u);
  s_grip = 1u;
}

uint8_t Act_GripState(void)
{
  /* 现在返回的是"最后一次下发的指令"，不是真实夹持状态。
   * TODO: 接夹空判定（行程开关 / 负压 / 爪内光电）后改成真实状态。 */
  return s_grip;
}

/* ==================== 视觉（底层模块还没有） ==================== */

bool Act_VisCodeStart(void)
{
  /* TODO: 向视觉模块发读码请求 */
  Act_Log("[VIS] 未实现：读码\r\n");
  return false;                      /* false = 发不出去 */
}

bool Act_VisCodePoll(char out[20])
{
  (void)out;
  return false;
}

bool Act_VisBiasStart(ActZone_t zone, uint8_t idx)
{
  (void)zone;
  (void)idx;

  /* TODO: 向视觉模块发测量请求 */
  Act_Log("[VIS] 未实现：偏置测量\r\n");
  return false;
}

bool Act_VisBiasPoll(ActBias_t *out)
{
  if (out != NULL)
  {
    out->dx     = 0.0f;
    out->dy     = 0.0f;
    out->eta_ms = 0.0f;
  }
  return false;
}

bool Act_VisVerifyStart(ActZone_t zone, int8_t ring)
{
  (void)zone;
  (void)ring;

  /* TODO: 向视觉模块发"回看确认"请求 */
  return false;
}

bool Act_VisVerifyPoll(uint8_t *ok)
{
  if (ok != NULL)
  {
    *ok = 0u;
  }
  return false;
}

/* ==================== 显示（暂用 UART5） ==================== */

void Act_DispCode(const char *code)
{
  Act_Log("[DISP] CODE: %s\r\n", (code != NULL) ? code : "----");
}

void Act_DispCounts(uint16_t a, uint16_t b)
{
  Act_Log("[DISP] %u / %u\r\n", (unsigned)a, (unsigned)b);
}

void Act_DispMsg(const char *msg)
{
  Act_Log("[DISP] %s\r\n", (msg != NULL) ? msg : "");
}
